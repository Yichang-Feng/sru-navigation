#!/usr/bin/env python3
"""ROS 2 transport for the mission protocol v1 executor."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point

from task_manager.mission_executor import DEFAULTS, MissionExecutor, finite
from task_manager.path_distance import RemainingPath, valid_point


def make_pose(step, frame, stamp):
    pose = PoseStamped()
    pose.header.frame_id, pose.header.stamp = frame, stamp
    pose.pose.position.x = float(step['x'])
    pose.pose.position.y = float(step['y'])
    pose.pose.position.z = float(step['z'])
    pose.pose.orientation.z = math.sin(step['yaw'] / 2.0)
    pose.pose.orientation.w = math.cos(step['yaw'] / 2.0)
    return pose


class TaskManagerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('task_manager', **kwargs)
        config = {name: self.declare_parameter(name, value).value
                  for name, value in DEFAULTS.items()}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.status_pub = self.create_publisher(String, config['status_topic'], qos)
        self.control_pub = self.create_publisher(String, config['control_topic'], qos)
        self.goal_pub = self.create_publisher(PoseStamped, config['goal_topic'], qos)
        self.path_distance = RemainingPath()
        self.path_frame = None
        self.goal_sent_stamp = self.path_stamp = None
        self.last_odom = self.odom_received = None
        self.localization_issue = ''
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.executor_core = MissionExecutor(
            self, self.publish_status, time.monotonic, self.stamp,
            self.get_logger().warning, config, control=self.publish_control)
        self.local_planner_status_sub = self.create_subscription(
            String, config['local_planner_status_topic'], self.local_planner_status_cb, qos)
        # Volatile request feedback must never replay an old goal's result.
        self.global_planner_status_sub = self.create_subscription(
            String, config['global_planner_status_topic'], self.global_planner_status_cb, qos)
        # A separate durable reader handles only node-wide health, including
        # a map-load fault published before this executor was started.
        health_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.localization_status_sub = self.create_subscription(
            String, config['localization_status_topic'], self.localization_status_cb, health_qos)
        self.global_planner_health_sub = self.create_subscription(
            String, config['global_planner_status_topic'],
            self.global_planner_health_cb, health_qos)
        path_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(
            Path, config['global_path_topic'], self.path_cb, path_qos)
        self.odom_sub = self.create_subscription(
            Odometry, config['odom_topic'], self.odom_cb, qos_profile_sensor_data)
        self.command_sub = self.create_subscription(
            String, config['command_topic'],
            lambda msg: self.executor_core.command(msg.data), qos)
        self.estop_sub = self.create_subscription(
            Bool, '/estop', lambda msg: self.executor_core.set_estop(msg.data), qos)
        # Wall/steady scheduling also works when simulated ROS time is stopped.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        rate = config['status_rate_hz']
        period = min(0.05, 1.0 / rate) if finite(rate) and rate > 0 else 0.05
        self.timer = self.create_timer(period, self.tick, clock=self.steady_clock)

    def stamp(self):
        msg = self.get_clock().now().to_msg()
        return {'sec': msg.sec, 'nanosec': msg.nanosec}

    def publish_status(self, status):
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        self.get_logger().info('Publishing status: ' + msg.data)
        self.status_pub.publish(msg)

    def publish_control(self, command):
        self.control_pub.publish(String(data=command))

    def send(self, step, frame):
        stamp = self.get_clock().now()
        self.path_distance.clear()
        self.path_frame = self.path_stamp = None
        self.goal_sent_stamp = stamp.nanoseconds
        self.goal_pub.publish(make_pose(step, frame, stamp.to_msg()))

    def local_planner_status_cb(self, msg):
        self.executor_core.local_status_json(msg.data)

    def localization_status_cb(self, msg):
        self.executor_core.localization_status_json(msg.data)

    def global_planner_status_cb(self, msg):
        status = msg.data
        if status != 'ready' and not status.startswith('map_load_failed:'):
            self.executor_core.global_status(status)

    def global_planner_health_cb(self, msg):
        status = msg.data
        if status == 'ready' or status.startswith('map_load_failed:'):
            self.executor_core.global_status(status)

    def point_in_frame(self, point, source, target, stamp):
        if not valid_point(point) or not source or not target:
            return None
        if source == target:
            return tuple(point)
        try:
            msg = PointStamped()
            msg.header.frame_id, msg.header.stamp = source, stamp
            msg.point.x, msg.point.y, msg.point.z = map(float, point)
            transform = self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp))
            transformed = do_transform_point(msg, transform).point
            result = transformed.x, transformed.y, transformed.z
            return result if valid_point(result) else None
        except TransformException:
            return None

    def path_cb(self, msg):
        core = self.executor_core
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if (core.step is None or core.token is None or self.goal_sent_stamp is None or
                stamp < self.goal_sent_stamp or
                (self.path_stamp is not None and stamp <= self.path_stamp)):
            return
        self.path_distance.clear()
        self.path_frame = None
        self.path_stamp = stamp
        core.distance = None
        if not msg.poses or not msg.header.frame_id:
            return
        if any(p.header.frame_id and p.header.frame_id != msg.header.frame_id for p in msg.poses):
            return
        step = core.step
        # The global planner appends the exact goal plus its configured Z offset.
        goal = self.point_in_frame(
            (step['x'], step['y'], step['z']), core.frame, msg.header.frame_id,
            Time().to_msg())
        if goal is None:
            return
        expected = goal[0], goal[1], goal[2] + core.config['global_path_z_offset']
        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        if not valid_point(points[-1]) or math.dist(points[-1], expected) > .05:
            return
        if self.path_distance.set_path(points):
            self.path_frame = msg.header.frame_id
            core.distance = self.remaining_distance()
            core.path_available()

    def odom_cb(self, msg):
        self.last_odom, self.odom_received = msg, time.monotonic()
        self.executor_core.distance = self.remaining_distance()

    def remaining_distance(self):
        core = self.executor_core
        if core.state not in ('running', 'paused', 'canceling') or core.step is None:
            return None
        if core.index in core.results and core.results[core.index]['outcome'] == 'reached':
            return 0.0
        if core.token is None or self.path_frame is None or self.last_odom is None:
            return None
        odom = self.last_odom
        age = (self.get_clock().now().nanoseconds -
               Time.from_msg(odom.header.stamp).nanoseconds) / 1e9
        limit = core.config['tf_timeout_sec']
        if not 0 <= age <= limit or time.monotonic() - self.odom_received > limit:
            return None
        p = odom.pose.pose.position
        robot = self.point_in_frame((p.x, p.y, p.z), odom.header.frame_id,
                                    self.path_frame, odom.header.stamp)
        return self.path_distance.remaining(robot) if robot is not None else None

    def localization_age(self):
        core = self.executor_core
        pair = '{} -> {}'.format(core.frame, core.config['robot_frame'])
        self.localization_issue = ''
        try:
            transform = self.tf_buffer.lookup_transform(
                core.frame, core.config['robot_frame'], Time())
            stamp = transform.header.stamp
            # A zero stamp denotes an entirely static TF chain.
            if stamp.sec == 0 and stamp.nanosec == 0:
                return 0.0
            age = (self.get_clock().now().nanoseconds -
                   Time.from_msg(stamp).nanoseconds) / 1e9
            if age < 0:
                self.localization_issue = 'TF {} is in the future (age={:.3f}s)'.format(pair, age)
                return None
            if age > core.config['tf_timeout_sec']:
                self.localization_issue = 'TF {} is stale (age={:.3f}s, limit={:.3f}s)'.format(
                    pair, age, core.config['tf_timeout_sec'])
            return age
        except TransformException as exc:
            self.localization_issue = 'TF {} unavailable: {}'.format(pair, exc)
            return None

    def tick(self):
        core = self.executor_core
        active = core.state in ('starting', 'running', 'paused', 'canceling')
        age = self.localization_age() if active else 0.0
        if self.localization_issue and active:
            self.get_logger().warning(self.localization_issue, throttle_duration_sec=2.0)
        core.distance = self.remaining_distance()
        core.tick(age is not None and age <= core.config['tf_timeout_sec'], age)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
