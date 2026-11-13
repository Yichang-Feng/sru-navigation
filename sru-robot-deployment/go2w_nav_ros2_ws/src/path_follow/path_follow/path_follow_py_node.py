#!/usr/bin/env python3
"""Generate body-frame Twist commands from a configured global or local path."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import tf2_geometry_msgs  # noqa: F401; registers PoseStamped with TF2
from tf2_ros import Buffer, TransformException, TransformListener

from path_follow.controller import ControllerConfig, PathController


class PathFollower(Node):
    def __init__(self, **kwargs):
        super().__init__('path_follower', **kwargs)
        self.settings = {}
        defaults = {
            'path_source': 'local', 'global_path_topic': '/global_path',
            'local_path_topic': '/local_path', 'path_topic': '',
            'odom_topic': '/lio/robo/odom', 'cmd_vel_topic': '/cmd_vel',
            'global_frame': 'map', 'local_frame': 'world', 'odom_frame': 'world',
            'control_frequency': 20.0, 'odom_timeout': 0.30, 'local_path_timeout': 0.50,
            'global_path_timeout': 0.0, 'tf_timeout': 0.30,
            'tf_jump_translation': 0.20, 'tf_jump_angle_degrees': 5.0,
        }
        defaults.update(vars(ControllerConfig()))
        for name, default in defaults.items():
            self.settings[name] = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
        s = self.settings
        if s['path_source'] not in ('global', 'local'):
            raise ValueError('path_source must be global or local')
        for name in ('control_frequency', 'odom_timeout', 'local_path_timeout', 'tf_timeout',
                     'tf_jump_translation', 'tf_jump_angle_degrees'):
            if not math.isfinite(s[name]) or s[name] <= 0:
                raise ValueError('%s must be finite and positive' % name)
        if not math.isfinite(s['global_path_timeout']) or s['global_path_timeout'] < 0:
            raise ValueError('global_path_timeout must be finite and nonnegative')
        for name in ('global_path_topic', 'local_path_topic', 'odom_topic', 'cmd_vel_topic',
                     'global_frame', 'local_frame', 'odom_frame'):
            if not s[name]:
                raise ValueError('%s cannot be empty' % name)
        if s['path_topic']:
            s[s['path_source']+'_path_topic'] = s['path_topic']
            self.get_logger().warning('path_topic is deprecated; use global_path_topic/local_path_topic')
        if s['global_path_topic'] == s['local_path_topic']:
            raise ValueError('global and local path topics must be distinct')
        self.controller = PathController(ControllerConfig(**{
            name: s[name] for name in vars(ControllerConfig())}))
        self.path = None
        self.odom = None
        self.path_received = None
        self.odom_received = None
        self.last_tick = None
        self.last_ros_time = None
        self.previous_transform = None
        self.last_status = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.transform_pose = self.buffer.registration.get(PoseStamped)
        self.publisher = self.create_publisher(Twist, s['cmd_vel_topic'], 1)
        self.global_sub = self.create_subscription(
            Path, s['global_path_topic'], lambda msg: self.path_callback(msg, 'global'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.local_sub = self.create_subscription(
            Path, s['local_path_topic'], lambda msg: self.path_callback(msg, 'local'),
            QoSProfile(depth=1))
        self.odom_sub = self.create_subscription(
            Odometry, s['odom_topic'], self.odom_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/s['control_frequency'], self.control_callback)
        self.get_logger().info('Following %s path: %s -> %s' % (
            s['path_source'], s[s['path_source']+'_path_topic'], s['cmd_vel_topic']))

    def fresh(self, stamp, timeout):
        try:
            age = (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9
        except (ValueError, TypeError, OverflowError):
            return False
        return -0.05 <= age <= timeout

    def set_status(self, status):
        if status != self.last_status:
            self.get_logger().info(status, throttle_duration_sec=2.0)
            self.last_status = status

    def stop_robot(self, status='stopped'):
        self.controller.stop()
        if self.context.ok():
            self.publisher.publish(Twist())
        self.set_status(status)

    def path_callback(self, msg, source):
        if source != self.settings['path_source']:
            return
        self.path = None
        self.controller.set_path([])
        self.previous_transform = None
        expected = self.settings[source+'_frame']
        if not msg.poses:
            self.stop_robot('path cancelled')
            return
        if msg.header.frame_id != expected or any(
                p.header.frame_id not in ('', expected) for p in msg.poses):
            self.stop_robot('path frame mismatch')
            return
        timeout = self.settings[source+'_path_timeout']
        if timeout > 0 and not self.fresh(msg.header.stamp, timeout):
            self.stop_robot('stale path timestamp')
            return
        points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        try:
            self.controller.set_path(points)
        except ValueError:
            self.stop_robot('invalid path positions')
            return
        self.path = msg
        self.path_received = time.monotonic()

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm_squared = sum(v*v for v in (q.x, q.y, q.z, q.w))
        if (msg.header.frame_id != self.settings['odom_frame'] or
                not all(math.isfinite(v) for v in values) or
                not math.isfinite(norm_squared) or norm_squared < 1e-8 or
                not self.fresh(msg.header.stamp, self.settings['odom_timeout'])):
            self.odom = None
            self.stop_robot('invalid or stale odometry')
            return
        self.odom = msg
        self.odom_received = time.monotonic()

    @staticmethod
    def yaw(q):
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def robot_in_path_frame(self):
        pose = PoseStamped()
        pose.header = self.odom.header
        pose.pose = self.odom.pose.pose
        transform = None
        if self.path.header.frame_id != self.odom.header.frame_id:
            transform = self.buffer.lookup_transform(
                self.path.header.frame_id, self.odom.header.frame_id, Time())
            stamp = transform.header.stamp
            if (stamp.sec != 0 or stamp.nanosec != 0) and not self.fresh(stamp, self.settings['tf_timeout']):
                raise ValueError('stale TF')
            pose = self.transform_pose(pose, transform)
        p = pose.pose.position
        yaw = self.yaw(pose.pose.orientation)
        return (p.x, p.y, p.z), yaw, transform

    def control_callback(self):
        tick = time.monotonic()
        ros_time = self.get_clock().now().nanoseconds
        dt = tick-self.last_tick if self.last_tick is not None else 1.0/self.settings['control_frequency']
        self.last_tick = tick
        if self.last_ros_time is not None and ros_time < self.last_ros_time:
            self.path = None
            self.odom = None
            self.previous_transform = None
            self.controller.set_path([])
        self.last_ros_time = ros_time
        s = self.settings
        if self.path is None or self.odom is None:
            self.stop_robot('waiting for selected path and odometry')
            return
        if (tick-self.odom_received > s['odom_timeout'] or
                not self.fresh(self.odom.header.stamp, s['odom_timeout'])):
            self.stop_robot('odometry timeout')
            return
        timeout = s[s['path_source']+'_path_timeout']
        if timeout > 0 and (tick-self.path_received > timeout or not self.fresh(self.path.header.stamp, timeout)):
            self.stop_robot('path timeout')
            return
        try:
            robot, yaw, transform = self.robot_in_path_frame()
        except (TransformException, ValueError) as error:
            self.stop_robot('TF unavailable: %s' % error)
            return
        # Compare the mapping of the SAME odom pose under consecutive TF samples,
        # so physical robot motion cannot be mistaken for a localization jump.
        if transform is not None:
            try:
                if self.previous_transform is not None:
                    previous = PoseStamped()
                    previous.header = self.odom.header
                    previous.pose = self.odom.pose.pose
                    old = self.transform_pose(previous, self.previous_transform)
                    delta = math.dist(robot, (old.pose.position.x, old.pose.position.y, old.pose.position.z))
                    angle = abs(math.atan2(math.sin(yaw-self.yaw(old.pose.orientation)),
                                           math.cos(yaw-self.yaw(old.pose.orientation))))
                    if delta > s['tf_jump_translation'] or angle > math.radians(s['tf_jump_angle_degrees']):
                        self.controller.reset_progress()
                        self.stop_robot('TF corrected; reacquiring path')
                        self.previous_transform = transform
                        return
                self.previous_transform = transform
            except TransformException:
                self.stop_robot('TF unavailable')
                return
        v, w, status = self.controller.compute(robot, yaw, dt)
        command = Twist()
        command.linear.x = v
        command.angular.z = w
        self.publisher.publish(command)
        self.set_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_robot('shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
