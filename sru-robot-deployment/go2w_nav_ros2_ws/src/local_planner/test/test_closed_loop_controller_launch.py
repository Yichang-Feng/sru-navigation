"""闭环控制接口回归：前馈跟踪、偏航冻结、停车样条、里程计超时及终点对齐。"""
import copy
import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from local_planner.msg import Bspline
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package="local_planner", executable="closed_loop_controller", name="closed_loop_controller",
        parameters=[{"max_vx": 1.0}], output="screen")
    return launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()])


class TestClosedLoopController(unittest.TestCase):
    def test_tracking_and_stop_contract(self):
        rclpy.init()
        node = rclpy.create_node("test_closed_loop_io")
        spline_pub = node.create_publisher(Bspline, "planning/bspline", 10)
        odom_pub = node.create_publisher(Odometry, "body_pose", qos_profile_sensor_data)
        goal_pub = node.create_publisher(Path, "global_path", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        commands, freezes = [], []
        subscriptions = [node.create_subscription(Twist, "cmd_vel", commands.append, 10),
                         node.create_subscription(Bool, "planning/go2_execution_frozen", freezes.append, 10)]
        trajectory_id = 0

        def spline(axis="x", interval=1.0, stationary=False):
            nonlocal trajectory_id
            trajectory_id += 1
            msg = Bspline()
            msg.order, msg.traj_id = 3, trajectory_id
            msg.start_time = node.get_clock().now().to_msg()
            msg.knots = [(i - 3) * interval for i in range(11)]
            for i in range(7):
                msg.pos_pts.append(Point(x=2.0 if stationary else (i * 0.2 if axis == "x" else 0.0),
                                         y=2.0 if stationary else (i * 0.2 if axis == "y" else 0.0), z=0.5))
            return msg

        def pump(seconds, curve=None, odometry=True, yaw=0.0, x=0.0, frame="world", stale=False,
                 bad_quaternion=False, refresh=True):
            end = time.monotonic() + seconds
            begin = len(commands)
            next_publish = 0.0
            while time.monotonic() < end:
                send = time.monotonic() >= next_publish
                if send:
                    next_publish = time.monotonic() + 0.02
                if odometry and send:
                    odom = Odometry()
                    odom.header.frame_id = frame
                    odom.header.stamp = node.get_clock().now().to_msg()
                    if stale:
                        odom.header.stamp.sec -= 2
                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.z = 0.5
                    if not bad_quaternion:
                        odom.pose.pose.orientation.z = math.sin(yaw / 2)
                        odom.pose.pose.orientation.w = math.cos(yaw / 2)
                    else:
                        odom.pose.pose.orientation.w = 0.0
                    odom_pub.publish(odom)
                if curve is not None and send:
                    message = copy.deepcopy(curve)
                    if refresh:
                        message.start_time = node.get_clock().now().to_msg()
                    spline_pub.publish(message)
                rclpy.spin_once(node, timeout_sec=0.005)
            return commands[begin:]

        def stopped():
            self.assertTrue(commands)
            self.assertEqual(commands[-1].linear.x, 0.0)
            self.assertEqual(commands[-1].linear.y, 0.0)
            self.assertEqual(commands[-1].angular.z, 0.0)

        try:
            deadline = time.monotonic() + 8
            while spline_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertGreater(spline_pub.get_subscription_count(), 0)
            pump(0.3)
            stopped()
            moving = pump(0.3, curve=spline())
            self.assertTrue(any(msg.linear.x > 0 for msg in moving))

            invalid = spline()
            invalid.knots[5] = invalid.knots[4]
            pump(0.2, curve=invalid)
            stopped()
            pump(0.2, curve=spline(stationary=True))
            stopped()  # 即使停车点远离当前位置，也不能用位置反馈驶向该点。
            pump(0.2, curve=spline())
            pump(0.45, odometry=False)
            stopped()
            pump(0.2)
            self.assertGreater(commands[-1].linear.x, 0.0)
            for arguments in ({"frame": "map"}, {"stale": True}, {"bad_quaternion": True}):
                pump(0.2, **arguments)
                stopped()
                pump(0.15)

            pump(0.3, curve=spline(axis="y"))
            self.assertEqual(commands[-1].linear.x, 0.0)
            self.assertEqual(commands[-1].linear.y, 0.0)
            self.assertGreater(commands[-1].angular.z, 0.0)
            self.assertTrue(freezes[-1].data)
            pump(0.3, curve=spline(axis="y"), yaw=math.pi / 2)
            self.assertGreater(commands[-1].linear.x, 0.0)
            self.assertFalse(freezes[-1].data)

            expired = spline()
            expired.start_time.sec -= 10
            pump(0.2, curve=expired, refresh=False)
            stopped()

            goal = Path()
            goal.header.frame_id = "world"
            final = PoseStamped()
            final.header = goal.header
            final.pose.position.x, final.pose.position.z = 1.0, 0.5
            final.pose.orientation.z = math.sin(math.pi / 4)
            final.pose.orientation.w = math.cos(math.pi / 4)
            goal.poses = [final]
            goal_pub.publish(goal)
            pump(0.2, x=1.0)
            spline_pub.publish(spline(interval=0.05))
            pump(0.5, x=1.0)
            self.assertEqual(commands[-1].linear.x, 0.0)
            self.assertGreater(commands[-1].angular.z, 0.0)
            pump(0.3, x=1.0, yaw=math.pi / 2)
            stopped()
            for command in commands:
                self.assertTrue(all(math.isfinite(v) for v in
                                    (command.linear.x, command.linear.y, command.angular.z)))
                self.assertLessEqual(abs(command.linear.x), 1.0)
                self.assertLessEqual(abs(command.linear.y), 0.35)
                self.assertLessEqual(abs(command.angular.z), 1.0)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
