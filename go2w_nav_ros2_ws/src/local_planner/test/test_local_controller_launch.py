"""使用真实 run.launch.py 验证默认闭环链路；所有速度输出重映射至测试话题。"""
import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from launch.launch_description_sources import PythonLaunchDescriptionSource
from local_planner.msg import Bspline
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    entry = os.path.join(get_package_share_directory("local_planner"), "launch", "run.launch.py")
    nodes = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(entry), launch_arguments={
        "body_pose_topic": "/controller_io/odom", "global_path_topic": "/controller_io/path",
        "local_path_topic": "/controller_io/local", "cmd_vel_topic": "/controller_io/cmd",
    }.items())
    return launch.LaunchDescription([nodes, launch_testing.actions.ReadyToTest()])


class TestLocalController(unittest.TestCase):
    def test_default_nodes_freeze_resume_and_planner_stop(self):
        rclpy.init()
        node = rclpy.create_node("test_local_controller_pipeline")
        odom_pub = node.create_publisher(Odometry, "/controller_io/odom", qos_profile_sensor_data)
        path_pub = node.create_publisher(Path, "/controller_io/path", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        commands, freezes, splines = [], [], []
        subscriptions = [node.create_subscription(Twist, "/controller_io/cmd", commands.append, 10),
                         node.create_subscription(Bool, "planning/go2_execution_frozen", freezes.append, 10),
                         node.create_subscription(Bspline, "planning/bspline", splines.append, 10)]

        def pump(seconds, yaw=0.0):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                odom = Odometry()
                odom.header.frame_id = "world"
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.pose.pose.position.z = 1.0
                odom.pose.pose.orientation.z = math.sin(yaw / 2)
                odom.pose.pose.orientation.w = math.cos(yaw / 2)
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.02)

        def reference(frame):
            path = Path()
            path.header.frame_id = frame
            for x in (0.0, 2.0):
                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.pose.position.x, pose.pose.position.z = x, 1.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            return path

        try:
            deadline = time.monotonic() + 8
            while path_pub.get_subscription_count() < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertGreaterEqual(path_pub.get_subscription_count(), 2)
            pump(0.4, yaw=math.pi)
            names = node.get_node_names()
            self.assertIn("local_planner_node", names)
            self.assertIn("closed_loop_controller", names)
            self.assertNotIn("path_follower", names)
            path_pub.publish(reference("world"))
            pump(0.8, yaw=math.pi)
            self.assertTrue(splines)
            self.assertTrue(freezes[-1].data)
            self.assertEqual(commands[-1].linear.x, 0.0)
            self.assertNotEqual(commands[-1].angular.z, 0.0)
            pump(0.6)
            self.assertFalse(freezes[-1].data)
            self.assertGreater(commands[-1].linear.x, 0.0)
            # 新参考系没有 TF，规划器应生成停车 B 样条，控制器收到后必须停止。
            path_pub.publish(reference("missing_map"))
            pump(1.0)
            self.assertEqual(commands[-1].linear.x, 0.0)
            self.assertEqual(commands[-1].linear.y, 0.0)
            self.assertEqual(commands[-1].angular.z, 0.0)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
