"""错误采样参数不得导致崩溃：应受控失败、撤销路径、发布停车轨迹并继续响应服务。"""
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from local_planner.msg import Bspline
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.srv import ListParameters
from rclpy.qos import qos_profile_sensor_data


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml"))
    # Foxy 中具名 YAML 与通配符字典混用时，具名参数可能优先；先合并保证故障参数生效。
    with open(config, encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["local_planner_node"]["ros__parameters"]
    parameters.update({"manager.control_points_distance": 1e-12, "fsm.max_replan_fail_count": 3})
    planner = launch_ros.actions.Node(
        package="local_planner", executable="local_planner_node", name="local_planner_node",
        parameters=[parameters], output="screen")
    return launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]), {"planner": planner}


class TestReplanLimits(unittest.TestCase):
    def test_bounded_failure_keeps_node_alive_and_stops(self, proc_output, planner):
        rclpy.init()
        node = rclpy.create_node("test_replan_limits")
        odom_pub = node.create_publisher(Odometry, "body_pose", qos_profile_sensor_data)
        path_pub = node.create_publisher(Path, "/global_path", 1)
        paths, splines = [], []
        subscriptions = [
            node.create_subscription(Path, "/local_path", paths.append, 10),
            node.create_subscription(Bspline, "planning/bspline", splines.append, 10),
        ]

        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                odom = Odometry()
                odom.header.frame_id = "world"
                odom.header.stamp = node.get_clock().now().to_msg()
                odom.pose.pose.position.z = 1.0
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            deadline = time.monotonic() + 8
            while path_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertGreater(path_pub.get_subscription_count(), 0)
            pump(0.4)
            path = Path()
            path.header.frame_id = "world"
            for x in (0.0, 2.0):
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.z = 1.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            path_pub.publish(path)
            pump(1.0)
            proc_output.assertWaitFor("bounded_sampling_failed", process=planner, timeout=5)
            self.assertTrue(paths)
            self.assertFalse(paths[-1].poses)
            self.assertTrue(splines, "sampling failure did not produce an emergency stop")
            self.assertTrue(all(abs(p.x) < 1e-9 and abs(p.y) < 1e-9 for p in splines[-1].pos_pts))
            client = node.create_client(ListParameters, "/local_planner_node/list_parameters")
            self.assertTrue(client.wait_for_service(timeout_sec=2))
            future = client.call_async(ListParameters.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            self.assertIsNotNone(future.result(), "node stopped responding after rejected sampling")
        finally:
            for sub in subscriptions:
                node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
