"""通过 ROS 消息验证外部全局路径的校验、替换和轨迹输出，不启动机器人。"""

import copy
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from local_planner.msg import Bspline
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml"))
    planner = launch_ros.actions.Node(
        package="local_planner", executable="local_planner_node",
        name="local_planner_node", parameters=[config], output="screen")
    return (launch.LaunchDescription([
        planner, launch_testing.actions.ReadyToTest(),
    ]), {"planner": planner})


class TestGlobalPath(unittest.TestCase):
    def test_path_validation_and_replacement(self, proc_info, proc_output, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=10)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("global_path_contract_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        odom_pub = node.create_publisher(Odometry, "body_pose", qos_profile_sensor_data)
        path_pub = node.create_publisher(Path, "/global_path", 10)
        received = []
        node.create_subscription(Bspline, "/planning/bspline", received.append, 10)

        path = Path()
        path.header.frame_id = "world"
        for x in (0.0, 2.0):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.z = 1.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        def pump(seconds, odometry=True, message=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if odometry:
                    odom = Odometry()
                    odom.header.frame_id = "world"
                    odom.header.stamp = node.get_clock().now().to_msg()
                    odom.pose.pose.position.z = 1.0
                    odom.pose.pose.orientation.w = 1.0
                    odom_pub.publish(odom)
                if message is not None:
                    path_pub.publish(message)
                executor.spin_once(timeout_sec=0.04)

        try:
            deadline = time.monotonic() + 8
            while path_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertGreater(path_pub.get_subscription_count(), 0)

            pump(0.3, odometry=False, message=path)
            proc_output.assertWaitFor("No odometry yet; ignoring global_path", process=planner, timeout=5)
            self.assertFalse(received)
            pump(0.3)
            self.assertFalse(received, "planning started without an accepted path")

            invalid = Path()
            pump(0.15, message=invalid)
            proc_output.assertWaitFor("Received empty global_path", process=planner, timeout=5)
            invalid = copy.deepcopy(path)
            invalid.header.frame_id = ""
            pump(0.15, message=invalid)
            proc_output.assertWaitFor("empty frame_id", process=planner, timeout=5)
            invalid = copy.deepcopy(path)
            invalid.poses[1].header.frame_id = "other"
            pump(0.15, message=invalid)
            proc_output.assertWaitFor("mixed pose frames", process=planner, timeout=5)
            invalid = copy.deepcopy(path)
            invalid.poses[1].pose.position.x = float("nan")
            pump(0.15, message=invalid)
            proc_output.assertWaitFor("non-finite coordinate", process=planner, timeout=5)
            self.assertFalse(received)

            pump(0.3, message=path)
            pump(0.5)
            self.assertTrue(received, "valid /global_path did not produce a trajectory")
            self.assertTrue(any(msg.pos_pts[-1].x > msg.pos_pts[0].x + 0.1 for msg in received))
            last_id = max(msg.traj_id for msg in received)

            updated = copy.deepcopy(path)
            updated.poses[-1].pose.position.y = 0.8
            pump(0.3, message=updated)
            pump(0.5)
            replacements = [msg for msg in received if msg.traj_id > last_id]
            self.assertTrue(replacements, "new path did not trigger replanning")
            self.assertTrue(any(msg.pos_pts[-1].y > 0.1 for msg in replacements))

            publishers = {name for name, _ in node.get_topic_names_and_types()
                          if any(info.node_name == "local_planner_node"
                                 for info in node.get_publishers_info_by_topic(name))}
            self.assertNotIn("/cmd_vel", publishers)
            self.assertNotIn("/body_pose", publishers)
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
