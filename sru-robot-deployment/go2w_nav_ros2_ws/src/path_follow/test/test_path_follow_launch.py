"""Run both selectable modes with real DDS messages and TF."""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'path_follow.yaml'))
    local = launch_ros.actions.Node(
        package='path_follow', executable='path_follow_node', name='path_follower',
        parameters=[config], remappings=[('/cmd_vel', '/test/local_cmd')], output='screen')
    # Also exercise the original executable alias.
    global_node = launch_ros.actions.Node(
        package='path_follow', executable='path_follow_py_node', name='global_follower',
        parameters=[{'path_source': 'global', 'cmd_vel_topic': '/test/global_cmd'}], output='screen')
    return launch.LaunchDescription([
        local, launch.actions.TimerAction(period=1.5, actions=[global_node]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestModes(unittest.TestCase):
    def test_selection_tf_watchdogs_cancel_and_recovery(self):
        rclpy.init()
        node = rclpy.create_node('test_path_follow_io')
        commands = {'local': [], 'global': []}
        subs = [node.create_subscription(Twist, '/test/'+source+'_cmd',
                                         lambda msg, key=source: commands[key].append(msg), 10)
                for source in commands]
        global_pub = node.create_publisher(
            Path, '/global_path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        local_pub = node.create_publisher(Path, '/local_path', 1)
        odom_pub = node.create_publisher(Odometry, '/lio/robo/odom', qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(node)

        def path(frame, y=0.0, empty=False):
            msg = Path()
            msg.header.frame_id = frame
            msg.header.stamp = node.get_clock().now().to_msg()
            if not empty:
                for x in (0.0, 2.0, 4.0):
                    pose = PoseStamped()
                    pose.header.frame_id = frame
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.5
                    pose.pose.orientation.w = 1.0
                    msg.poses.append(pose)
            return msg

        global_pub.publish(path('map'))

        def pump(seconds, local=True, odom=True, tf=True, y=0.0, yaw=0.0, bad_odom=False, stale_local=False):
            begin = {key: len(value) for key, value in commands.items()}
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                stamp = node.get_clock().now().to_msg()
                if odom:
                    msg = Odometry()
                    msg.header.frame_id = 'world'
                    msg.header.stamp = stamp
                    msg.pose.pose.position.z = 0.5
                    msg.pose.pose.orientation.w = 0.0 if bad_odom else 1.0
                    odom_pub.publish(msg)
                if local:
                    msg = path('world')
                    if stale_local:
                        msg.header.stamp.sec -= 5
                    local_pub.publish(msg)
                if tf:
                    msg = TransformStamped()
                    msg.header.frame_id = 'map'
                    msg.child_frame_id = 'world'
                    msg.header.stamp = stamp
                    msg.transform.translation.y = y
                    msg.transform.rotation.z = math.sin(yaw/2)
                    msg.transform.rotation.w = math.cos(yaw/2)
                    broadcaster.sendTransform(msg)
                rclpy.spin_once(node, timeout_sec=0.02)
            return {key: value[begin[key]:] for key, value in commands.items()}

        def stopped(messages):
            self.assertTrue(messages)
            self.assertEqual(messages[-1].linear.x, 0.0)
            self.assertEqual(messages[-1].angular.z, 0.0)

        def moving(messages):
            self.assertTrue(any(msg.linear.x > 0.0 for msg in messages))

        try:
            initial = pump(3.0)
            moving(initial['local'])
            moving(initial['global'])  # Late subscriber receives the one-shot map path.
            corrected = pump(1.0, y=0.5)
            self.assertLess(corrected['global'][-1].angular.z, 0.0)
            self.assertAlmostEqual(corrected['local'][-1].angular.z, 0.0)
            rotated = pump(1.0, yaw=math.pi/2)
            stopped_linear = rotated['global'][-1]
            self.assertEqual(stopped_linear.linear.x, 0.0)
            self.assertLess(stopped_linear.angular.z, 0.0)
            moving(pump(0.8)['global'])
            # No fallback to the still-valid global path when the local producer dies.
            absent = pump(0.9, local=False)
            stopped(absent['local'])
            moving(absent['global'])
            moving(pump(0.7)['local'])
            stale = pump(0.7, stale_local=True)
            stopped(stale['local'])
            moving(stale['global'])
            moving(pump(0.7)['local'])
            absent = pump(0.7, odom=False)
            stopped(absent['local'])
            stopped(absent['global'])
            moving(pump(0.7)['local'])
            invalid = pump(0.7, bad_odom=True)
            stopped(invalid['local'])
            stopped(invalid['global'])
            moving(pump(0.7)['global'])
            absent_tf = pump(0.8, tf=False)
            stopped(absent_tf['global'])
            moving(absent_tf['local'])  # world -> world does not require TF.
            moving(pump(0.7)['global'])
            local_pub.publish(path('world', empty=True))
            cancelled = pump(0.4, local=False)
            stopped(cancelled['local'])
            moving(cancelled['global'])
            global_pub.publish(path('map', empty=True))
            cancelled = pump(0.4)
            stopped(cancelled['global'])
            moving(cancelled['local'])
            global_pub.publish(path('map'))
            moving(pump(0.7)['global'])
            for values in commands.values():
                for msg in values:
                    self.assertGreaterEqual(msg.linear.x, 0.0)
                    self.assertLessEqual(msg.linear.x, 0.5)
                    self.assertLessEqual(abs(msg.angular.z), 1.0)
                    self.assertEqual(msg.linear.y, 0.0)
        finally:
            for sub in subs:
                node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
