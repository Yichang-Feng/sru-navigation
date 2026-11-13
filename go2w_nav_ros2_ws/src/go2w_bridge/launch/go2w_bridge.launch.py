"""Launch the velocity bridge and wireless-controller reader."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start both a2_bridge nodes."""
    return LaunchDescription([
        Node(
            package='go2w_bridge',
            executable='cmd_vel_bridge',
            name='go2w_cmd_vel_bridge',
            output='screen',
        ),
        Node(
            package='go2w_bridge',
            executable='wireless_controller',
            name='go2w_wireless_controller',
            output='screen',
        ),
    ])
