"""Launch one configurable global/local path follower."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('path_follow'), 'config', 'path_follow.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='path_follow', executable='path_follow_node', name='path_follower',
             output='screen', parameters=[LaunchConfiguration('params_file'),
                                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
