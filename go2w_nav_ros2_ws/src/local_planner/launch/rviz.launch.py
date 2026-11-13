"""可选 RViz 可视化；显示全局路径、局部采样路径、规划轨迹和局部地图。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("local_planner"), "rviz", "default.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("fixed_frame", default_value="world", description="显示固定坐标系，应与规划坐标系一致"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", config, "-f", LaunchConfiguration("fixed_frame")]),
    ])
