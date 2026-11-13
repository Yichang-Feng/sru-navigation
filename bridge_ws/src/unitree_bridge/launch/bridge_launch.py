from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    # 启动节点
    unitree_bridge_node = Node(
        package='unitree_bridge',
        executable='unitree_bridge',
        name='unitree_bridge',
        output='screen'
    )

    sport_mode_subscriber_node = Node(
            package='unitree_bridge',
            executable='sport_mode_subscriber',
            name='sport_mode_subscriber',
            output='screen'
        )

    # 启动ros1_bridge
    parameter_bridge_node = ExecuteProcess(
        cmd=['ros2', 'run', 'ros1_bridge', 'parameter_bridge'],
        output='screen'
    )

    return LaunchDescription([
        unitree_bridge_node,
        parameter_bridge_node,
        sport_mode_subscriber_node
    ]) 