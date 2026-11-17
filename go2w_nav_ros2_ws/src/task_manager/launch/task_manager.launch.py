"""Launch sequential mission executor with localization status gating."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from task_manager.mission_executor import DEFAULTS


def generate_launch_description():
    arguments = []
    parameters = {}
    for name, default in DEFAULTS.items():
        arguments.append(DeclareLaunchArgument(name, default_value=str(default)))
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=type(default))
    return LaunchDescription(arguments + [Node(
        package='task_manager', executable='task_manager_node', name='task_manager',
        output='screen', parameters=[parameters],
    )])
