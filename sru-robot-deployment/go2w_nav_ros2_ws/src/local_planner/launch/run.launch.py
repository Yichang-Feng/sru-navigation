"""启动局部规划与 B 样条闭环控制；定位、传感器和全局路径由外部提供。"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameters(filename, node_name):
    # 先合并 YAML 与启动覆盖项，再作为一个参数字典传入；避免 Foxy 的具名/通配符优先级问题。
    with open(filename, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise RuntimeError("parameter file must contain a ROS node parameter mapping")
    result = dict(document.get("/**", {}).get("ros__parameters", {}))
    result.update(document.get(node_name, {}).get("ros__parameters", {}))
    if not result:
        raise RuntimeError("parameter file has no parameters for " + node_name)
    return result


def _setup(context):
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    parameters = _parameters(LaunchConfiguration("params_file").perform(context), "local_planner_node")
    parameters["grid_map.sensor_type"] = sensor_type
    actions = [Node(
        package="local_planner",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[parameters],
        remappings=[
            ("body_pose", LaunchConfiguration("body_pose_topic")),
            ("sensor_pose", LaunchConfiguration("sensor_pose_topic")),
            ("cloud", LaunchConfiguration("cloud_topic")),
            ("depth", LaunchConfiguration("depth_topic")),
            ("/global_path", LaunchConfiguration("global_path_topic")),
            ("/local_path", LaunchConfiguration("local_path_topic")),
        ],
    )]
    enabled = LaunchConfiguration("enable_controller").perform(context).lower()
    if enabled not in ("true", "false", "1", "0"):
        raise RuntimeError("enable_controller must be true or false")
    if enabled in ("true", "1"):
        controller = _parameters(LaunchConfiguration("controller_params_file").perform(context),
                                 "closed_loop_controller")
        controller["frame_id"] = parameters.get("grid_map.frame_id", "world")
        controller["use_sim_time"] = parameters.get("use_sim_time", False)
        actions.append(Node(
            package="local_planner", executable="closed_loop_controller", name="closed_loop_controller",
            output="screen", parameters=[controller],
            remappings=[
                ("body_pose", LaunchConfiguration("body_pose_topic")),
                ("global_path", LaunchConfiguration("global_path_topic")),
                ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ],
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("local_planner")
    arguments = {
        "params_file": os.path.join(share, "config", "planner.yaml"),
        "controller_params_file": os.path.join(share, "config", "controllers.yaml"),
        "enable_controller": "true",
        "cmd_vel_topic": "/cmd_vel",
        "sensor_type": "lidar",
        "body_pose_topic": "/lio/robo/odom",
        "sensor_pose_topic": "/lio/imu/odom",
        "cloud_topic": "/lio/cloud_world",
        "depth_topic": "/camera/aligned_depth_to_color/image_raw",
        "global_path_topic": "/global_path",
        "local_path_topic": "/local_path",
    }
    # 规划器和控制器使用同一机身里程计及全局路径，冻结反馈在包内自动连接。
    descriptions = {
        "params_file": "规划器 YAML 参数文件，节点键应为 local_planner_node",
        "controller_params_file": "闭环控制器参数文件，节点键为 closed_loop_controller",
        "enable_controller": "是否启动闭环控制器，默认 true；false 时只规划",
        "cmd_vel_topic": "闭环控制器输出的机体速度指令话题",
        "sensor_type": "建图传感器类型：lidar 或 depth",
        "body_pose_topic": "机身里程计输入（规划坐标系）",
        "sensor_pose_topic": "传感器位姿输入，外参处理由 YAML 配置",
        "cloud_topic": "激光点云输入",
        "depth_topic": "深度图输入，仅 depth 模式使用",
        "global_path_topic": "外部全局参考路径输入",
        "local_path_topic": "B 样条采样的局部路径输出，可接 path_follow",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=descriptions[name])
          for name, value in arguments.items()],
        OpaqueFunction(function=_setup),
    ])
