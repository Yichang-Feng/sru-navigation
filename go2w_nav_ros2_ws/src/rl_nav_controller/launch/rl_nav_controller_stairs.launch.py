#!/usr/bin/env python3
"""rl_nav_controller_stairs.launch.py —— 楼梯感知 RL 控制器单节点启动。

替代手写这一长串：

    PYTHONPATH=... python3 -u src/rl_nav_controller/rl_nav_controller/\
        rl_nav_controller_stairs.py --ros-args -p ... -p ...

用法：

    # 先 source ROS 与工作空间
    source /opt/ros/foxy/setup.bash          # Humble 改成 humble
    cd ~/go2w_nav_ros2_ws
    source install/setup.bash

    ros2 launch rl_nav_controller rl_nav_controller_stairs.launch.py

    # 直接按路径运行（不需要重新 colcon build 也能用，前提是节点 executable 已安装）
    ros2 launch ~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/\
        rl_nav_controller_stairs.launch.py

常用覆盖：

    ros2 launch rl_nav_controller rl_nav_controller_stairs.launch.py \
        depth_topic:=/camera/camera/depth/image_rect_raw \
        lookahead_max:=1.2 report_failure:=false

注意：
- 本文件的默认值 = 实机上调通的参数，直接
  `ros2 launch rl_nav_controller rl_nav_controller_stairs.launch.py`
  即可，不需要再传任何参数。
- 默认 depth_topic 为 /camera/depth/image_rect_raw（本机相机不带 namespace）。
  若相机改用 `camera_namespace:=camera camera_name:=camera` 启动，话题会变成
  /camera/camera/depth/image_rect_raw，启动时覆盖：
  depth_topic:=/camera/camera/depth/image_rect_raw
- 节点完全由深度回调驱动：话题不对就一行 cmd_vel 都不会发，
  2 秒后安全看门狗报 "Camera depth frames lost" 并转 ERROR。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _bool(name):
    """Launch 参数是字符串，转成 rclpy 需要的 bool。"""
    return PythonExpression(["'true' == '", LaunchConfiguration(name), "'"])


def _float(name):
    """Launch 参数是字符串，转成 rclpy 需要的 float。"""
    return PythonExpression(["float('", LaunchConfiguration(name), "')"])


def generate_launch_description():
    def arg(name, default, description=''):
        return DeclareLaunchArgument(
            name,
            default_value=str(default),
            description=description or name,
        )

    declared_args = [
        # 接口话题
        arg('path_topic', '/global_path', '全局分段路径话题'),
        arg('stop_topic', '/mission/control', '任务控制话题'),
        arg('status_topic', '/local_planner/status', '状态上报话题'),
        arg('status_msgs_format', 'list', '状态字段格式: list | string_brackets | empty_string'),
        arg('cmd_vel_topic', '/cmd_vel', '速度指令输出话题'),
        arg('start_navi_topic', '/startNavi', '底盘安全总闸话题'),
        arg('motion_enable_topic', '/motion_enabled', '手柄 A/B 运动门话题'),
        arg('odom_topic', '/lio/robo/odom', '里程计话题'),
        arg('auto_unlock_bridge', 'true', '首次收到里程计后置 /startNavi=True（不自动打开 /motion_enabled，需 START/A 放行）'),
        arg('path_frame_id', '', '路径坐标系，空=自动匹配'),
        arg('lookahead_distance', '2.0', '固定前瞻距离 (m)'),
        arg('camera_timeout', '2.0', '深度帧超时看门狗 (s)'),
        arg('goal_reached_timeout', '1.0', '到点后转 idle 延时 (s)'),
        arg('blocked_timeout', '4.0', '停滞判定超时 (s)'),
        arg('report_failure', 'true', '停滞时是否上报 failed'),

        # 深度相机（本机实机话题，相机未设 namespace）
        # 若相机以 camera_namespace:=camera 启动，话题是
        # /camera/camera/depth/image_rect_raw，启动时覆盖：
        #   depth_topic:=/camera/camera/depth/image_rect_raw
        arg('depth_topic', '/camera/depth/image_rect_raw', '深度图话题'),
        arg('min_depth', '0.15', '最小有效深度 (m)'),
        arg('max_depth', '10.0', '最大有效深度 (m)'),

        # 平地自适应前瞻
        arg('enable_adaptive_lookahead', 'true', '启用动态前瞻'),
        arg('adaptive_mode', 'curvature_only', 'both | curvature_only | clearance_only | none'),
        arg('lookahead_min', '0.4', '自适应前瞻下界 (m)'),
        arg('lookahead_max', '2.0', '自适应前瞻上界 (m)'),
        arg('adaptive_window_dist', '1.6', '曲率前瞻窗口 (m)'),
        arg('turn_angle_low_deg', '20.0', '曲率下阈值 (deg)'),
        arg('turn_angle_high_deg', '50.0', '曲率上阈值 (deg)'),
        arg('switch_mode', 'corner_dist', 'curvature | corner_dist | hybrid'),
        arg('corner_switch_thresh_deg', '25.0', '拐角判定角度 (deg)'),
        arg('corner_switch_margin', '0.25', '拐角前提前量 (m)'),
        arg('slew_rate_increase', '0.8', '前瞻恢复速率 (m/s)'),
        arg('slew_rate_decrease', '2.5', '前瞻收缩速率 (m/s)'),
        arg('door_speed_boost', '0.0', '门区保底速度 (m/s)'),
        arg('segment_arrive_threshold', '0.45', '路径段到达阈值 (m)'),

        # 楼梯感知
        arg('enable_stairs_mode', 'true', '启用楼梯坡度门控'),
        arg('enable_stair_remapping', 'false', '启用楼梯深度重映射'),
        arg('stair_lookahead_dist', '0.40', '下楼前瞻锁存 (m)'),
        arg('stair_up_lookahead_dist', '0.45', '上楼前瞻锁存 (m)'),
        arg('stair_min_forward_vel', '0.0', '下楼保底线速度 (m/s)'),
        arg('stair_slope_threshold', '-0.20', '下楼坡度阈值'),
        arg('stair_up_slope_threshold', '0.20', '上楼坡度阈值'),
        arg('stair_side_min_clamp', '0.50', '扶手侧边钳位 (m)'),
        arg('stair_min_elevation_drop', '0.15', '下楼最小落差 (m)'),


        # 终点航向对齐
        arg('enable_terminal_heading_align', 'true', '启用终点航向对齐'),
        arg('terminal_heading_tol', '0.26', '航向容差 (rad)'),
        arg('terminal_align_enter_dist', '0.50', '允许进入转身的距离 (m)'),
        arg('terminal_align_abort_dist', '0.60', '放弃转身的距离 (m)'),
        arg('terminal_align_lever_arm', '0.55', '原地旋转杠杆臂 (m)'),
    ]

    parameters = {
        'use_sim_time': False,
        'path_topic': LaunchConfiguration('path_topic'),
        'stop_topic': LaunchConfiguration('stop_topic'),
        'status_topic': LaunchConfiguration('status_topic'),
        'status_msgs_format': LaunchConfiguration('status_msgs_format'),
        'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        'start_navi_topic': LaunchConfiguration('start_navi_topic'),
        'motion_enable_topic': LaunchConfiguration('motion_enable_topic'),
        'odom_topic': LaunchConfiguration('odom_topic'),
        'auto_unlock_bridge': _bool('auto_unlock_bridge'),
        'path_frame_id': LaunchConfiguration('path_frame_id'),
        'lookahead_distance': _float('lookahead_distance'),
        'camera_timeout': _float('camera_timeout'),
        'goal_reached_timeout': _float('goal_reached_timeout'),
        'blocked_timeout': _float('blocked_timeout'),
        'report_failure': _bool('report_failure'),

        'depth_topic': LaunchConfiguration('depth_topic'),
        'min_depth': _float('min_depth'),
        'max_depth': _float('max_depth'),

        'enable_adaptive_lookahead': _bool('enable_adaptive_lookahead'),
        'adaptive_mode': LaunchConfiguration('adaptive_mode'),
        'lookahead_min': _float('lookahead_min'),
        'lookahead_max': _float('lookahead_max'),
        'adaptive_window_dist': _float('adaptive_window_dist'),
        'turn_angle_low_deg': _float('turn_angle_low_deg'),
        'turn_angle_high_deg': _float('turn_angle_high_deg'),
        'switch_mode': LaunchConfiguration('switch_mode'),
        'corner_switch_thresh_deg': _float('corner_switch_thresh_deg'),
        'corner_switch_margin': _float('corner_switch_margin'),
        'slew_rate_increase': _float('slew_rate_increase'),
        'slew_rate_decrease': _float('slew_rate_decrease'),
        'door_speed_boost': _float('door_speed_boost'),
        'segment_arrive_threshold': _float('segment_arrive_threshold'),

        'enable_stairs_mode': _bool('enable_stairs_mode'),
        'enable_stair_remapping': _bool('enable_stair_remapping'),
        'stair_lookahead_dist': _float('stair_lookahead_dist'),
        'stair_up_lookahead_dist': _float('stair_up_lookahead_dist'),
        'stair_min_forward_vel': _float('stair_min_forward_vel'),
        'stair_slope_threshold': _float('stair_slope_threshold'),
        'stair_up_slope_threshold': _float('stair_up_slope_threshold'),
        'stair_side_min_clamp': _float('stair_side_min_clamp'),
        'stair_min_elevation_drop': _float('stair_min_elevation_drop'),


        'enable_terminal_heading_align': _bool('enable_terminal_heading_align'),
        'terminal_heading_tol': _float('terminal_heading_tol'),
        'terminal_align_enter_dist': _float('terminal_align_enter_dist'),
        'terminal_align_abort_dist': _float('terminal_align_abort_dist'),
        'terminal_align_lever_arm': _float('terminal_align_lever_arm'),
    }

    stairs_node = Node(
        package='rl_nav_controller',
        executable='rl_nav_controller_stairs',
        name='navigation_policy_node',
        output='screen',
        parameters=[parameters],
        # 不要再传 --segment-arrive-threshold：那个 argparse 参数以前是用来绕开
        # 「--arrive-threshold 默认 0.45 覆盖 ROS 参数」的 bug；该 bug 已修好，
        # ROS 参数 segment_arrive_threshold 现在是唯一来源。
    )

    return LaunchDescription(declared_args + [stairs_node])
