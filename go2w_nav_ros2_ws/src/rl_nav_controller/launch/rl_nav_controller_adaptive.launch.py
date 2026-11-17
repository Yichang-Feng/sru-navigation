from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ----------------------------------------------------------------------------
    # 1. Standard navigation arguments
    # ----------------------------------------------------------------------------
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='If "true", run in simulation mode. If "false", run on real hardware.'
    )
    sim = LaunchConfiguration('sim')

    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed',
        default_value='false',
        description='If "true", launch the ZED camera. If "false", do not launch ZED camera.'
    )
    launch_zed = LaunchConfiguration('launch_zed')

    path_topic_arg = DeclareLaunchArgument('path_topic', default_value='/global_path', description='Global segmented path topic')
    path_topic = LaunchConfiguration('path_topic')

    stop_topic_arg = DeclareLaunchArgument('stop_topic', default_value='/mission/control', description='Mission control topic (continue/stop)')
    stop_topic = LaunchConfiguration('stop_topic')

    status_topic_arg = DeclareLaunchArgument('status_topic', default_value='/local_planner/status', description='Local planner status reporting topic')
    status_topic = LaunchConfiguration('status_topic')

    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='Velocity command output topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    start_navi_topic_arg = DeclareLaunchArgument('start_navi_topic', default_value='/startNavi', description='Chassis safety gate topic')
    start_navi_topic = LaunchConfiguration('start_navi_topic')

    motion_enable_topic_arg = DeclareLaunchArgument('motion_enable_topic', default_value='/motion_enabled', description='Go2W bridge motion enable topic')
    motion_enable_topic = LaunchConfiguration('motion_enable_topic')

    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/lio/robo/odom', description='Robot odometry topic')
    odom_topic = LaunchConfiguration('odom_topic')

    auto_unlock_bridge_arg = DeclareLaunchArgument('auto_unlock_bridge', default_value='true', description='Auto unlock chassis bridge')
    auto_unlock_bridge = LaunchConfiguration('auto_unlock_bridge')

    report_failure_arg = DeclareLaunchArgument('report_failure', default_value='true', description='Whether to report and transition to error/failed state')
    report_failure = LaunchConfiguration('report_failure')

    segment_arrive_threshold_arg = DeclareLaunchArgument('segment_arrive_threshold', default_value='0.15', description='Segment arrival distance threshold')
    segment_arrive_threshold = LaunchConfiguration('segment_arrive_threshold')

    blocked_timeout_arg = DeclareLaunchArgument('blocked_timeout', default_value='8.0', description='Stall detection timeout')
    blocked_timeout = LaunchConfiguration('blocked_timeout')

    goal_reached_timeout_arg = DeclareLaunchArgument('goal_reached_timeout', default_value='1.0', description='Goal reached timeout before auto transition to idle')
    goal_reached_timeout = LaunchConfiguration('goal_reached_timeout')

    # ----------------------------------------------------------------------------
    # 2. Adaptive lookahead specific arguments
    # ----------------------------------------------------------------------------
    enable_adaptive_arg = DeclareLaunchArgument(
        'enable_adaptive_lookahead', default_value='true', description='Enable dual-factor adaptive lookahead'
    )
    enable_adaptive = LaunchConfiguration('enable_adaptive_lookahead')

    adaptive_mode_arg = DeclareLaunchArgument(
        'adaptive_mode', default_value='both', description='Adaptive mode: both, curvature_only, clearance_only, none'
    )
    adaptive_mode = LaunchConfiguration('adaptive_mode')

    lookahead_min_arg = DeclareLaunchArgument('lookahead_min', default_value='0.5', description='Minimum lookahead distance (m)')
    lookahead_min = LaunchConfiguration('lookahead_min')

    lookahead_max_arg = DeclareLaunchArgument('lookahead_max', default_value='2.0', description='Maximum lookahead distance (m)')
    lookahead_max = LaunchConfiguration('lookahead_max')

    window_dist_arg = DeclareLaunchArgument('adaptive_window_dist', default_value='1.5', description='Curvature lookahead window distance (m)')
    window_dist = LaunchConfiguration('adaptive_window_dist')

    passage_latch_dist_arg = DeclareLaunchArgument('passage_latch_dist', default_value='1.2', description='Spatial latching distance for narrow passage (m)')
    passage_latch_dist = LaunchConfiguration('passage_latch_dist')

    door_speed_boost_arg = DeclareLaunchArgument('door_speed_boost', default_value='0.35', description='Minimum forward speed boost inside doors (m/s)')
    door_speed_boost = LaunchConfiguration('door_speed_boost')

    # ----------------------------------------------------------------------------
    # 3. Joy node
    # ----------------------------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name='joy_rsl',
        output='screen',
        remappings=[
            ('/joy', 'rsl_joy'),
            ('/joy_vel', 'rsl_joy_vel'),
        ],
        parameters=[{
            'autorepeat_rate': 50.0
        }]
    )

    # ----------------------------------------------------------------------------
    # 4. ZED camera launch (conditionally included)
    # ----------------------------------------------------------------------------
    zed_launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        condition=IfCondition(launch_zed)
    )

    # ----------------------------------------------------------------------------
    # 5. Static TF
    # ----------------------------------------------------------------------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.257', '0.0', '0.28', '0.0', '0.0', '0.0', 'base_link', 'zed_camera_link'],
        output='screen'
    )

    # ----------------------------------------------------------------------------
    # 6. Adaptive RL Navigation Node
    # ----------------------------------------------------------------------------
    rl_navigation_adaptive_node = Node(
        package='rl_nav_controller',
        executable='rl_nav_controller_adaptive',
        name='rl_nav_controller_adaptive',
        output='screen',
        parameters=[{
            'use_sim': PythonExpression(["'true' == '", sim, "'"]),
            'path_topic': path_topic,
            'stop_topic': stop_topic,
            'status_topic': status_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'start_navi_topic': start_navi_topic,
            'motion_enable_topic': motion_enable_topic,
            'odom_topic': odom_topic,
            'auto_unlock_bridge': PythonExpression(["'true' == '", auto_unlock_bridge, "'"]),
            'report_failure': PythonExpression(["'", report_failure, "'.lower() == 'true'"]),
            'segment_arrive_threshold': PythonExpression(["float('", segment_arrive_threshold, "')"]),
            'blocked_timeout': PythonExpression(["float('", blocked_timeout, "')"]),
            'goal_reached_timeout': PythonExpression(["float('", goal_reached_timeout, "')"]),
            'enable_adaptive_lookahead': PythonExpression(["'", enable_adaptive, "'.lower() == 'true'"]),
            'adaptive_mode': adaptive_mode,
            'lookahead_min': PythonExpression(["float('", lookahead_min, "')"]),
            'lookahead_max': PythonExpression(["float('", lookahead_max, "')"]),
            'adaptive_window_dist': PythonExpression(["float('", window_dist, "')"]),
            'passage_latch_dist': PythonExpression(["float('", passage_latch_dist, "')"]),
            'door_speed_boost': PythonExpression(["float('", door_speed_boost, "')"]),
        }],
        arguments=[
            PythonExpression([
                "'",
                sim,
                "' == 'true' and '--sim' or ''"
            ])
        ],
        remappings=[
            ('/dlio/odom_node/odom', odom_topic),
            ('/path_manager/path_manager_ros/nav_vel', cmd_vel_topic),
        ]
    )

    return LaunchDescription([
        sim_arg,
        launch_zed_arg,
        path_topic_arg,
        stop_topic_arg,
        status_topic_arg,
        cmd_vel_topic_arg,
        start_navi_topic_arg,
        motion_enable_topic_arg,
        odom_topic_arg,
        auto_unlock_bridge_arg,
        report_failure_arg,
        segment_arrive_threshold_arg,
        blocked_timeout_arg,
        goal_reached_timeout_arg,
        enable_adaptive_arg,
        adaptive_mode_arg,
        lookahead_min_arg,
        lookahead_max_arg,
        window_dist_arg,
        passage_latch_dist_arg,
        door_speed_boost_arg,

        joy_node,
        static_tf_node,
        rl_navigation_adaptive_node,
        zed_launch_camera,
    ])
