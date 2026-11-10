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
    # 1. Declare a 'sim' argument (default = 'false' = no simulation)
    # ----------------------------------------------------------------------------
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='If "true", run in simulation mode. If "false", run on real hardware.'
    )
    sim = LaunchConfiguration('sim')  # will be either 'true' or 'false'

    # ----------------------------------------------------------------------------
    # 2. Declare a 'launch_zed' argument (default = 'false' = do not launch ZED)
    # ----------------------------------------------------------------------------
    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed',
        default_value='false',
        description='If "true", launch the ZED camera. If "false", do not launch ZED camera.'
    )
    launch_zed = LaunchConfiguration('launch_zed')  # will be either 'true' or 'false'

    # Global Navigation Arguments
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

    lookahead_distance_arg = DeclareLaunchArgument('lookahead_distance', default_value='2.0', description='Lookahead distance in meters')
    lookahead_distance = LaunchConfiguration('lookahead_distance')

    report_failure_arg = DeclareLaunchArgument('report_failure', default_value='true', description='Whether to report and transition to error/failed state')
    report_failure = LaunchConfiguration('report_failure')

    segment_arrive_threshold_arg = DeclareLaunchArgument('segment_arrive_threshold', default_value='0.15', description='Segment arrival distance threshold')
    segment_arrive_threshold = LaunchConfiguration('segment_arrive_threshold')

    blocked_timeout_arg = DeclareLaunchArgument('blocked_timeout', default_value='8.0', description='Stall detection timeout')
    blocked_timeout = LaunchConfiguration('blocked_timeout')

    goal_reached_timeout_arg = DeclareLaunchArgument('goal_reached_timeout', default_value='1.0', description='Goal reached timeout before auto transition to idle')
    goal_reached_timeout = LaunchConfiguration('goal_reached_timeout')

    # ----------------------------------------------------------------------------
    # 2. Package names
    # ----------------------------------------------------------------------------
    joy_package_name = 'joy'
    rl_navigation_package_name = 'rl_nav_controller'

    # ----------------------------------------------------------------------------
    # 3. Joy node (always launched)
    # ----------------------------------------------------------------------------
    joy_node = Node(
        package=joy_package_name,
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
    # 4. ZED camera launch (only if launch_zed=='true')
    # ----------------------------------------------------------------------------
    zed_launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i'
        }.items(),
        condition=IfCondition(launch_zed)  # only include when launch_zed == 'true'
    )

    # ----------------------------------------------------------------------------
    # 5. Static tf publisher (always launched to publish base_link → zed_camera_link for Go2W)
    # ----------------------------------------------------------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.257", "0.0", "0.28",
            "0.0", "0.0", "0.0",
            "base_link", "zed_camera_link"
        ],
    )

    # ----------------------------------------------------------------------------
    # 6. RL‐navigation node (always launched, but use_sim_time follows 'sim')
    # ----------------------------------------------------------------------------
    rl_navigation_node = Node(
        package='rl_nav_controller',
        executable='rl_nav_controller',
        name='rl_nav_controller',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'path_topic': path_topic,
            'stop_topic': stop_topic,
            'status_topic': status_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'start_navi_topic': start_navi_topic,
            'motion_enable_topic': motion_enable_topic,
            'odom_topic': odom_topic,
            'auto_unlock_bridge': PythonExpression(["'true' == '", auto_unlock_bridge, "'"]),
            'lookahead_distance': PythonExpression(["float('", lookahead_distance, "')"]),
            'report_failure': PythonExpression(["'", report_failure, "'.lower() == 'true'"]),
            'segment_arrive_threshold': PythonExpression(["float('", segment_arrive_threshold, "')"]),
            'blocked_timeout': PythonExpression(["float('", blocked_timeout, "')"]),
            'goal_reached_timeout': PythonExpression(["float('", goal_reached_timeout, "')"]),
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

    # ----------------------------------------------------------------------------
    # 7. Assemble LaunchDescription
    # ----------------------------------------------------------------------------
    return LaunchDescription([
        # 1) declare arguments
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
        lookahead_distance_arg,
        report_failure_arg,
        segment_arrive_threshold_arg,
        blocked_timeout_arg,
        goal_reached_timeout_arg,

        # 2) always-launch nodes
        joy_node,
        static_tf_node,
        rl_navigation_node,

        # 3) conditionally include ZED launch only when launch_zed == 'true'
        zed_launch_camera,
    ])
