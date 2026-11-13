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
        default_value='true',
        description='If "true", launch the ZED camera. If "false", do not launch ZED camera.'
    )
    launch_zed = LaunchConfiguration('launch_zed')  # will be either 'true' or 'false'

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
    # 4. Livox ROS Driver2 launch (always launched)
    # ----------------------------------------------------------------------------
    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ])
    )

    # ----------------------------------------------------------------------------
    # 5. Super LIO launch (always launched)
    # ----------------------------------------------------------------------------
    super_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('super_lio'),
                'launch',
                'relocation.py'
            ])
        ])
    )

    # ----------------------------------------------------------------------------
    # 6. ZED camera launch (only if launch_zed=='true')
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
    # 7. Static tf publisher (always launched to publish base_link → zed_camera_link)
    # ----------------------------------------------------------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.257", "0.0", "0.28",
            "0", "0.0", "0",
            "base_link", "zed_camera_link"
        ],
    )

    # ----------------------------------------------------------------------------
    # 8. RL‐navigation node (always launched, but use_sim_time follows 'sim')
    # ----------------------------------------------------------------------------
    rl_navigation_node = Node(
        package='rl_nav_controller',
        executable='rl_nav_controller',  # or the actual executable name
        name='rl_nav_controller',
        output='screen',
        parameters=[{
            'use_sim_time': sim
        }],
        arguments=[
            # This PythonExpression will expand to "--sim" if sim=="true", or "" otherwise.
            PythonExpression([
                "'",
                sim,
                "' == 'true' and '--sim' or ''"
            ])
        ],
        remappings=[
            ('/dlio/odom_node/odom', '/lio/robo/odom'),
            ('/path_manager/path_manager_ros/nav_vel', '/cmd_vel'),
            # ('/zed/zed_node/depth/depth_registered', '/camera/camera/depth/image_rect_32fc1'),
        ]
    )

    # ----------------------------------------------------------------------------
    # 9. Assemble LaunchDescription
    # ----------------------------------------------------------------------------
    return LaunchDescription([
        # 1) declare arguments
        sim_arg,
        launch_zed_arg,

        # 2) launch included launch files
        # livox_driver_launch,
        # super_lio_launch,

        # 3) always-launch nodes
        # joy_node,
        static_tf_node,
        rl_navigation_node,

        # 4) conditionally include ZED launch only when launch_zed == 'true'
        zed_launch_camera,
    ])
