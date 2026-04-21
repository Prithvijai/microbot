from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('mircobot_description')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    # Process URDF with Xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'mircobot.urdf.xacro')
    robot_description_config = Command([
        'xacro ', xacro_file, 
        ' use_ros2_control:=', use_ros2_control,
        ' use_mock_hardware:=', use_mock_hardware
    ])

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    # Manual Sliders
    joint_state_publisher_gui_node = Node(
        condition=UnlessCondition(use_mock_hardware),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    # Controller Manager
    controller_params = os.path.join(pkg_path, 'config', 'my_controllers.yaml')
    controller_manager = Node(
        condition=IfCondition(use_mock_hardware),
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_config},
            controller_params
        ],
        output='screen'
    )

    # Spawners
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    # Use Event Handlers to ensure manager is up before spawning
    delayed_joint_broad = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_cont = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_cont_spawner],
        )
    )

    rviz_config_file = os.path.join(pkg_path, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description='Use mock hardware'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        controller_manager,
        delayed_joint_broad,
        delayed_diff_cont,
        rviz_node
    ])
