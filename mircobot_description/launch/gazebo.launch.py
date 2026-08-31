import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('mircobot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    warehouse_models = os.path.join(
        description_share, 'models', 'robotnik_warehouse'
    )
    warehouse_resources = os.pathsep.join([
        warehouse_models,
        os.path.join(warehouse_models, 'workcell', 'materials', 'textures'),
        os.path.join(
            warehouse_models, 'workcell_bin', 'materials', 'textures'
        ),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])
    robot_description = Command([
        'xacro ',
        os.path.join(description_share, 'urdf', 'mircobot.urdf.xacro'),
        ' use_gazebo:=true',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_share, 'config', 'display.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[os.path.join(description_share, 'config', 'joystick.yaml')],
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            os.path.join(description_share, 'config', 'joystick.yaml'),
            {'publish_stamped_twist': True},
        ],
        remappings=[('cmd_vel', 'cmd_vel_joy')],
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            os.path.join(description_share, 'config', 'twist_mux.yaml'),
            {
                'use_sim_time': True,
                'use_stamped': True,
            },
        ],
        remappings=[('cmd_vel_out', '/diff_cont/cmd_vel')],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mircobot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                description_share, 'config', 'gz_bridge.yaml'
            )
        }],
    )
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(description_share, 'worlds', 'empty.world'),
            description='Gazebo world file to load',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz',
        ),
        DeclareLaunchArgument('spawn_x', default_value='-2.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            warehouse_resources,
        ),
        robot_state_publisher,
        rviz_node,
        joy_node,
        teleop_node,
        twist_mux,
        gazebo,
        spawn_robot,
        diff_drive,
        joint_state_broadcaster,
        bridge,
        image_bridge,
    ])
