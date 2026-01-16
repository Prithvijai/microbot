from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    display_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory("mircobot_description"),'launch','rsp.launch.py'
                    )]), launch_arguments={'use_sim_time': 'true','use_ros2_control': 'false'}.items()  # for gazebo simulations only is the better to use gazebo plugin so better movement. use ros2_control for real-robot integration. 
        )
    gazebo_params_path = os.path.join(
                  get_package_share_directory("mircobot_description"),'config','gazebo_params.yaml')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                    )]), launch_arguments={'extra_gazebo_args': '-s libgazebo_ros_init.so '
     '-s libgazebo_ros_factory.so '
     '-s libgazebo_ros_state.so '
     '-s libgazebo_ros_clock.so ''--ros-args --params-file ' + gazebo_params_path }.items()
             )
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("mircobot_description"),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory("mircobot_description"),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )
    
    default_world = os.path.join(
        get_package_share_directory("mircobot_description"),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mircobot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )

    joint_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    return LaunchDescription([
        display_launch,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        urdf_spawn_node,
        diff_drive,
        joint_broad
    ])
