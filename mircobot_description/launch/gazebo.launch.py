from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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
                    )]), launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
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
        gazebo,
        urdf_spawn_node,
        diff_drive,
        joint_broad
    ])
