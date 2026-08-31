import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('mircobot_description')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = Command([
        'xacro ',
        os.path.join(description_share, 'urdf', 'mircobot.urdf.xacro'),
        ' use_gazebo:=false',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_share, 'config', 'display.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
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
        parameters=[os.path.join(description_share, 'config', 'joystick.yaml')],
        remappings=[('cmd_vel', 'cmd_vel_joy')],
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            os.path.join(description_share, 'config', 'twist_mux.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('cmd_vel_out', '/diff_cont/cmd_vel')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the clock published by Isaac Sim',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz',
        ),
        robot_state_publisher,
        rviz_node,
        joy_node,
        teleop_node,
        twist_mux,
    ])
