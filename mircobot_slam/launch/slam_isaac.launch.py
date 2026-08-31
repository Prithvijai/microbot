import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mircobot_slam_pkg = get_package_share_directory('mircobot_slam')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to the specific SLAM parameters for your robot
    slam_params_file = os.path.join(mircobot_slam_pkg, 'config', 'mapper_params_online_async.yaml')

    # Path to the pointcloud to laserscan parameters
    pc_params_file = os.path.join(mircobot_slam_pkg, 'config', 'slam_3d_to_2d_params.yaml')

    pc_2_laser_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[pc_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cloud_in', '/point_cloud'),
            ('scan', '/scan')
        ]
    )

    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_pkg, 'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        pc_2_laser_node,
        slam_toolbox_node
    ])
