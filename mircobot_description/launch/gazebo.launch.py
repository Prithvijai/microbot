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
                    )]), launch_arguments={'use_sim_time': 'true','use_ros2_control': 'true'}.items()  # for gazebo simulations only is the better to use gazebo plugin so better movement. use ros2_control for real-robot integration. 
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
        'warehouse.world'
        )    
    
    # export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/saitama/Downloads/warehouse_simulation_toolkit-master/models
    # export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/saitama/Downloads/warehouse_simulation_toolkit-master/models/workcell/materials/textures
    # export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/saitama/Downloads/warehouse_simulation_toolkit-master/models/workcell_bin/materials/textures/

    # run this for getting the world properly warehouse.world

    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    
    gazebo_params_path = os.path.join(
                  get_package_share_directory("mircobot_description"),'config','gazebo_params.yaml')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r v4 ', world], 'on_exit_shutdown': 'true' }.items()
             )

    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mircobot',
            '-topic', 'robot_description',
            '-z', '0.1',
            '-y', '4.0'
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

    

    bridge_params = os.path.join(get_package_share_directory("mircobot_description"),'config','gz_bridge.yaml')

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        display_launch,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        urdf_spawn_node,
        diff_drive,
        joint_broad,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
