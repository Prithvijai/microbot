# **MiRCoBot: Indoor 4-Wheel Autonomous Mobile Robot**

MiRCoBot is a four-wheel differential-drive robot for ROS 2 Jazzy. This
repository provides separate bringup paths for Gazebo Harmonic and Isaac Sim,
joystick command multiplexing, 3D LiDAR and camera sensors, and an optional
SLAM Toolbox pipeline.

Inspired by the work of **Articulated Robotics** ([YouTube video](https://www.youtube.com/watch?v=OWeLUSzxMsw&ab_channel=ArticulatedRobotics)), this project adapts those concepts specifically for a 4-wheel robot using **ROS 2 Jazzy**.  

## Gazebo Harmonic

<img width="2462" height="1513" alt="image" src="https://github.com/user-attachments/assets/0a324b35-08ea-462a-b477-e30fc57cef66" />

<img width="2365" height="1104" alt="image" src="https://github.com/user-attachments/assets/0b83c015-030a-4382-a069-85df209785ac" />

Start Gazebo, MiRCoBot, `ros2_control`, joystick teleoperation, sensor bridges,
and RViz:

```bash
ros2 launch mircobot_description gazebo.launch.py
```

The default world is `empty.world`. Select another local SDF world with the
`world` argument:

```bash
ros2 launch mircobot_description gazebo.launch.py \
  world:=/path/to/world.sdf
```

Available launch arguments include:

| Argument | Default | Description |
| --- | --- | --- |
| `world` | Installed `empty.world` | SDF world file |
| `rviz` | `true` | Start RViz |
| `spawn_x` | `-2.0` | Robot X coordinate |
| `spawn_y` | `0.0` | Robot Y coordinate |
| `spawn_z` | `0.1` | Robot height |
| `spawn_yaw` | `0.0` | Robot yaw in radians |

### Bundled Worlds

World source files are stored in:

```text
mircobot_description/worlds/
```

After building, they are installed in:

```text
/ros2_ws/install/mircobot_description/share/mircobot_description/worlds/
```

The tested offline worlds are:

| World | Purpose |
| --- | --- |
| `empty.world` | Controller and sensor testing |
| `simple_maze.world` | Mapping and structured navigation |
| `robotnik_warehouse.world` | Realistic indoor warehouse mapping |

Launch the maze:

```bash
ros2 launch mircobot_description gazebo.launch.py \
  world:=/ros2_ws/install/mircobot_description/share/mircobot_description/worlds/simple_maze.world \
  spawn_x:=0.5 spawn_y:=0.5
```

Launch the warehouse:

```bash
ros2 launch mircobot_description gazebo.launch.py \
  world:=/ros2_ws/install/mircobot_description/share/mircobot_description/worlds/robotnik_warehouse.world
```

The warehouse uses the safe default spawn pose `(-2, 0, 0.1)`. Its required
meshes and textures are stored under
`mircobot_description/models/robotnik_warehouse/`, installed by CMake, and
added to `GZ_SIM_RESOURCE_PATH` by `gazebo.launch.py`. No network connection is
required at runtime.

The simplified warehouse is based on Robotnik Automation's
[`warehouse_world`](https://github.com/RobotnikAutomation/robotnik_gazebo_worlds/tree/jazzy-devel/warehouse_world),
which is derived from `warehouse_simulation_toolkit`. The vendored assets retain
their BSD 3-Clause license in
`mircobot_description/models/robotnik_warehouse/LICENSE`.

### Controls

The joystick command path is:

```text
/joy -> /cmd_vel_joy -> twist_mux -> /diff_cont/cmd_vel
```

Joystick commands have priority over teleoperation, tracker, and navigation
commands. Gazebo uses stamped velocity commands for the differential-drive
controller.

### Point Cloud And TF

Gazebo publishes the 3D LiDAR point cloud through `ros_gz_bridge`:

```text
Gazebo pointcloud/points -> ROS /point_cloud
```

The point-cloud frame is `lidar`, with this TF chain:

```text
odom -> base_footprint -> base_link -> lidar
```

The controller and odometry transform are updated at 100 Hz to keep transforms
available for sensor timestamps. After changing controller parameters, restart
the launch process because controllers read them only during startup.

Useful diagnostics are:

```bash
ros2 topic echo /point_cloud --once --field header
ros2 topic hz /point_cloud
ros2 run tf2_ros tf2_echo odom lidar
```

A brief missing-transform warning while controllers start is expected. If it
continues during operation, rebuild, source the workspace, and restart Gazebo.

RViz can be disabled when it is not needed:

```bash
ros2 launch mircobot_description gazebo.launch.py rviz:=false
```

## Isaac Sim

Open `mircobot_description/urdf/mircobot/warehouse_mircobot.usd` in Isaac Sim,
start its timeline, and then run:

```bash
ros2 launch mircobot_description isaac_sim.launch.py
```

This launch starts the robot state publisher, joystick, twist mux, and optional
RViz. It does not start Isaac Sim. The USD Action Graph is expected to:

- Subscribe to `/diff_cont/cmd_vel` as `geometry_msgs/msg/Twist`.
- Publish `/clock` when `use_sim_time:=true`.
- Publish `/joint_states` for moving-link transforms.
- Publish odometry and the `odom -> base_footprint` transform.
- Publish sensor data, including `/point_cloud` when SLAM is used.

Inspect its live ROS interface with:

```bash
ros2 topic list
ros2 topic info /diff_cont/cmd_vel -v
ros2 topic info /joint_states -v
ros2 run tf2_ros tf2_echo odom base_footprint
```

RViz can be disabled with `rviz:=false`. If Isaac Sim does not publish
`/clock`, use wall time with `use_sim_time:=false`.

## SLAM

The current pipeline converts `/point_cloud` to `/scan` and runs SLAM Toolbox:

```bash
ros2 launch mircobot_slam slam_isaac.launch.py
```

Start simulator bringup before launching SLAM. The simulator must provide
`/point_cloud`, odometry, the robot TF tree, and `/clock` when simulation time
is enabled.

## Verification

Run package tests inside the container:

```bash
colcon test --packages-select mircobot_description
colcon test-result --verbose
```
