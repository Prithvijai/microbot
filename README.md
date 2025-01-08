# **MiRCoBot: Indoor 4-Wheel Autonomous Mobile Robot**

A compact robot designed for autonomous navigation and tasks in indoor environments. 


## Introduction 

This project focuses on creating a **digital twin** of a 4-wheel robot. It integrates various sensors using plugins to simulate the robot in a **Gazebo environment**. The project also implements **Simultaneous Localization and Mapping (SLAM)** and **Navigation** tailored for indoor settings.  

Inspired by the work of **Articulated Robotics** ([YouTube video](https://www.youtube.com/watch?v=OWeLUSzxMsw&ab_channel=ArticulatedRobotics)), this project adapts those concepts specifically for a 4-wheel robot using **ROS 2 Humble**.  

---

## Features  

- **Digital Twin**: Accurate simulation of a 4-wheel robot in Gazebo.  
- **Sensor Integration**: Connection of multiple sensors using ROS 2 plugins.  
- **SLAM Implementation**: Enables mapping and localization in indoor environments using slam_toolbox by https://github.com/SteveMacenski/slam_toolbox.  
- **Autonomous Navigation**: Path planning and execution in Gazebo using Nav2.

---

## Project demo images.

<div style="display: flex; overflow-x: auto;">

  <img src="https://github.com/user-attachments/assets/0a20294e-0885-40c9-b59f-b1018d5a9724" alt="Screenshot 1" style="max-width: 80%; margin-right: 10px;">
  <img src="https://github.com/user-attachments/assets/0fdc258b-90af-4aaa-b18d-13df16ef928a" alt="Screenshot 2" style="max-width: 80%;">

</div>

---
## User Instructions

Follow these steps to set up and run the project:

#### Step 1: Install ROS2 Humble.
If you don't have ROS 2 Humble installed, follow the installation guide for your operating system: ROS 2 Installation Guide

#### Step 2: Clone this Directory.
```bash
mkdir ros2_ws 
cd ros2_ws/
git clone https://github.com/Prithvijai/microbot.git src/
```

#### Step 3: Install dependencies.
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
#### Step 4: Build the workspace.
```bash
cd ros2_ws/
colcon build
```

#### Step 5: Run the launch file. 
```bash
cd ros2_ws/
source install/setup.bash
ros2 launch mircobot_description gazebo.launch.py
```
you able to spawn a robot in gazebo environment. check all the available launch files for further operations. 

## Important additonal  Notes

- **Dependencies**:  
  You may need to install additional packages such as `joy_tester`, `slam_toolbox`, and `Nav2` to run other launch files for SLAM and Navigation.

- **ROS2 control**:
    for working with real-robot use ros2_control and may need to install ros2 contorl packages before runing with `ros2_control: true`

---

## Find a bug ?

go to https://github.com/Prithvijai/microbot/issues for new issues. 

---

## Known issue 

* some times lidar cause some warnings 
`[rviz2-2] [INFO] [1736363085.869472333] [rviz2]: Message Filter dropping message: frame 'lidar_1' at time 18.723 for reason 'the timestamp on the message is earlier than all the data in the transform cache' ` does not cause any issue with simulation

* not fully implementated Nav2 with the robot but may work on it in future. maybe updated in repo while you 
  

---


