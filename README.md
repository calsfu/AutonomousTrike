# Autonomous Trike Project

## Overview
This project aims to develop an autonomous trike using ROS2, stereo cameras, and other low-cost solutions. The system integrates GPS-less navigation, sensor fusion, and motor control.

## Features
- GPS-less navigation using computer vision and sensor fusion
- Monocular camera-based perception
- ROS2 integration for modularity and scalability
- Motor control via Arduino and Jetson Orin Nano

## Dependencies
Ensure you have the following dependencies installed before running the project:

### System Requirements
- Ubuntu 22.04 (recommended)
- ROS2 Humble
- Jetson Orin Nano (or compatible SBC)
- Arduino (for motor control)

### Software Dependencies
- sudo apt install ros-humble-depthai-ros

## Installation
1. Create a ROS2 workspace:
```bash
mkdir -p ~/trike_ws/src
cd ~/trike_ws/src
```
2. Clone the repository:
```bash
git clone https://github.com/calsfu/AutonomousTrike.git
```
3. Clone external packages
```bash
vcs import . < AutonomousTrike/trike.repos 
```
4. Build the project:
```bash
cd ~/trike_ws
colcon build
```
5. Source the workspace:
```bash
source install/setup.bash
```
6. Launch the system:
```bash
ros2 launch trike main_launch.py
```

### Testing
Individual components can be run using the following commands

- Oak-D camera
```bash
ros2 launch depthai_ros_driver camera.launch.py
```

- Steering and Braking motors
```bash
ros2 launch trike trike_launch.py
```
