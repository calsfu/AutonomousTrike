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
3. Install source dependencies:
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

- Internal systems
```bash
ros2 launch trike trike_launch.py
```

### Usage
In order to use the system without ssh, you can setup a startup script that will run the launch command automatically. This can be done by creating a file named `trike_startup.sh` in your home directory with the following content:
```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/trike_ws/install/setup.bash
ros2 launch trike main_launch.py
```
Make the script executable:
```bash
chmod +x ~/trike_startup.sh
```
Then, you can add the script to your startup applications.
### Troubleshooting
If you encounter issues, check the following:
- Ensure all dependencies are installed
- Verify the camera is connected and recognized by the system
- Check the ROS2 logs for any errors
- Ensure the Arduino is connected and configured correctly
- Verify the network connection for remote access
- Check the camera calibration and parameters
- Ensure the motor control is functioning correctly