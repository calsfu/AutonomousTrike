# Autonomous Trike Project

## Overview

This project aims to develop an autonomous trike using ROS2, stereo cameras, and other low-cost solutions. The system integrates GPS-less navigation, sensor fusion, and motor control.

## Hardware Setup

- Connect the Oak-D camera via USB to the Jetson Orin Nano
- Connect the Arduinos to the Jetson via USB for motor control
- Ensure motors are wired properly to the Arduino motor driver
- Flash the arduionos with `braking_control.ino` and `steering_control.ino`

## System Requirements

- Ubuntu 22.04 (recommended)
- ROS2 Humble
- Jetson Orin Nano 
- Arduino (for motor control)

## Software Dependencies

Install required ROS2 packages:

```bash
sudo apt update
sudo apt install ros-humble-depthai-ros
```

## Installation

Create a ROS2 workspace:

```bash
mkdir -p ~/trike_ws/src
cd ~/trike_ws/src
```

Clone the repository:

```bash
git clone https://github.com/calsfu/AutonomousTrike.git
```

Install source dependencies:

```bash
vcs import . < AutonomousTrike/trike.repos
rosdep install --from-paths src --ignore-src -r -y
```

Build the project:

```bash
cd ~/trike_ws
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Launch the system:

```bash
ros2 launch trike main_launch.py
```

## Testing

Individual components can be run using the following commands:

**Oak-D camera:**

```bash
ros2 launch depthai_ros_driver camera.launch.py
```

**Internal systems:**

```bash
ros2 launch trike trike_launch.py
```

## Usage

To run the system automatically on boot, create a startup script:

Create a file named `trike_startup.sh` in your home directory:

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

Add the script to your startup applications.

## Troubleshooting

If you encounter issues, check the following:

- Ensure all dependencies are installed
- Verify the camera is connected and recognized by the system
- Check the ROS2 logs for any errors
- Ensure the Arduinos are connected and configured correctly
- Verify the network connection for remote access

## Project Structure

- `trike/` — Main ROS2 package for navigation and control
- `trike/launch/` — Launch files to start system components
- `trike.repos` — VCS file to fetch external dependencies
