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
1. Clone the repository:
```bash
git clone
cd autonomous_trike
```
2. Install dependencies:
```bash
sudo apt install ros-humble-depthai-ros
```
3. Build the project:
```bash
colcon build
```
4. Source the workspace:
```bash
source install/setup.bash
```
5. Connect the Jetson Orin Nano to the Arduino for motor control.
6. Connect the stereo cameras to the Jetson Orin Nano.
7. Launch the system:
```bash
ros2 launch trike main_launch.py

```
