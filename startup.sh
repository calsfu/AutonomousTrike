#!/bin/bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash
source ~/trike_ws/install/setup.bash

# Launch ROS2 with the trike configuration in the background
ros2 launch trike main_launch.py &
docker run -v /home/coler/Desktop/SCA:/home/SCA -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' --runtime nvidia -it --rm --network=host dustynv/l4t-pytorch:autonomous_plus_logs
