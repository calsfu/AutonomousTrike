#!/bin/bash
# Set up ROS2 environment
source /opt/ros/humble/setup.bash
source ~/AutonomousTrike/install/setup.bash

# Launch ROS2 with the trike configuration in the background
ros2 launch trike main_launch.py &

# run keyboard node
ros2 run tester key_op2 &
ros2 bag record -a -o ~/Desktop/rosbags/"my_bag_${EPOCHSECONDS}"

