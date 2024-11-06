# Use the official ROS2 Humble image
FROM osrf/ros:humble-desktop

# Set the working directory
WORKDIR /ros2_ws

# Copy the package(s) to the container
COPY ros2_ws /ros2_ws/

# Install dependencies
RUN apt update && \
    apt install -y python3-pip && \
    pip3 install -r ros2_ws/trike_control/requirements.txt

# Build the ROS2 package
RUN colcon build

# Set environment variables
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2

# Run the command to start the ROS2 node
CMD ["ros2", "launch", "trike_control", "your_launch_file.launch.py"]
