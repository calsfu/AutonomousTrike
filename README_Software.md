# Autonomous Trike Project - Software Documentation

## Overview

This project controls an **autonomous trike** equipped with GPS, cameras, and obstacle sensors. The trike uses global navigation (GPS) combined with local obstacle avoidance for outdoor navigation without pre-built maps.

### How ROS2 Works in This Project

The system is built using **ROS2 (Robot Operating System 2)**, a framework designed for building reliable and scalable robot software. Each major part of the trike's behavior — such as GPS navigation, vision processing, obstacle avoidance, and motor control — runs as a **separate ROS2 node**.

Nodes communicate using **topics** to publish and subscribe to messages, and **services** for direct requests. A **launch file** coordinates bringing up all necessary nodes together. This modular architecture makes the system flexible, allowing individual components to be debugged, replaced, or upgraded independently without impacting the whole system.

## Project Structure - trike Package

The `trike` package is the central ROS2 package that organizes all software components for the autonomous trike. It includes source code, launch files, configuration, and testing resources.

### Top-Level Structure Overview

- **audio/**: Pre-recorded `.wav` files for system audio feedback (e.g., "destination confirmed", "brakes on").
- **CMakeLists.txt**: CMake configuration file for building the package using `colcon` and `ament`.
- **config/**: Miscellaneous configuration files and documentation.
- **include/trike/**: C++ header files, such as `constants.hpp`, used across modules.
- **launch/**: ROS2 launch files for bringing up different parts of the system (main system, robot description, teleoperation, etc.).
- **map/**: Static map images and routing overlays for navigation planning.
- **msg/**: Directory for custom ROS2 messages
- **package.xml**: ROS2 package manifest describing dependencies and metadata.
- **param/**: YAML configuration files for node parameters (e.g., `trike.yaml` for system-wide settings).
- **scripts/**: Helper Python scripts like `segment_map.py` for map preparation.
- **src/**: Main source code organized into functional submodules:
  - **audio/**: Python code for playing system audio notifications.
  - **controller/**: Main mission controller and mode manager (e.g., autonomous vs manual mode switching).
  - **data/**: C++ components for managing telemetry and logging.
  - **embedded/**: C++ drivers interfacing with hardware (steering servo, motor controller).
  - **image_processing/**: Python utilities for basic image processing tasks.
  - **key_op/**: Keyboard operation scripts for teleoperation and testing.
  - **localization/**: C++ drivers to interface with the VN200 GPS/INS device.
  - **navigation/**: Navigation utilities including lane following and map-based routing.
  - **perception/**: Obstacle detection and emergency stop logic.
  - **robot_model/**: Python code for maintaining an internal kinematic model of the trike.
  - **util/**: Miscellaneous helper scripts and tools.
- **srv/**: Directory for custom ROS2 services
- **test/**: Test scripts, including hardware tests, simulation demos (Carla), and trajectory planning.
- **urdf/**: Robot model files in URDF and XACRO format for simulation and visualization (RViz, Gazebo).

---

Each folder is structured to keep functionality modular and allow easier maintenance and scalability as the system grows.

## Internal Node Descriptions
Inside the trike package, the following nodes are used by our system. 
| Node | Description |
|:-----|:------------|
| `servo_controller` | Controls the braking motor via commands received from the navigation systems. |
| `steering_controller` | Adjusts the steering angle of the trike, interpreting signals from the navigation modules. |
| `data_manager` | Handles the collection and management of sensor data, relaying data based on the current mode of the trike|
| `audio_player` | Plays pre-recorded audio feedback based on the system's state or mode, such as turning or stops. |
| `controller_line` | Responsible for controlling the yaw of the trike along a designated line. |
| `image_segment` | Processes images for segmentation to detect road lanes in the environment. |
| `mode_manager` | Manages the various operational modes of the trike (e.g., manual, autonomous, park). |
| `emergency_stop` | Monitors for collision risks and triggers an immediate stop. |
| `key_op` | Handles key press operations, possibly for manual control or mode switching via keyboard input. |

## Basic Control Flow

1. **Image Processing:**
   - The **camera** captures an image, which is sent to the `image_segment` node.
   - The `image_segment` node processes the image, performs segmentation, and finds a centroid that represents the trike's lane or path.

2. **Path Control:**
   - The centroid data is passed to the `controller_line` node.
   - The `controller_line` node calculates a **PID error** to determine how far off the trike is from the desired path.
   - Based on this error, a **Twist message** (representing the required speed and steering direction) is generated and passed to the `steering_controller`.

3. **Steering Control:**
   - The `steering_controller` node receives the Twist message and translates it into **steering commands**, adjusting the trike’s steering angle to follow the desired path.

4. **Obstacle Detection and Emergency Braking:**
   - The `emergency_stop` node constantly monitors the environment for obstacles.
   - If an obstacle is detected, `emergency_stop` triggers an immediate stop by sending a **brake command** to the `servo_controller` node.
   - This ensures that the trike comes to a halt to avoid collisions.

5. **Audio Feedback:**
   - Each of the control actions (e.g., turning, stopping, mode changes) is accompanied by appropriate **audio feedback** via the `audio_player` node.
   - For example, when the trike starts turning, the `audio_player` will play the "turning left" or "turning right" sound.

6. **Mode Management:**
   - The `mode_manager` node manages the current operational mode (e.g., park, manual, autonomous).
   - Depending on the mode, it relays relevant messages to the `data_manager` to update system status.
   - For example, if the mode is "park", the `data_manager` will prevent commands from being sent to the `servo_controller` or `steering_controller`, keeping the system idle.

---

This flow ensures smooth operation of the autonomous trike by tightly integrating vision processing, path planning, control systems, and safety mechanisms.

## Launch Files

### trike_launch.py
The `trike_launch.py` file is responsible for launching all the nodes that we wrote in the **trike package**. This includes nodes for core functionality like audio, controller operations, steering, and perception.

### main_launch.py
The `main_launch.py` file is a higher-level launch file that brings up both the **trike_launch.py** and external packages. It is responsible for integrating the external dependencies like the **DepthAI OAK-D node** and **vectornav node** along with the internal system.

Key external nodes launched by `main_launch.py`:
- **depthai camera.launch.py** – Handles camera input and image processing for dynamic obstacle detection and lane following.
- **vectornav vectornav.launch.py** – Interfaces with the VectorNav IMU to enhance the trike’s localization and provide inertial data for navigation.

By launching `main_launch.py`, the system is able to bring up both internal and external components, ensuring the complete trike software stack is running.

---

These launch files streamline the process of starting up the autonomous trike by organizing related nodes into manageable sections. This structure allows for easy integration and scaling as new features or external dependencies are added.

## Development and Build Tool Information

### How Building Works in ROS2

ROS2 uses a build system based around **CMake** and **ament** to manage and compile code. Each part of the robot software (called a **package**) has its own directory, `package.xml`, and `CMakeLists.txt`. 

The entire workspace is built using **colcon**, which finds all packages automatically and builds them in the correct order based on dependencies. This system allows easy modular development and cross-compilation for different platforms.

Key tools:
- **CMake**: Underlying build system for C++ packages.
- **ament**: ROS2-specific build extension that manages configuration, testing, and installation.
- **colcon**: High-level build tool that compiles all packages in a workspace.
- **package.xml**: Defines package metadata and dependencies.

**Typical workflow**:
```bash
cd ~/trike_ws
colcon build --symlink-install
source install/setup.bash
```

---

### Software and Package Versions

| Software/Package | Version |
|------------------|---------|
| ROS2 Humble Hawksbill | Latest Patch (2025-04) |
| Ubuntu | 22.04 LTS |
| Python | 3.10 |
| OpenCV | 4.5.5 |
| C++ Compiler | GCC 11.3 |
| CMake | 3.22.1 |
| Arduino IDE | 2.2.1 |
| Jetson Orin Nano Libraries | JetPack 6.2 DP |

CUDA and TensorRT are installed but **optional** (used for accelerated vision inference if enabled).

----

## Installation Guide - Full Setup from Blank Hard Drive

### Step 1: Flash Operating System
- Download and flash **Ubuntu 22.04 LTS** onto the target system (e.g., Jetson Orin Nano or x86 laptop).
- If using Jetson, install **JetPack 6.0 DP** (developer preview).

### Step 2: Install ROS2 Humble
```bash
sudo apt update
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install ros-humble-desktop
```

**Source the ROS2 environment:**
```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Set Up ROS2 Workspace
Create a workspace where all packages will live:
```bash
mkdir -p ~/trike_ws/src
cd ~/trike_ws/src
```

Clone the project repository:
```bash
git clone https://github.com/coler/autonomous-trike.git
```

Clone external packages
```bash
cd ..
vcs import . < AutonomousTrike/trike.repos 
```

Go back to the workspace root:
```bash
cd ~/trike_ws
```

Install any missing dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build the Workspace
Use **colcon** to build all packages:
```bash
colcon build --symlink-install
```

Source your workspace so ROS2 can find the new packages:
```bash
source install/setup.bash
```

---

## Arduino (Microcontroller) Setup

### Step 1: Install Arduino IDE
Install **Arduino IDE 2.2.1** from the [official Arduino website](https://www.arduino.cc/en/software).

### Step 2: Flash Arduino Code
1. Connect the braking Arduino to the host computer via USB.
2. Open `arduino/braking_controller.ino`.
3. Upload the sketch to the Arduino.
4. Connect the steering Arduino to the host computer via USB.
5. Open `arduino/steering_controller.ino`.
6. Upload the sketch to the Arduino.
7. Arduino listens over serial for velocity and steering commands sent from ROS2.

---

