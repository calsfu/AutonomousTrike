import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Specify the name of the robot
    robot_name = "trike"

    # Set the path to your robot's URDF file
    urdf_path = os.path.join(
        get_package_share_directory("trike"),  # Replace with your robot's description package name
        "urdf",
        "ackerman.urdf"  # Replace with the actual filename of your URDF
    )

    # Launch Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros2'), 'launch', 'gazebo.launch.py')
        )
    )

    # Define the initial pose of the robot
    initial_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

    # Spawn the robot using the gazebo_ros2 spawner
    spawn_entity = Node(
        package='gazebo_ros2',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', robot_name,
                   '-topic', '/robot_description',
                   '-x', str(initial_pose['x']),
                   '-y', str(initial_pose['y']),
                   '-z', str(initial_pose['z']),
                   '-R', str(initial_pose['roll']),
                   '-P', str(initial_pose['pitch']),
                   '-Y', str(initial_pose['yaw'])],
        output='screen'
    )

    # Publish the robot description to the /robot_description topic
    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}] # Use 'xacro' if your file is a .xacro
    )

    return LaunchDescription([
        gazebo,
        robot_description_publisher,
        spawn_entity,
    ])