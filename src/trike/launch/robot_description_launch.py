import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your Xacro file
    xacro_file = os.path.join(
        FindPackageShare('my_robot_description').find('my_robot_description'),
        'urdf', 'trike.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=xacro_file, description='Path to Xacro model'),

        # Node to process the Xacro file
        Node(
            package='xacro',
            executable='xacro',
            name='xacro',
            output='screen',
            arguments=[xacro_file],
            remappings=[('/robot_description', '/robot_description')]
        ),

        # Node to publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('model')}],
        ),

        # Launch RViz (Optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(FindPackageShare('my_robot_description').find('my_robot_description'), 'rviz', 'config.rviz')]
        ),
    ])
