import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node  # Correct import for ROS2
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # trike launch file
    trike_launch_path = os.path.join(
        get_package_share_directory('trike'),
        'launch',
        'trike_launch.py'
    )

    # vector_nav_path = os.path.join(
    #     get_package_share_directory('vectornav'),
    #     'launch',
    #     'vectornav.launch.py'
    # )

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(trike_launch_path)
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(vector_nav_path)
        # ),
    ])
