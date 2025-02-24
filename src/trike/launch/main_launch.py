import launch
from launch.actions import IncludeLaunchDescription
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

    # camera launch file
    camera_launch_path = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch',
        'camera.launch.py'
    )

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(trike_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path)
        )
    ])
