#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('trike'),
            'param',
            'trike.yaml'))
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('trike'),
            'map',
            'segmented_left_turn.yaml'))

    # launch
    # TODO: Add back when done testing
    # camera_launch_path = os.path.join(
    # get_package_share_directory('depthai_ros_driver'),
    # 'launch',
    # 'camera.launch.py'
    # )
    # trike_launch_path = os.path.join(
    #     get_package_share_directory('trike'),
    #     'launch',
    #     'trike_launch.py'
    # )
    main_launch_path = os.path.join(
        get_package_share_directory('trike'),
        'launch',
        'main_launch.py'
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        # TODO: Add back when done testing
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(camera_launch_path)
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(trike_launch_path)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # Node(
        #     package='trike',
        #     executable='trike', # TODO: Fix
        #     parameters=[
        #         param_dir,
        #         {'odometry.frame_id': PythonExpression(['"', namespace, '/odom"'])},
        #         {'odometry.child_frame_id': PythonExpression(
        #             ['"', namespace, '/base_footprint"'])}],
        #     arguments=['-i', usb_port],
        #     output='screen'),
    ])