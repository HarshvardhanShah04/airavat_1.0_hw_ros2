#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():

    bringup_pkg = get_package_share_directory('airavat_bringup')
    slam_pkg    = get_package_share_directory('airavat_slam')

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'core.launch.py')
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'airavat_slam.launch.py')
        )
    )

    return LaunchDescription([
        core_launch,
        slam_launch
    ])
