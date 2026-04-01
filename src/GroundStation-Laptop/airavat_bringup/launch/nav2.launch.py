#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('airavat_bringup')
    nav2_pkg    = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(
        get_package_share_directory('airavat_nav2'),
        'config',
        'nav2_params.yaml'
    )

    map_yaml = os.path.join(
        get_package_share_directory('airavat_nav2'),
        'maps',
        'my_map_nav2test.yaml'
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'core.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': map_yaml
        }.items()
    )

    return LaunchDescription([
        core_launch,
        nav2_launch
    ])
