#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --------------------------------------------------
    # Paths
    # --------------------------------------------------
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('airavat_nav2'),
        'config',
        'nav2_params.yaml'
    ])

    map_yaml_file = PathJoinSubstitution([
        FindPackageShare('airavat_nav2'),
        'maps',
        'my_map_nav2test.yaml'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('airavat_nav2'),
        'rviz2',
        'nav2_view.rviz'
    ])

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated clock (false for real robot)'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2 lifecycle nodes'
    )

    # --------------------------------------------------
    # Nav2 bringup (map_server + AMCL + navigation)
    # --------------------------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart
        }.items()
    )

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz)

    return ld
