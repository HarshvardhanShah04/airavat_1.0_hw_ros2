#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable



def generate_launch_description():
    robot_description_path = os.path.join(get_package_share_directory('airavat_description'),
    'urdf',
    'my_airavat_1.0_compiled.urdf.xacro'
    )

    robot_description_value = ParameterValue(Command([FindExecutable(name='xacro'), ' ', robot_description_path]), value_type=str)


    description_pkg = get_package_share_directory('airavat_description')
    ekf_params = os.path.join(
        get_package_share_directory('ekf_node'),
        'config',
        'ekf.yaml'
    )

    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'robot_description':robot_description_value
    }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    return LaunchDescription([
        robot_state_publisher,
        ekf_node
    ])
