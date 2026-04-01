import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    robot_description_path = os.path.join(get_package_share_directory('airavat_description'),
    'urdf',
    'my_airavat_1.0_compiled.urdf.xacro'
    )
    robot_description_value = ParameterValue(Command([FindExecutable(name='xacro'), ' ', robot_description_path]), value_type=str)

    rviz_config_file = os.path.join(get_package_share_directory('airavat_description'),
    'rviz',
    'rviz_config.rviz'
    )


    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'robot_description':robot_description_value
        }]
    )


    rviz2 = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output='screen',
        arguments = ['-d', rviz_config_file]
    )


    return LaunchDescription([
        robot_state_publisher,
        rviz2
    ])