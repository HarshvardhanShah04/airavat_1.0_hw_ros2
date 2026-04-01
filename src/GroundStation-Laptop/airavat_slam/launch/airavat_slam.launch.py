from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('airavat_slam'), 
        'config', 
        'mapper_params_online_async.yaml'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('airavat_slam'), 
        'rviz', 
        'slam_config.rviz'
    ])

    rviz2 = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output='screen',
        arguments = ['-d', rviz_config_file]
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
            # Add if needed:
            # remappings=[('scan', '/your_lidar_topic')]
        ),
        
        rviz2
    ])