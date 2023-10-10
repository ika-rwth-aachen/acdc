import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    params_arg = DeclareLaunchArgument('params', default_value='params.yml')
    config = PathJoinSubstitution([
        get_package_share_directory("carla_gnss_emulator"), "config",
        LaunchConfiguration('params')
    ])

    return LaunchDescription([
        params_arg,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        Node(
            package='carla_gnss_emulator',
            executable='gnss_emulator_node',
            name='carla_gnss_emulator',
            parameters=[config,{'use_sim_time': use_sim_time}],
            output={'both': 'log'},
            ),
    ])
