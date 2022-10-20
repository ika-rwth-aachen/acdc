from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_path = Path(get_package_share_directory('racing'), 'config', 'racing.yaml')
    vehicle_controller_node = Node(package='racing', executable='vehicle_controller_node', output='screen',
                                   parameters=[parameters_file_path])
    vehicle_timer_node = Node(package='racing', executable='vehicle_timer_node', output='screen',
                              parameters=[parameters_file_path])

    ld = LaunchDescription()

    ld.add_action(vehicle_controller_node)
    ld.add_action(vehicle_timer_node)

    return ld
