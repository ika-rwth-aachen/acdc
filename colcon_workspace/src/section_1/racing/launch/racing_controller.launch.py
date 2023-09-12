import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    racing_dir = get_package_share_directory('racing')
    config = os.path.join(racing_dir, 'config', 'params.yaml')

    vehicle_controller_node = Node(package='racing', 
                                   executable='vehicle_controller_node', 
                                   output='screen',
                                   parameters=[config])
    
    vehicle_timer_node = Node(package='racing', 
                              executable='vehicle_timer_node',
                              output='screen',
                              parameters=[config])

    ld = LaunchDescription()

    ld.add_action(vehicle_controller_node)
    ld.add_action(vehicle_timer_node)

    return ld
