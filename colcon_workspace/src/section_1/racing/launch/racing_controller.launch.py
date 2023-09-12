# Importing necessary modules.
# 'os' is for file and directory operations
import os
# 'get_package_share_directory' is used to find a package's shared directory in a ROS2 environment.
from ament_index_python.packages import get_package_share_directory
# 'LaunchDescription' helps describe the nodes to be launched in a ROS2 system.
from launch import LaunchDescription
# 'Node' is an action that represents a node in the ROS2 graph.
from launch_ros.actions import Node

def generate_launch_description():
    # Fetching the shared directory path for the 'racing' package.
    racing_dir = get_package_share_directory('racing')
    # Constructing the path to the configuration file 'params.yaml'.
    config = os.path.join(racing_dir, 'config', 'params.yaml')

    # Defining a node for the vehicle_controller.
    vehicle_controller_node = Node(package='racing', 
                                   executable='vehicle_controller_node',
                                   name='vehicle_controller_node', 
                                   output='screen',
                                   parameters=[config])
    
    # Defining a node for the vehicle_timer.
    vehicle_timer_node = Node(package='racing', 
                              executable='vehicle_timer_node',
                              name='vehicle_timer_node',
                              output='screen',
                              parameters=[config])

    # Creating a LaunchDescription object to store the nodes.
    ld = LaunchDescription()

    # Adding both nodes to the launch description.
    ld.add_action(vehicle_controller_node)
    ld.add_action(vehicle_timer_node)

    # Returning the launch description, which is used by the ROS2 launch system.
    return ld