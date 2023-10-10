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
    cpp_pubsub_tutorial_dir = get_package_share_directory('cpp_pubsub_tutorial')
    # Constructing the path to the configuration file 'params.yaml'.
    config = os.path.join(cpp_pubsub_tutorial_dir, 'config', 'params.yaml')

    # Defining a node for the publisher.
    publisher_node = Node(package='cpp_pubsub_tutorial', 
                                   executable='publisher',
                                   name='publisher_node', 
                                   output='screen',
                                   parameters=[config])
    
    # Defining a node for the subscriber.
    subscriber_node = Node(package='cpp_pubsub_tutorial', 
                              executable='subscriber',
                              name='subscriber_node',
                              output='screen',
                              parameters=[config])

    # Creating a LaunchDescription object to store the nodes.
    ld = LaunchDescription()

    # Adding both nodes to the launch description.
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)

    # Returning the launch description, which is used by the ROS2 launch system.
    return ld
