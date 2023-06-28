import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Get the package and params directory
    image_segmentation_dir = get_package_share_directory('pointcloud_segmentation_r2')
    config = os.path.join(image_segmentation_dir, "config","params.yaml")
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time')
        
    # ROSBAG PLAY node
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play','--rate', '0.75', '-l',
             '/home/rosuser/ws/bag/lidar_campus_melaten',
        ],
        output='screen'
    )
        # CAMERA SEGMENTATION NODE
    pointcloud_segmentation_node = Node(
        package='pointcloud_segmentation_r2',
        name='pointcloud_segmentation',
        executable='pointcloud_segmentation',
        output='screen',
        parameters=[config]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(use_sim_time)
    ld.add_action(rosbag_play_node)
    ld.add_action(pointcloud_segmentation_node)

    return ld
