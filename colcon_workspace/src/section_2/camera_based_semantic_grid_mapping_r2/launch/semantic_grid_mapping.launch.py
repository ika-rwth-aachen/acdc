import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the package and params directory
    semantic_grid_mapping_dir = get_package_share_directory('camera_based_semantic_grid_mapping_r2')
    config = os.path.join(semantic_grid_mapping_dir, 'config', 'params.yaml')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock time')

    # ROSBAG PLAY node
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--rate', '0.1', '-l',
             '/home/rosuser/ws/bag/semantic_8_cams.db3'],
        output='screen'
    )

    # SEMANTIC GRID MAPPING NODE
    semantic_grid_mapping_node = Node(
        package='camera_based_semantic_grid_mapping_r2',
        name='camera_based_semantic_grid_mapping',
        executable='semantic_grid_mapping',
        output='screen',
        parameters=[config]
    )

    # NODES FOR VISUALIZATION
    front_left_segmented_image_node = Node(
        package='image_view',
        executable='image_view',
        name='front_left_segmented_image',
        remappings=[('image', 'carla/ego_vehicle/semantic_segmentation_front_left_1/image')],
    )

    front_right_segmented_image_node = Node(
        package='image_view',
        executable='image_view',
        name='front_right_segmented_image',
        remappings=[('image', 'carla/ego_vehicle/semantic_segmentation_front_right_1/image')],
    )

    segmentation_viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='segmentation_viewer',
        remappings=[('image', 'BEV_image')],
            parameters=[
                {'autosize': True},
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(use_sim_time)
    ld.add_action(rosbag_play_node)
    ld.add_action(semantic_grid_mapping_node)
    ld.add_action(front_left_segmented_image_node)
    ld.add_action(front_right_segmented_image_node)
    ld.add_action(segmentation_viewer_node)

    return ld
