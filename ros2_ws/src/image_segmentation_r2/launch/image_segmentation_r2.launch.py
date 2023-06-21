import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("image_segmentation_r2"),
        "config",
        "params.yaml"
    )

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock time')

    """ # ROSBAG PLAY node
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--rate', '0.05', '-l',
             '/home/rosuser/ws/bag/left_camera_templergraben',
             '--topics', '/sensors/camera/left/image_raw',
             '/sensors/camera/left/camera_info'],
        output='screen'
    )"""


    # STEREO IMAGE PROC node
    stereo_image_proc_node = Node(
        package='stereo_image_proc',
        name='stereo_image_proc',
        executable='stereo_image_proc',
        namespace='sensors/camera/',
        output='screen'
    )

    # CAMERA SEGMENTATION NODE
    camera_segmentation_node = Node(
        package='image_segmentation_r2',
        name='image_segmentation',
        executable='image_segmentation',
        output='screen',
        parameters=[
            {'params_file': os.path.join(get_package_share_directory('image_segmentation_r2'), 'launch', 'params.yaml')}
        ],
        remappings=[
            ('/image_rect_color', '/sensors/camera/left/image_rect_color')
        ]
    )

    # NODES FOR VISUALIZATION
    segmentation_viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='segmentation_viewer',
        arguments=['image:=/image_rect_segmented']
    )

    camera_left_node = Node(
        package='image_view',
        executable='image_view',
        name='camera_left',
        arguments=['image:=/sensors/camera/left/image_rect_color']
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(use_sim_time)
    #ld.add_action(rosbag_play_node)
    ld.add_action(stereo_image_proc_node)
    ld.add_action(camera_segmentation_node)
    ld.add_action(segmentation_viewer_node)
    ld.add_action(camera_left_node)

    return ld
