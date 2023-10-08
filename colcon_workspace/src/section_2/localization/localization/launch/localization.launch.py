import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'urdf/passat_ika_carla.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('localization'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config = 'config/localization_config.rviz'
    rviz_config_path = os.path.join(
        get_package_share_directory('localization'),
        rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d'+str(rviz_config_path)]),

        Node(
            package='tf_broadcaster',
            executable='tf_broadcaster_node',
            name='tf_broadcaster_node',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='localization',
            executable='gnss_localization_node',
            name='gnss_localization_node',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}]),
            
        Node(
                package='kiss_icp',
                executable='odometry_node',
                output="screen",
                remappings=[('pointcloud_topic', '/lidar/pointcloud')],
                parameters=[{
                        'publish_alias_tf': False,
                        'publish_odom_tf': True,
                        'odom_frame': 'odom',
                        'child_frame': 'ego_vehicle/lidar',
                        'max_range': 100.0,
                        'min_range': 5.0,
                        'deskew': False,
                        'max_points_per_voxel': 20,
                        'initial_threshold': 2.0,
                        'min_motion_th': 0.1,
                    }],
                name='lidar_odometry_node'),
    ])
