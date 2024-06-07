import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # Node(
        #     package='localization',
        #     executable='controller',
        #     name='controller',
        # ),
        Node(
            package='localization',
            executable='coordinate_transform',
            name='coordinate_transform'),
        Node(
            package='camera_viewer',
            executable='camera_viewer',
            name='camera_viewer',
        ),
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            output='screen',
            parameters=[
                {'cam_base_topic': 'camera1/image_raw'},
                {'marker_size': 0.045}
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('camera_info_publisher'), 'launch', 'camera_info.launch.py')
            )
        ),
        Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "base_link", "laser"]
        ),
        Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "base_link", "Imagen"]
        )
        # Node(
        #     package='localization',
        #     executable='localization',
        #     # output='screen',
        #     name='localization',
        # )
        
    ])

