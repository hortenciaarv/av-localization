import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='localization',
            executable='localization',
            name='localization',
        ),
        # Node(
        #     package='localization',
        #     executable='controller',
        #     name='controller',
        # ),
        Node(
            package='localization',
            executable='coordinate_transform',
            name='coordinate_transform'),
        # Node(
        #     package='localization',
        #     executable='to_goal',
        #     name='to_goal'),
        # Node(
        #     package='localization',
        #     executable='to_goal_bug_2',
        #     name='to_goal_bug_2'),
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
        
    ])

