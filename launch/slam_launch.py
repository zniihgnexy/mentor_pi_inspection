#!/usr/bin/env python3
"""Launch SLAM for mapping mode - use this first to build the classroom map."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mentor_pi_inspection')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam_params_file', default_value=slam_params),

        # SLAM Toolbox in mapping mode
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),

        # Our SLAM management node
        Node(
            package='mentor_pi_inspection',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[{'mode': 'mapping'}],
        ),

        # Lidar interface
        Node(
            package='mentor_pi_inspection',
            executable='lidar_interface',
            name='lidar_interface',
            output='screen',
        ),

        Node(
            package='mentor_pi_inspection',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
        ),
    ])
