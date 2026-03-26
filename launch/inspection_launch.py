#!/usr/bin/env python3
"""Launch the inspection-specific nodes (vision, data recording, scheduling)."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mentor_pi_inspection')
    params = os.path.join(pkg_dir, 'config', 'inspection_params.yaml')

    return LaunchDescription([
        # Lidar interface
        Node(
            package='mentor_pi_inspection',
            executable='lidar_interface',
            name='lidar_interface',
            output='screen',
            parameters=[params],
        ),

        # Camera interface
        Node(
            package='mentor_pi_inspection',
            executable='camera_interface',
            name='camera_interface',
            output='screen',
            parameters=[params],
        ),

        # Depth + Lidar fusion
        Node(
            package='mentor_pi_inspection',
            executable='depth_fusion',
            name='depth_fusion',
            output='screen',
            parameters=[params],
        ),

        # Image processor
        Node(
            package='mentor_pi_inspection',
            executable='image_processor',
            name='image_processor',
            output='screen',
            parameters=[params],
        ),

        # Power LED detector
        Node(
            package='mentor_pi_inspection',
            executable='power_led_detector',
            name='power_led_detector',
            output='screen',
            parameters=[params],
        ),

        # Data recorder
        Node(
            package='mentor_pi_inspection',
            executable='data_recorder',
            name='data_recorder',
            output='screen',
            parameters=[params],
        ),

        # Scheduler
        Node(
            package='mentor_pi_inspection',
            executable='scheduler',
            name='scheduler',
            output='screen',
            parameters=[params],
        ),

        # Sensor manager
        Node(
            package='mentor_pi_inspection',
            executable='sensor_manager',
            name='sensor_manager',
            output='screen',
            parameters=[params],
        ),
    ])
