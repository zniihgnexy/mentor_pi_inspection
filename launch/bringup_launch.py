#!/usr/bin/env python3
"""
Full system bringup - launches everything needed for inspection patrol.
Usage:
  1. First time: run slam_launch.py to build map, then save it
  2. Then: run this launch file for full inspection

  ros2 launch mentor_pi_inspection bringup_launch.py
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mentor_pi_inspection')
    params = os.path.join(pkg_dir, 'config', 'inspection_params.yaml')

    nav_launch = os.path.join(pkg_dir, 'launch', 'navigation_launch.py')
    inspection_launch = os.path.join(pkg_dir, 'launch', 'inspection_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/inspection_maps/classroom_map.yaml'),
        ),

        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
        ),

        # Inspection nodes (delayed to let nav stack initialize)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(inspection_launch),
                ),
            ],
        ),

        # Inspection manager (delayed further)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='mentor_pi_inspection',
                    executable='inspection_manager',
                    name='inspection_manager',
                    output='screen',
                    parameters=[params],
                ),
            ],
        ),
    ])
