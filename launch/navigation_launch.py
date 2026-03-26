#!/usr/bin/env python3
"""Launch Nav2 navigation stack for autonomous patrol."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mentor_pi_inspection')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/inspection_maps/classroom_map.yaml'),
            description='Full path to map yaml file'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': map_file
            }],
        ),

        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 behavior server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'map_server', 'amcl', 'planner_server',
                    'controller_server', 'behavior_server',
                    'bt_navigator', 'waypoint_follower'
                ]
            }],
        ),

        # Our path planner node
        Node(
            package='mentor_pi_inspection',
            executable='path_planner',
            name='path_planner',
            output='screen',
        ),

        # Motion controller
        Node(
            package='mentor_pi_inspection',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
        ),
    ])
