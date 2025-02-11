#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to the Nav2 Bringup launch files
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # Map file (in your package's map directory)
    default_map_file = PathJoinSubstitution([
        get_package_share_directory('my_diff_drive_robot'),
        'map',
        'my_map.yaml'
    ])

    # Nav2 params file (in your config directory)
    default_nav2_params_file = PathJoinSubstitution([
        get_package_share_directory('my_diff_drive_robot'),
        'config',
        'nav2_params.yaml'
    ])

    # Declare launch arguments so we can override if needed
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file to load'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params_file,
        description='Full path to the nav2 parameters file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Include Nav2 bringup launch
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Construct the LaunchDescription
    ld = LaunchDescription()

    # Add the declared arguments
    ld.add_action(map_arg)
    ld.add_action(params_arg)
    ld.add_action(use_sim_time_arg)

    # Add the included Nav2 bringup
    ld.add_action(nav2_bringup)

    return ld
