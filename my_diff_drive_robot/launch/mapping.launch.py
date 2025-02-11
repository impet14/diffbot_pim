#!/usr/bin/env python3
"""
Launch slam_toolbox (online_async) with a delay to ensure the TF chain is up.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the slam_toolbox online_async launch file.
    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_share_dir, 'launch', 'online_async_launch.py')
    
    # Declare launch arguments.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    default_slam_params_file = os.path.join(
        get_package_share_directory('my_diff_drive_robot'),  # Change this to your package name.
        'config',
        'slam_toolbox_real.yaml'
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='Full path to the SLAM toolbox parameters file'
    )
    
    # Delay the slam_toolbox launch by 3 seconds.
    slam_toolbox_launch = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()
        )]
    )
    
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(slam_toolbox_launch)
    return ld
