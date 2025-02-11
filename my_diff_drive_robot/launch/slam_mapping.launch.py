#!/usr/bin/env python3
# slam_mapping.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to the slam_toolbox's online_async_launch.py (part of slam_toolbox package)
    slam_toolbox_share_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_share_dir, 'launch', 'online_async_launch.py')

    # (1) Declare launch arguments
    # --------------------------------------------------------------
    # Here, we declare a `use_sim_time` argument that defaults to `true`
    # (assuming you are using Gazebo or any simulation).
    # You can override it from the command line with:
    #   ros2 launch my_package slam.launch.py use_sim_time:=false
    # if running on a real robot.

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # If you have a custom parameter file for Slam Toolbox,
    # specify it. Otherwise, you can rely on default parameters.
    default_slam_params_file = os.path.join(
        get_package_share_directory('my_diff_drive_robot'),  # or whichever package holds your config
        'config',
        'slam_toolbox_params.yaml'
    )
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='Full path to the SLAM toolbox parameters file'
    )

    # (2) Construct the SLAM launch
    # --------------------------------------------------------------
    # We include slam_toolbox’s `online_async_launch.py`, passing in
    # our arguments for sim time and the custom params file.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items()
    )

    # (3) Assemble the LaunchDescription
    # --------------------------------------------------------------
    ld = LaunchDescription()

    # Declare the arguments so they can be set at runtime
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)

    # Include the SLAM launch
    ld.add_action(slam_toolbox_launch)

    return ld
