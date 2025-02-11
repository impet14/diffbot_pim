# nav2_bringup.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Common arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    slam = LaunchConfiguration('slam', default='true')  # Set slam:=true for online SLAM
    params_file = LaunchConfiguration('params_file')

    # Path to your package and default nav2_params.yaml
    pkg_my_diff_drive_robot = get_package_share_directory('my_diff_drive_robot')
    default_nav2_params_path = os.path.join(pkg_my_diff_drive_robot, 'config', 'nav2_params.yaml')

    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack'
    )
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Run Nav2 in SLAM mode (skip AMCL/map_server)'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params_path,
        description='Full path to the Nav2 parameter file to use'
    )

    # Include the official Nav2 bringup launch (navigation_launch.py)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'slam': slam
        }.items()
    )

    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch arguments so they can be overridden on the command line
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_slam)
    ld.add_action(declare_params_file)

    # NOTE: No static transform publisher here since YDLIDAR launch already does 'base_link->laser_frame'

    # Add the Nav2 bringup to the launch description
    ld.add_action(navigation_launch)

    return ld
