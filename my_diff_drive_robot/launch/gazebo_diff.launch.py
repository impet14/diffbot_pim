#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your robot state publisher launch file
    rsp_launch = os.path.join(
        get_package_share_directory('my_diff_drive_robot'),
        'launch',
        'robot_state_publisher.launch.py'
    )

    # Gazebo launch file from gazebo_ros package
    gazebo_ros_pkgs = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch'
    )

    return LaunchDescription([
        # Include the robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch)
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkgs, 'gazebo.launch.py')
            ),
            launch_arguments={
                'verbose': 'true'
            }.items()
        ),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_diffbot',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'diffbot'
            ],
            output='screen'
        ),
    ])
