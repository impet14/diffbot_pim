#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Path to your main Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('my_diff_drive_robot'),
        'urdf',
        'my_diff_drive.urdf.xacro'
    )

    # Process the Xacro file to create URDF
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Create a Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        rsp_node
    ])
