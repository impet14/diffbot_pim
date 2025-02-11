#!/usr/bin/env python3
# gazebo_world.launch.py
"""
Launch file that:
1) Launches Gazebo with a TurtleBot3 world (e.g. turtlebot3_world.world).
2) Spawns *only* your custom diff-drive robot (URDF/Xacro).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ###########
    # 1. LAUNCH ARGUMENTS
    ###########
    # Whether to launch Gazebo's GUI
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable Gazebo GUI'
    )

    # Which TurtleBot3 world file to use
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world.world',
        description='Name of the TurtleBot3 .world file in turtlebot3_gazebo/worlds'
    )

    # Robot URDF package (where your URDF/Xacro is located)
    robot_urdf_package_arg = DeclareLaunchArgument(
        'urdf_package',
        default_value='my_diff_drive_robot',
        description='Package name containing the custom robot URDF/Xacro'
    )

    # Robot URDF file path inside the above package
    robot_urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='urdf/my_diff_drive.urdf.xacro',
        description='Relative path to the robot URDF or Xacro file inside the package'
    )

    # Spawn position arguments (optional)
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='-2.0')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='0.0')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='0.1')

    ###########
    # 2. GAZEBO LAUNCH (Using a TurtleBot3 world)
    ###########
    # We include the standard gazebo_ros launch file but override the 'world' argument
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'worlds',
                LaunchConfiguration('world')
            ]),
            # Decide if you want to start paused or unpaused:
            'pause': 'true'
        }.items()
    )

    ###########
    # 3. SPAWN YOUR CUSTOM DIFF-DRIVE ROBOT
    ###########
    # This assumes you have a "description.launch.py" in another package (e.g. 'urdf_launch')
    # that publishes /robot_description from your URDF. If not, you can directly publish
    # the URDF in this file — but we'll keep it consistent with your approach.
    description_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('urdf_launch'),  # adapt if your description.launch.py is elsewhere
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_file')
        }.items()
    )

    spawn_diffdrive_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_diffdrive_robot',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'my_diff_drive_robot',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-unpause'
        ],
        output='screen'
    )

    ###########
    # 4. BUILD THE LAUNCH DESCRIPTION
    ###########
    ld = LaunchDescription()

    # Declare the arguments
    ld.add_action(gui_arg)
    ld.add_action(world_arg)
    ld.add_action(robot_urdf_package_arg)
    ld.add_action(robot_urdf_file_arg)
    ld.add_action(robot_x_arg)
    ld.add_action(robot_y_arg)
    ld.add_action(robot_z_arg)

    # Include the main Gazebo launch using the chosen TurtleBot3 world
    ld.add_action(gazebo_launch)

    # Launch your URDF description
    ld.add_action(description_launch_py)

    # Spawn your robot
    ld.add_action(spawn_diffdrive_robot)

    return ld
