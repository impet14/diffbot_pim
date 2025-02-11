# my_diff_drive_robot/launch/cartographer.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Path to this package
    pkg_my_diff_drive_robot = get_package_share_directory('my_diff_drive_robot')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(pkg_my_diff_drive_robot, 'config')
    )

    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='diffbot_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # Example: We use the TB3 Cartographer RViz config here. If you prefer Nav2’s RViz config,
    # you can remove this Node or replace it.
    # rviz_config_dir = os.path.join(
    #     get_package_share_directory('turtlebot3_cartographer'),
    #     'rviz',
    #     'tb3_cartographer.rviz'
    # )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to Cartographer config directory'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of the Cartographer .lua file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                # e.g., if your lidar topic is /scan
                ('scan', '/scan')
            ]
        ),

        # OccupancyGrid Node (from occupancy_grid.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        # (Optional) RViz for Cartographer
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2_cartographer',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),
    ])
