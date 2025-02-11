import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration

def generate_launch_description():
    # Paths
    my_robot_cartographer_dir = get_package_share_directory('my_diff_drive_robot')
    cartographer_config_dir = os.path.join(my_robot_cartographer_dir, 'config')
    cartographer_config_file = 'diffbot_lds_2d.lua'
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(my_robot_cartographer_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1) Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_file
        ],
        remappings=[
            ('scan', '/scan')
        ]
    )
    
    # 2) Occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )

    # 3) Static transform if needed for base_link -> laser_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1', '0.0', '0.2',  # x, y, z offset
            '0', '0', '0', '1',   # quaternion (no rotation)
            'base_link',
            'laser_frame'
        ]
    )
    
    # 4) Nav2 launch (slam = true)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        static_tf,
        navigation_launch
    ])
