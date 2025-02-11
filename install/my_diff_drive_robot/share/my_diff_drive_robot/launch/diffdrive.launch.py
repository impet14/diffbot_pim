from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='my_diff_drive_robot')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/my_diff_drive.urdf.xacro')

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('my_diff_drive_robot'), 'rviz', 'urdf.rviz']),
    )

    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('my_diff_drive_robot'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_dd_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'diff_drive_base_controller'],
        output='screen'
    )
    rqt_robot_steering_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
    )
    return LaunchDescription([
        package_arg,
        model_arg,
        rvizconfig_arg,
        gazebo_launch,
        rviz_node,
        load_joint_state_controller,
        load_dd_controller,
        rqt_robot_steering_node
    ])









# # File: my_diff_drive_robot/launch/diffdrive.launch.py

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     # Arguments for URDF (if using a separate description system)
#     package_arg = DeclareLaunchArgument(
#         'urdf_package',
#         default_value='my_diff_drive_robot',
#         description='Package with the robot description'
#     )
#     model_arg = DeclareLaunchArgument(
#         'urdf_package_path',
#         default_value='urdf/my_diff_drive.urdf.xacro',
#         description='Path to the URDF Xacro'
#     )

#     # RViz config argument
#     rvizconfig_arg = DeclareLaunchArgument(
#         name='rvizconfig',
#         default_value=PathJoinSubstitution([
#             FindPackageShare('my_diff_drive_robot'),
#             'rviz',
#             'view_robot.rviz'
#         ]),
#         description='Path to RVIZ config file'
#     )

#     # Include the Gazebo + spawn
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('my_diff_drive_robot'),
#                 'launch',
#                 'gazebo.launch.py'
#             ])
#         ]),
#         launch_arguments={
#             'urdf_package': LaunchConfiguration('urdf_package'),
#             'urdf_package_path': LaunchConfiguration('urdf_package_path')
#         }.items(),
#     )

#     # RViz2
#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         arguments=['-d', LaunchConfiguration('rvizconfig')],
#     )

#     # Load + activate joint_state_broadcaster
#     load_joint_state_broadcaster = ExecuteProcess(
#         cmd=[
#             'ros2', 'control', 'load_controller',
#             '--set-state', 'active',  # instead of "start"
#             'joint_state_broadcaster'
#         ],
#         output='screen'
#     )

#     # Load + activate diff_drive_controller
#     load_diff_drive_controller = ExecuteProcess(
#         cmd=[
#             'ros2', 'control', 'load_controller',
#             '--set-state', 'active',  # instead of "start"
#             'diff_drive_controller'   # must match your YAML name
#         ],
#         output='screen'
#     )

#     # rqt_robot_steering
#     rqt_robot_steering_node = Node(
#         package='rqt_robot_steering',
#         executable='rqt_robot_steering',
#         output='screen'
#     )

#     return LaunchDescription([
#         package_arg,
#         model_arg,
#         rvizconfig_arg,
#         gazebo_launch,
#         rviz_node,
#         load_joint_state_broadcaster,
#         load_diff_drive_controller,
#         rqt_robot_steering_node
#     ])
