from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_pkg = 'my_diff_drive_robot'
    default_model_path = PathJoinSubstitution([
        FindPackageShare(urdf_pkg), 'urdf', 'my_diff_drive.urdf.xacro'
    ])
    default_rviz_config_path = PathJoinSubstitution([
        FindPackageShare(urdf_pkg), 'rviz', 'urdf.rviz'
    ])

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to the URDF/xacro file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    # We can use the generic URDF launch from "urdf_launch" or create our own
    # For demonstration, let's assume we have "urdf_launch" installed:
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py'])
        ]),
        launch_arguments={
            'urdf_package': urdf_pkg,
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')
        }.items()
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        included_launch
    ])
