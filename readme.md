ros2 launch ydlidar_ros2_driver ydlidar_launch.py
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200
ros2 launch my_diff_drive_robot cartographer.launch.py
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz