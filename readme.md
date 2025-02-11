# Running Your Differential Drive Robot with ROS2 Navigation

This guide outlines the steps to launch and run your differential drive robot system using ROS2, including YDLidar, micro-ROS, Cartographer SLAM, and Navigation2.

**Prerequisites**

Before you begin, ensure you have the following:

*   **ROS2 Humble Hawksbill** (or your ROS2 distribution) installed and your environment sourced.
*   **Required ROS2 Packages Installed:**
    You need to install the necessary ROS2 packages. Open a terminal and run:

    ```bash
    sudo apt update
    sudo apt install ros-humble-ydlidar-ros2-driver ros-humble-micro-ros-agent ros-humble-nav2-bringup ros-humble-rviz2
    # Install your custom robot package (replace 'my_diff_drive_robot' with your actual package name)
    sudo apt install ros-humble-my-diff-drive-robot
    # If you built your custom package from source using colcon, ensure it's built and sourced.
    ```

    **Note:** Replace `ros-humble-my-diff-drive-robot` with the actual name of your ROS2 package containing your robot's launch files and configurations. If you are building your `my_diff_drive_robot` package from source, make sure you have built it using `colcon build` and sourced your ROS2 environment in each terminal you use.

*   **Micro-ROS Firmware Uploaded to Microcontroller:**
    Make sure you have flashed your microcontroller (e.g., Arduino, ESP32) with the appropriate micro-ROS firmware that communicates over serial and publishes/subscribes to the necessary ROS2 topics (e.g., sensor data, motor commands). Ensure the baud rate in your micro-ROS firmware is set to `115200`.
*   **YDLidar Sensor Connected:**
    Ensure your YDLidar sensor is physically connected to your robot and your computer.
*   **Robot Hardware Setup:**
    Your differential drive robot should be properly assembled with motors, encoders (if used for odometry), and power supply.

**Running the Robot System - Open Five Terminals**

You will need to open **five separate terminal windows** to run each of the following commands concurrently.

**Terminal 1: Launch YDLidar Driver**

1.  Open a new terminal.
2.  Source your ROS2 environment:

    ```bash
    source /opt/ros/humble/setup.bash  # Or your ROS2 setup file
    ```

3.  Launch the YDLidar driver:

    ```bash
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py
    ```

    **Explanation:** This command starts the driver for your YDLidar sensor, enabling it to publish laser scan data.

**Terminal 2: Run micro-ROS Agent**

1.  Open a **new terminal**.
2.  Source your ROS2 environment:

    ```bash
    source /opt/ros/humble/setup.bash # Or your ROS2 setup file
    ```

3.  Run the micro-ROS agent for serial communication:

    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -b 115200
    ```

    **Important:**
    *   **Verify Serial Port:** Double-check that `/dev/ttyUSB1` is the correct serial port for your microcontroller connection. You can use tools like `ls /dev/ttyUSB*` or `dmesg | grep ttyUSB` to help identify the correct port. If it's different, **replace `/dev/ttyUSB1` with the correct port**.
    *   **Baud Rate Matching:** Ensure the baud rate `-b 115200` matches the baud rate configured in your micro-ROS firmware on your microcontroller.

    **Explanation:** This command starts the micro-ROS agent, bridging communication between your robot's microcontroller and ROS2 over a serial connection.

**Terminal 3: Launch Cartographer SLAM**

1.  Open a **new terminal**.
2.  Source your ROS2 environment:

    ```bash
    source /opt/ros/humble/setup.bash # Or your ROS2 setup file
    ```

3.  Launch Cartographer SLAM:

    ```bash
    ros2 launch my_diff_drive_robot cartographer.launch.py
    ```

    **Important:** Replace `my_diff_drive_robot` with the actual name of your robot package if it's different.

    **Explanation:** This command launches the Cartographer SLAM algorithm, which will build a map of your environment using data from your YDLidar and potentially odometry from micro-ROS.

**Terminal 4: Launch Navigation2**

1.  Open a **new terminal**.
2.  Source your ROS2 environment:

    ```bash
    source /opt/ros/humble/setup.bash # Or your ROS2 setup file
    ```

3.  Launch Navigation2:

    ```bash
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False
    ```

    **Explanation:** This command launches the Navigation2 stack, providing navigation capabilities to your robot. It will use the map created by Cartographer to plan paths and navigate.

**Terminal 5: Run RViz2 with Navigation2 View**

1.  Open a **new terminal**.
2.  Source your ROS2 environment:

    ```bash
    source /opt/ros/humble/setup.bash # Or your ROS2 setup file
    ```

3.  Run RViz2 with the Navigation2 default configuration:

    ```bash
    ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
    ```

    **Explanation:** This command launches RViz2 with a configuration optimized for visualizing your Navigation2 setup. You will be able to see the map, robot pose, laser scans, navigation plans, and more in RViz2.

**Verifying and Testing**

*   **RViz2 Visualization:** In RViz2 (Terminal 5), you should start seeing data being visualized. Check for:
    *   Laser Scan data from your YDLidar.
    *   A map being built in the map display (after moving your robot around).
    *   Robot model visualized at its estimated pose.
    *   Navigation plans (once you set goals in Navigation2).

*   **Robot Movement (If Applicable):** If you have motor control implemented via micro-ROS, you can start sending navigation goals through Navigation2 (e.g., using the 2D Goal button in RViz2) to test autonomous navigation.

**Troubleshooting**

*   **No Lidar Data in RViz2:**
    *   Check if Terminal 1 (YDLidar driver) is running without errors.
    *   Verify that your YDLidar sensor is properly connected and powered on.
    *   Check the topics being published by the YDLidar driver using `ros2 topic list` and `ros2 topic echo /scan` (or the appropriate scan topic name).
*   **micro-ROS Agent Connection Issues:**
    *   Check Terminal 2 (micro-ROS agent) for error messages.
    *   Verify the serial port (`/dev/ttyUSB1`) and baud rate (`115200`) are correct.
    *   Ensure your micro-ROS firmware is correctly uploaded and running on your microcontroller.
    *   Test serial communication separately if needed to rule out basic serial port issues.
*   **Cartographer or Navigation2 Not Launching:**
    *   Check Terminals 3 and 4 for any error messages when launching Cartographer and Navigation2.
    *   Verify that your `my_diff_drive_robot` package is correctly set up and built (if it's a custom package).
    *   Check for missing dependencies or configuration errors in your launch files.
*   **RViz2 Errors:**
    *   If RViz2 crashes or shows errors, check Terminal 5 for error messages.
    *   Ensure that the RViz2 configuration file path is correct for your ROS2 distribution.

This detailed guide should help you set up and run your differential drive robot system. Remember to adapt package names and serial port configurations to match your specific setup. Good luck!