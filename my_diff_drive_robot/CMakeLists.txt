cmake_minimum_required(VERSION 3.8)
project(my_diff_drive_robot)

# Enable C++17 (optional but recommended)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
# We will eventually need the following for Gazebo and ros2_control, but only if you plan to build them:
# find_package(controller_manager REQUIRED)
# find_package(diff_drive_controller REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(position_controllers REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(rqt_robot_steering REQUIRED)

# Install URDF, launch, and RViz files
install(DIRECTORY urdf launch rviz config 
  DESTINATION share/${PROJECT_NAME}
)



# If you have any scripts (Python or others) to install, you could do something like:
# install(PROGRAMS scripts/my_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
