cmake_minimum_required(VERSION 3.8)
project(quadcopter_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(xacro REQUIRED)

# Include directories
include_directories(include)

# No C++ executables - using Python nodes only

# Install targets - none for C++

# Install directories
install(DIRECTORY launch config urdf rviz worlds scripts
  DESTINATION share/${PROJECT_NAME}/)

# Install Python scripts - list all scripts explicitly
install(PROGRAMS
  scripts/drone_controller.py
  scripts/drone_teleop.py
  scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()