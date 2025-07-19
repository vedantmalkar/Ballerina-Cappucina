cmake_minimum_required(VERSION 3.5)
project(gz_rosa_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Plugin library
add_library(gazebo_ros_omni_drive SHARED
  src/gazebo_ros_omni_drive.cpp
)

# Link dependencies to plugin
ament_target_dependencies(gazebo_ros_omni_drive
  rclcpp
  gazebo_ros
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install the plugin
install(TARGETS
  gazebo_ros_omni_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch and urdf folders
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export the plugin to Gazebo
ament_export_libraries(gazebo_ros_omni_drive)

# Package registration
ament_package()

