cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py"
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.yaml"
)

# Install rviz config
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.rviz"
)

# Install world files
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.world"
)

# Install scripts
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
