cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
action(
  FILES
    launch/display.launch.py
    launch/robot_state_publisher.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF and Xacro files
install(
  DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install Python modules
ament_python_install_package(robot_description
  PACKAGE_DIR src/robot_description/robot_description)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
