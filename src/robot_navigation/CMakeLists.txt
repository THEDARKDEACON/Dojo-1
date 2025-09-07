cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

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
    launch/navigation.launch.py
    launch/localization.launch.py
    launch/map_server.launch.py
    config/nav2_params.yaml
    config/localization_params.yaml
    config/map_server_params.yaml
    config/bt_navigator_params.yaml
    config/planner_params.yaml
    config/controller_params.yaml
    config/costmap_common_params.yaml
    config/global_costmap_params.yaml
    config/local_costmap_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
action(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
action(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
