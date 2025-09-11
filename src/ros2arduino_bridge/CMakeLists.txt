cmake_minimum_required(VERSION 3.8)
project(ros2arduino_bridge)

# Default to C++17 if we have C++ code
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python dependencies
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros_py REQUIRED)

# Install Python package
ament_python_install_package(ros2arduino_bridge
  PACKAGE_NAME ros2arduino_bridge
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

# Install Arduino files
install(DIRECTORY
  arduino
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python entry points
install(PROGRAMS
  scripts/arduino_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Add tests if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add pytest tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_arduino_bridge
    test/test_arduino_bridge.py
    TIMEOUT 30
  )
endif()

ament_package()
