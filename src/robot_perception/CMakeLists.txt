cmake_minimum_required(VERSION 3.8)
project(robot_perception)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/robot_perception)

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

# Install Python package as a script
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/robot_perception/camera_processor.py
  ${CMAKE_CURRENT_SOURCE_DIR}/robot_perception/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Make Python scripts executable
file(GLOB PYTHON_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/robot_perception/*.py"
)

foreach(script ${PYTHON_SCRIPTS})
  get_filename_component(script_name ${script} NAME)
  install(PROGRAMS
    ${script}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script_name}
  )
  # Make the script executable
  execute_process(
    COMMAND chmod +x ${script}
  )
endforeach()

ament_package()
