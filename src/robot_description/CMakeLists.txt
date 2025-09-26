cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF, meshes, and RViz config
set(DIRS_TO_INSTALL urdf)

# Only include directories that exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
    list(APPEND DIRS_TO_INSTALL meshes)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
    list(APPEND DIRS_TO_INSTALL rviz)
endif()

foreach(dir ${DIRS_TO_INSTALL})
    install(
        DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir}
        PATTERN "*"
    )
endforeach()

# Install all .xacro and .gazebo files
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.xacro"
  PATTERN "*.gazebo"
  PATTERN "*.urdf"
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  PATTERN "*.yaml"
  PATTERN "*.rviz"
)

# Install package.xml
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "package.xml"
  PATTERN "setup.py"
  PATTERN "setup.cfg"
)

# Install Python modules
install(
  DIRECTORY src/robot_description
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Add Python modules to the PYTHONPATH
ament_python_install_package(robot_description
  PACKAGE_DIR src/robot_description
)

# Install executables
install(
  PROGRAMS
    scripts/display_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
