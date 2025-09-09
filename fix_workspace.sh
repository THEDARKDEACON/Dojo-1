#!/bin/bash

# Exit on error
set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting workspace fix script...${NC}"

# Define workspace directory
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR="${WORKSPACE_DIR}/src"

echo -e "${GREEN}Workspace directory: ${WORKSPACE_DIR}${NC}"

# Create required directories for all packages
create_directories() {
    echo -e "${YELLOW}Creating required directories...${NC}"
    
    # List of all packages
    local packages=(
        "robot_control"
        "robot_description"
        "robot_navigation"
        "robot_perception"
        "robot_sensors"
        "arduino_bridge"
        "ros2arduino_bridge"
        "robot_bringup"
    )
    
    for pkg in "${packages[@]}"; do
        echo "  - Setting up ${pkg}..."
        
        # Create standard directories
        mkdir -p "${SRC_DIR}/${pkg}/launch"
        mkdir -p "${SRC_DIR}/${pkg}/config"
        mkdir -p "${SRC_DIR}/${pkg}/resource"
        
        # For Python packages
        mkdir -p "${SRC_DIR}/${pkg}/src/${pkg}"
        touch "${SRC_DIR}/${pkg}/src/${pkg}/__init__.py"
        touch "${SRC_DIR}/${pkg}/resource/${pkg}"
        
        # For robot_description meshes
        if [ "${pkg}" == "robot_description" ]; then
            mkdir -p "${SRC_DIR}/${pkg}/meshes"
        fi
    done
}

# Fix CMake files
fix_cmake_files() {
    echo -e "${YELLOW}Fixing CMake files...${NC}"
    
    # Find and replace 'action(' with 'install(' in all CMakeLists.txt files
    find "${SRC_DIR}" -name "CMakeLists.txt" -type f -exec sed -i 's/action(/install(/g' {} \;
}

# Create navigation config files
create_nav_configs() {
    echo -e "${YELLOW}Creating navigation configuration files...${NC}"
    
    local nav_dir="${SRC_DIR}/robot_navigation/config"
    
    # Create map_server_params.yaml
    cat > "${nav_dir}/map_server_params.yaml" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
    yaml_filename: map.yaml
    topic_name: map
    frame_id: map
EOL

    # Create nav2_params.yaml
    cat > "${nav_dir}/nav2_params.yaml" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
    autostart: True
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    global_frame: map
    robot_base_frame: base_link
    robot_radius: 0.22
    transform_tolerance: 0.2
    planner_server:
      ros__parameters:
        expected_planner_frequency: 20.0
    controller_server:
      ros__parameters:
        controller_frequency: 20.0
EOL

    # Create other required config files with minimal content
    for file in "localization_params.yaml" "bt_navigator_params.yaml" "planner_params.yaml" \
                "controller_params.yaml" "costmap_common_params.yaml" \
                "global_costmap_params.yaml" "local_costmap_params.yaml"; do
        cat > "${nav_dir}/${file}" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
EOL
    done
}

# Setup Python packages
setup_python_packages() {
    echo -e "${YELLOW}Setting up Python packages...${NC}"
    
    local python_packages=(
        "robot_control"
        "robot_sensors"
        "arduino_bridge"
        "ros2arduino_bridge"
    )
    
    for pkg in "${python_packages[@]}"; do
        echo "  - Setting up ${pkg}..."
        
        # Create setup.py if it doesn't exist
        if [ ! -f "${SRC_DIR}/${pkg}/setup.py" ]; then
            cat > "${SRC_DIR}/${pkg}/setup.py" << EOL
from setuptools import find_packages, setup

package_name = '${pkg}'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
EOL
        fi
        
        # Create package.xml if it doesn't exist
        if [ ! -f "${SRC_DIR}/${pkg}/package.xml" ]; then
            cat > "${SRC_DIR}/${pkg}/package.xml" << EOL
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${pkg}</name>
  <version>0.0.1</version>
  <description>TODO: Package description</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOL
        fi
    done
}

# Main execution
main() {
    echo -e "${GREEN}Starting workspace fix process...${NC}"
    
    # Check if running in the correct directory
    if [ ! -d "${SRC_DIR}" ]; then
        echo -e "${YELLOW}Error: 'src' directory not found. Please run this script from your workspace root.${NC}"
        exit 1
    fi
    
    # Run all fix functions
    create_directories
    fix_cmake_files
    create_nav_configs
    setup_python_packages
    
    echo -e "${GREEN}Workspace fix completed successfully!${NC}"
    echo -e "${YELLOW}You can now build the workspace with:${NC}"
    echo "  colcon build --symlink-install"
}

# Run main function
main
