#!/bin/bash

# List of Python packages that need fixing
packages=("robot_control" "robot_sensors" "arduino_bridge" "ros2arduino_bridge")

for pkg in "${packages[@]}"; do
    echo "Fixing $pkg..."
    
    # Create __init__.py
    mkdir -p "/home/Dojo/Dojo/src/$pkg/src/$pkg"
    touch "/home/Dojo/Dojo/src/$pkg/src/$pkg/__init__.py"
    
    # Create resource directory and file
    mkdir -p "/home/Dojo/Dojo/src/$pkg/resource"
    touch "/home/Dojo/Dojo/src/$pkg/resource/$pkg"
    
    # Create a basic setup.py if it doesn't exist
    if [ ! -f "/home/Dojo/Dojo/src/$pkg/setup.py" ]; then
        cat > "/home/Dojo/Dojo/src/$pkg/setup.py" << 'EOL'
from setuptools import find_packages, setup

package_name = '$pkg'

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
        # Replace the package name in setup.py
        sed -i "s/\$pkg/$pkg/g" "/home/Dojo/Dojo/src/$pkg/setup.py"
    fi
    
    # Create a basic package.xml if it doesn't exist
    if [ ! -f "/home/Dojo/Dojo/src/$pkg/package.xml" ]; then
        cat > "/home/Dojo/Dojo/src/$pkg/package.xml" << 'EOL'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$pkg</name>
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
        # Replace the package name in package.xml
        sed -i "s/\$pkg/$pkg/g" "/home/Dojo/Dojo/src/$pkg/package.xml"
    fi
done

echo "All Python packages have been fixed."
