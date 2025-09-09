#!/bin/bash

# Create config directories for all packages
for pkg in robot_perception robot_control robot_sensors arduino_bridge ros2arduino_bridge; do
    mkdir -p "/home/Dojo/Dojo/src/$pkg/launch"
    mkdir -p "/home/Dojo/Dojo/src/$pkg/config"
    mkdir -p "/home/Dojo/Dojo/src/$pkg/launch"
    
    # Create a basic launch file if it doesn't exist
    if [ ! -f "/home/Dojo/Dojo/src/$pkg/launch/${pkg}.launch.py" ]; then
        cat > "/home/Dojo/Dojo/src/$pkg/launch/${pkg}.launch.py" << 'EOL'
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Add your launch configurations here
    ])
EOL
    fi
    
    # Create a basic config file if it doesn't exist
    if [ ! -f "/home/Dojo/Dojo/src/$pkg/config/${pkg}_params.yaml" ]; then
        cat > "/home/Dojo/Dojo/src/$pkg/config/${pkg}_params.yaml" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
EOL
    fi
done

echo "Created missing directories and files"
