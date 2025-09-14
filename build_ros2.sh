#!/bin/bash

# ROS 2 Build Script
# This script will attempt to build the workspace with automatic fixes for common issues

set -e  # Exit on error

# Configuration
WORKSPACE="/root/Dojo"
MAX_ATTEMPTS=3
BUILD_FLAGS="--symlink-install --event-handlers console_direct+"

# Change to workspace directory
cd "$WORKSPACE" || { echo "Failed to change to workspace directory"; exit 1; }

# Function to fix common issues
fix_issues() {
    echo "🔧 Applying fixes..."
    
    # Fix permissions
    echo "  Fixing permissions..."
    find . -user root -exec chown -R $(whoami):$(id -gn) {} \; || true
    chmod -R a+rw .
    
    # Ensure required directories exist
    echo "  Creating required directories..."
    mkdir -p src build install log
    
    # Install system dependencies
    echo "  Installing system dependencies..."
    apt-get update && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential || true
    
    # Update pip and install specific versions of Python packages
    echo "  Setting up Python environment..."
    python3 -m pip install --upgrade pip
    # Install specific version of setuptools that works with ROS 2 Humble
    pip3 install 'setuptools<66.0.0' wheel vcstool colcon-common-extensions
    
    # Install package dependencies
    if [ -f "requirements.txt" ]; then
        pip3 install -r requirements.txt
    fi
    
    # Initialize rosdep if needed
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        echo "  Initializing rosdep..."
        sudo rosdep init || true
        rosdep update
    fi
    
    # Install ROS dependencies
    echo "  Installing ROS dependencies..."
    rosdep install --from-paths src --ignore-src -r -y || true
}

# Function to build a specific package
build_package() {
    local pkg=$1
    local attempt=1
    
    echo "📦 Building package: $pkg"
    
    while [ $attempt -le $MAX_ATTEMPTS ]; do
        echo "  Attempt $attempt of $MAX_ATTEMPTS..."
        
        # Clean previous build artifacts
        rm -rf "build/$pkg" "install/$pkg"
        
        # Special handling for arduino_bridge
        if [ "$pkg" = "arduino_bridge" ]; then
            echo "  Applying special handling for arduino_bridge..."
            if [ -f "src/arduino_bridge/setup.py" ]; then
                # Backup original setup.py
                cp "src/arduino_bridge/setup.py" "src/arduino_bridge/setup.py.bak"
                
                # Clean up setup.py
                sed -i 's/tests_require=\[.*\],//g' src/arduino_bridge/setup.py
                sed -i 's/--editable//g' src/arduino_bridge/setup.py
                
                # Try direct installation with clean environment
                echo "  Trying direct installation with clean environment..."
                cd "src/arduino_bridge"
                
                # Create a clean virtual environment
                python3 -m venv /tmp/arduino_bridge_venv
                source /tmp/arduino_bridge_venv/bin/activate
                pip install 'setuptools<66.0.0' wheel
                
                if python setup.py install; then
                    deactivate
                    rm -rf /tmp/arduino_bridge_venv
                    echo "✅ Successfully installed $pkg with clean environment"
                    cd "$WORKSPACE"
                    return 0
                fi
                
                deactivate
                rm -rf /tmp/arduino_bridge_venv
                cd "$WORKSPACE"
                
                # Restore original setup.py
                mv "src/arduino_bridge/setup.py.bak" "src/arduino_bridge/setup.py"
            fi
        fi
        
        # Try building with colcon
        if colcon build --packages-select "$pkg" $BUILD_FLAGS; then
            echo "✅ Successfully built $pkg"
            # Source the package after successful build
            if [ -f "install/setup.bash" ]; then
                source install/setup.bash
            fi
            return 0
        fi
        
        echo "⚠️  Attempt $attempt failed"
        ((attempt++))
    done
    
    echo "❌ Failed to build $pkg after $MAX_ATTEMPTS attempts"
    return 1
}

# Main build function
build_workspace() {
    echo "🚀 Starting build process..."
    
    # Source ROS 2 environment
    source "/opt/ros/humble/setup.bash"
    
    # Apply fixes first
    fix_issues
    
    # Get list of all packages
    local all_packages=($(colcon list -n))
    
    # Define build order manually to ensure dependencies are met
    local packages=()
    
    # First build order attempt (most independent packages first)
    local build_order=(
        "robot_description"
        "robot_sensors"
        "robot_perception"
        "robot_control"
        "robot_navigation"
        "robot_bringup"
        "arduino_bridge"
        "ros2arduino_bridge"
        "vision_system"
    )
    
    # Filter out packages that don't exist in the workspace
    for pkg in "${build_order[@]}"; do
        if [[ " ${all_packages[*]} " =~ " $pkg " ]]; then
            packages+=("$pkg")
        fi
    done
    
    # Add any remaining packages not in the predefined order
    for pkg in "${all_packages[@]}"; do
        if [[ ! " ${packages[*]} " =~ " $pkg " ]]; then
            packages+=("$pkg")
            echo "⚠️  Package $pkg not in predefined build order, adding to end"
        fi
    done
    
    # Build packages one by one with dependency handling
    local success=true
    local remaining_attempts=3
    
    while [ $remaining_attempts -gt 0 ]; do
        success=true
        local built_something=false
        
        for pkg in "${packages[@]}"; do
            # Skip if already built successfully
            if [ -f "install/$pkg/share/$pkg/package.sh" ]; then
                continue
            fi
            
            echo "🔍 Checking dependencies for $pkg..."
            local deps_met=true
            
            # Get package dependencies
            local deps=$(colcon info "$pkg" 2>/dev/null | grep '^\s*'"$pkg"'\s' | grep -oP '\S+$' || echo "")
            
            # Check if all dependencies are built
            for dep in $deps; do
                if [ ! -f "install/$dep/share/$dep/package.sh" ]; then
                    echo "  ⏳ Waiting for dependency: $dep"
                    deps_met=false
                    break
                fi
            done
            
            if [ "$deps_met" = true ]; then
                echo "🚀 Building $pkg (dependencies met)"
                if build_package "$pkg"; then
                    built_something=true
                    # Source the environment after successful build
                    if [ -f "install/setup.bash" ]; then
                        source install/setup.bash
                    fi
                else
                    success=false
                fi
            fi
        done
        
        # If we didn't build anything this round, we're either done or stuck
        if [ "$built_something" = false ]; then
            if [ "$success" = true ]; then
                echo "✅ All buildable packages have been built successfully"
                break
            else
                echo "⚠️  No progress made this round, but there are still build failures"
                ((remaining_attempts--))
                echo "   Remaining attempts: $remaining_attempts"
                if [ $remaining_attempts -le 0 ]; then
                    echo "❌ Giving up after maximum retry attempts"
                    exit 1
                fi
            fi
        fi
    done
    
    # Final build to catch any remaining packages
    echo "🏗️  Performing final build..."
    if colcon build $BUILD_FLAGS --packages-skip-build-finished; then
        echo "✨ Build completed successfully!"
        echo "Source the workspace with:"
        echo "  source $WORKSPACE/install/setup.bash"
    else
        echo "❌ Final build failed"
        exit 1
    fi
}

# Run the build process
build_workspace
