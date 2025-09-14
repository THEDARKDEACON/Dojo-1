#!/bin/bash

# Exit on error
set -e

# Set workspace directory
WORKSPACE="/home/robosync/Robot/Dojo"
cd "$WORKSPACE"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Function to fix common issues
fix_issues() {
    echo "🔧 Fixing common issues..."
    
    # Fix permissions
    sudo chmod -R a+rw "$WORKSPACE"
    
    # Create necessary directories if they don't exist
    mkdir -p "$WORKSPACE/install"
    mkdir -p "$WORKSPACE/build"
    mkdir -p "$WORKSPACE/log"
    
    # Remove problematic PPA if it exists
    echo "🔄 Removing problematic PPA if it exists..."
    if [ -f "/etc/apt/sources.list.d/unison-team-ubuntu-unison-stable-jammy.list" ]; then
        sudo rm /etc/apt/sources.list.d/unison-team-ubuntu-unison-stable-jammy.list
    fi
    
    # Install build dependencies
    echo "📦 Installing build dependencies..."
    # Update package lists ignoring any errors from problematic repositories
    sudo apt-get update -o Acquire::AllowInsecureRepositories=true || true
    
    # Install required packages
    sudo apt-get install -y --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep
    
    # Initialize rosdep if needed
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Install package dependencies
    echo "📦 Installing package dependencies..."
    
    # First, install common ROS 2 development tools
    echo "🔄 Installing ROS 2 development tools..."
    sudo apt-get install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
        python3-colcon-common-extensions
    
    # Update rosdep with custom rules for missing packages
    echo "🔄 Updating rosdep with custom rules..."
    sudo rosdep fix-permissions
    rosdep update --include-eol-distros
    
    # Install dependencies, ignoring any missing packages
    echo "📦 Installing package dependencies (this may take a while)..."
    rosdep install --from-paths src --ignore-src -r -y || true
    
    # Install specific ROS 2 Humble packages that might be missing
    echo "📦 Installing ROS 2 Humble specific packages..."
    sudo apt-get install -y \
        ros-humble-gazebo-ros \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-gazebo-plugins \
        ros-humble-ament-cmake \
        ros-humble-ament-cmake-python \
        python3-ament-package \
        python3-colcon-ros || true
    
    # Install Python dependencies
    pip3 install -U setuptools wheel vcstool colcon-common-extensions
}

# Function to build a single package
build_package() {
    local pkg=$1
    echo "📦 Building package: $pkg"
    
    # Try to build the package
    if colcon build --packages-select "$pkg" --symlink-install; then
        echo "✅ Successfully built $pkg"
        # Source the workspace to make the package available
        if [ -f "$WORKSPACE/install/setup.bash" ]; then
            source "$WORKSPACE/install/setup.bash"
        fi
        return 0
    else
        echo "❌ Failed to build $pkg"
        return 1
    fi
}

# Main build function
build_workspace() {
    echo "🚀 Starting build process on Raspberry Pi..."
    
    # Fix common issues first
    fix_issues
    
    # Get list of all packages
    local all_packages=($(colcon list -n))
    
    # Define build order (most independent packages first)
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
    local packages=()
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
            if [ -f "$WORKSPACE/install/$pkg/share/$pkg/package.sh" ]; then
                continue
            fi
            
            echo "🔍 Checking dependencies for $pkg..."
            local deps_met=true
            
            # Get package dependencies
            local deps=$(colcon info "$pkg" 2>/dev/null | grep '^\s*'"$pkg"'\s' | grep -oP '\S+$' || echo "")
            
            # Check if all dependencies are built
            for dep in $deps; do
                if [ ! -f "$WORKSPACE/install/$dep/share/$dep/package.sh" ]; then
                    echo "  ⏳ Waiting for dependency: $dep"
                    deps_met=false
                    break
                fi
            done
            
            if [ "$deps_met" = true ]; then
                echo "🚀 Building $pkg (dependencies met)"
                if build_package "$pkg"; then
                    built_something=true
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
    if colcon build --symlink-install; then
        echo "✨ Build completed successfully!"
        echo "Source the workspace with:"
        echo "  source $WORKSPACE/install/setup.bash"
    else
        echo "❌ Final build failed"
        exit 1
    fi
}

# Run the build
build_workspace
