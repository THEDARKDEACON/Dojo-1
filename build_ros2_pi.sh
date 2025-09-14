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
    
    # Fix permissions and ownership
    echo "🔧 Setting up workspace permissions..."
    sudo chown -R $USER:$USER "$WORKSPACE"
    chmod -R u+rwX "$WORKSPACE"
    
    # Create necessary directories if they don't exist
    echo "📁 Creating workspace directories..."
    mkdir -p "$WORKSPACE/install"
    mkdir -p "$WORKSPACE/build"
    mkdir -p "$WORKSPACE/log"
    
    # Clean up any existing build artifacts that might cause conflicts
    echo "🧹 Cleaning up previous build artifacts..."
    rm -rf "$WORKSPACE/install"/* "$WORKSPACE/build"/* "$WORKSPACE/log"/*
    
    # Ensure no root-owned files remain
    if [ -d "$WORKSPACE/build" ] || [ -d "$WORKSPACE/install" ] || [ -d "$WORKSPACE/log" ]; then
        sudo chown -R $USER:$USER "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log" || true
    fi
    
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
    
    # Install ROS 2 Humble development tools and base packages
    echo "📦 Installing ROS 2 Humble development tools..."
    sudo apt-get update
    sudo apt-get install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-vcstool \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-rosdistro \
        python3-ament-package \
        python3-colcon-ros

    # Install Gazebo/ROS integration packages
    echo "📦 Installing ROS 2 Humble Gazebo integration..."
    
    # Install ros_gz packages
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-interfaces
        
    # Install Gazebo Fortress (the version compatible with ROS 2 Humble)
    echo "🔧 Setting up Gazebo Fortress..."
    sudo apt-get install -y \
        wget \
        lsb-release \
        gnupg
        
    # Add Gazebo repository
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo_stable.list > /dev/null
    
    # Install Gazebo
    sudo apt-get update
    sudo apt-get install -y \
        gz-fortress \
        libgz-sim7-dev \
        libgz-common5-dev
        
    # Update environment
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
    source ~/.bashrc
    
    # Verify installation
    if ! command -v gz &> /dev/null; then
        echo "❌ Failed to install Gazebo. Please check the logs and try again."
        exit 1
    fi
    
    echo "✅ Successfully installed Gazebo Fortress and ROS 2 Humble integration"
    
    # Install Python dependencies globally
    echo "🐍 Installing Python dependencies globally..."
    
    # Install system dependencies first
    echo "📦 Installing system dependencies..."
    sudo apt-get update
    sudo apt-get install -y \
        python3-pip \
        python3-empy \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-setuptools \
        python3-wheel
    
    # Install specific versions of pip packages
    echo "📦 Installing Python packages with specific versions..."
    sudo -H pip3 install --upgrade 'pip<24.0'  # Use a slightly older pip version for better compatibility
    sudo -H pip3 install --upgrade \
        'setuptools<70.0.0' \
        'wheel<1.0.0' \
        'setuptools-scm<8.0.0' \
        'setuptools-scm-git-archive<3.0.0' \
        'empy==3.3.4'  # Specific version known to work with ROS 2 Humble
    
    # Verify empy installation
    if ! python3 -c "import em; print('empy version:', em.VERSION)" &>/dev/null; then
        echo "❌ Failed to install empy. Trying alternative installation method..."
        # Try installing empy from source as a fallback
        cd /tmp
        git clone https://github.com/ros/empy.git
        cd empy
        git checkout 3.3.4
        sudo python3 setup.py install
        cd ~
    fi
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
    
    # Clean the build directories to avoid path conflicts
    if [ -d "$WORKSPACE/build" ] || [ -d "$WORKSPACE/install" ]; then
        echo "🧹 Cleaning up build directories..."
        rm -rf "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"
        mkdir -p "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"
    fi
    
    # Run the build with clean environment and proper path handling
    echo "🚀 Starting the build process..."
    
    # First, build all ament_cmake packages
    if colcon build \
        --symlink-install \
        --packages-skip-build-finished \
        --event-handlers console_cohesion+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_INSTALL_PREFIX="$WORKSPACE/install" \
            -DCMAKE_PREFIX_PATH="$WORKSPACE/install" \
            -DCMAKE_INSTALL_LIBDIR=lib; then
        
        # Then build ament_python packages with isolated environment
        echo "🐍 Building Python packages with isolated environment..."
        for pkg in "${packages[@]}"; do
            if [ -f "$WORKSPACE/src/$pkg/setup.py" ]; then
                echo "📦 Building Python package: $pkg"
                (cd "$WORKSPACE" && \
                 PYTHONPATH="" \
                 python3 -m pip install --no-deps --ignore-installed -e "src/$pkg" || \
                 echo "⚠️  Failed to build $pkg, continuing with other packages")
            fi
        done
        
        # Final verification
        if [ $? -eq 0 ]; then
            echo "✨ Build completed successfully!"
            echo "Source the workspace with:"
            echo "  source $WORKSPACE/install/setup.bash"
        else
            echo "❌ Final build failed"
            exit 1
        fi
    else
        echo "❌ Failed to build ament_cmake packages"
        exit 1
    fi
}

# Run the build
build_workspace
