#!/bin/bash

# Exit on error
set -e

# Set workspace directory
WORKSPACE="/home/robosync/Robot/Dojo"
cd "$WORKSPACE"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Function to fix common issues
# In build_ros2_pi.sh, update the fix_issues function to include:
fix_issues() {
    echo "🔧 Fixing common issues..."
    
    # Ensure rosdep is properly initialized
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        echo "🔄 Initializing rosdep..."
        sudo rosdep init || true
        rosdep update || true
    fi
    
    # Install system dependencies
    echo "📦 Installing system dependencies..."
    sudo apt-get update
    sudo apt-get install -y \
        python3-pip \
        python3-empy \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-setuptools \
        python3-wheel
        
    # Install specific Python packages
    echo "📦 Installing Python packages..."
    # Install system package for empy
    sudo apt-get install -y python3-empy
    
    # Install other Python packages
    pip3 install --user --upgrade \
        'setuptools==59.6.0' \
        'wheel==0.37.1' \
        'packaging==21.3' \
        'setuptools-scm==6.4.2' \
        'empy==3.3.4'
        
    # Verify empy installation
    if ! python3 -c "import em; print('empy version:', em.VERSION)" &>/dev/null; then
        echo "❌ Error: empy is not properly installed."
        echo "Trying alternative installation method..."
        sudo -H pip3 install empy==3.3.4
    fi
}

# Function to verify Python environment
verify_python_environment() {
    echo "🔍 Verifying Python environment..."
    
    # Check for Python version
    local python_version=$(python3 --version 2>&1 | cut -d' ' -f2)
    local major=$(echo $python_version | cut -d. -f1)
    local minor=$(echo $python_version | cut -d. -f2)
    
    if [[ $major -lt 3 ]] || { [[ $major -eq 3 ]] && [[ $minor -lt 8 ]]; }; then
        echo "❌ Python 3.8 or higher is required. Found Python $python_version"
        return 1
    fi
    
    # Check for required packages
    local required_pkgs=("setuptools" "wheel" "packaging" "em")
    for pkg in "${required_pkgs[@]}"; do
        if ! python3 -c "import $pkg" &>/dev/null; then
            echo "❌ Missing required Python package: $pkg"
            return 1
        fi
    done
    
    echo "✅ Python environment looks good!"
    return 0
}

# Function to build a single package
build_package() {
    local pkg=$1
    echo "📦 Building package: $pkg"
    
    # Special handling for ros2arduino_bridge
    if [ "$pkg" = "ros2arduino_bridge" ]; then
        echo "🔧 Building ros2arduino_bridge with user installation..."
        cd "$WORKSPACE/src/ros2arduino_bridge"
        pip3 install --user -e .
        cd "$WORKSPACE"
        return $?
    fi
    
    # Build other packages with standard settings
    if colcon build \
        --packages-select "$pkg" \
        --symlink-install \
        --cmake-args \
            -DCMAKE_INSTALL_PREFIX=install \
            -DCMAKE_INSTALL_LIBDIR=lib \
            -DCMAKE_INSTALL_BINDIR=lib/$pkg \
            -DCMAKE_INSTALL_INCLUDEDIR=include; then
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
    
    # Verify Python environment
    if ! verify_python_environment; then
        echo "❌ Python environment verification failed. Please fix the issues above and try again."
        exit 1
    fi
    
    # Clean previous builds if they exist
    if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
        echo "🧹 Cleaning previous build artifacts..."
        rm -rf build/ install/ log/
    fi
    
    # Create required directories that might be missing
    mkdir -p src/robot_description/config
    touch src/robot_description/config/.keep
    
    # Get list of all packages
    echo "🔍 Discovering all packages in the workspace..."
    local all_packages=($(colcon list -n))
    
    # Define packages to exclude (URDF, Gazebo, and simulation-related)
    local excluded_packages=(
        "robot_gazebo"
        "gazebo_ros2_control"
        "gazebo_ros"
        "gazebo_plugins"
        "gazebo_ros_control"
        "gazebo_dev"
        "gazebo_msgs"
        "gazebo_ros_pkgs"
        "gazebo_simulator"
        "rviz"
        "rviz2"
        "rviz_common"
        "rviz_default_plugins"
        "rviz_rendering"
        "rviz_visual_tools"
    )
    
    # Filter out excluded packages
    local packages=()
    local excluded_count=0
    
    for pkg in "${all_packages[@]}"; do
        local exclude=0
        for excluded in "${excluded_packages[@]}"; do
            if [[ "$pkg" == "$excluded" ]]; then
                exclude=1
                excluded_count=$((excluded_count + 1))
                echo "ℹ️  Excluding package: $pkg (simulation/URDF related)"
                break
            fi
        done
        if [[ $exclude -eq 0 ]]; then
            packages+=("$pkg")
        fi
    done
    
    echo "📦 Found ${#all_packages[@]} total packages, excluded $excluded_count packages"
    echo "📦 Will build ${#packages[@]} packages: ${packages[*]}"
    
    echo "📦 Packages to build (${#packages[@]}): ${packages[*]}"
    
    # Filter out packages that don't exist in the workspace
    local filtered_packages=()
    for pkg in "${packages[@]}"; do
        if [[ " ${all_packages[*]} " =~ " $pkg " ]]; then
            filtered_packages+=("$pkg")
        fi
    done
    
    # Update packages with filtered list
    packages=("${filtered_packages[@]}")
    
    # Don't add back excluded packages - they were excluded for a reason
    echo "✅ Final package list (${#packages[@]} packages): ${packages[*]}"
    
    # Build packages one by one with dependency handling
    local success=true
    local remaining_attempts=3
    local built_packages=()
    
    while [ $remaining_attempts -gt 0 ]; do
        success=true
        local built_something=false
        
        for pkg in "${packages[@]}"; do
            # Skip if already built successfully
            if [[ " ${built_packages[@]} " =~ " ${pkg} " ]]; then
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
            
            # Try to build the package with special handling for ros2arduino_bridge
            if $deps_met; then
                echo "🚀 Building $pkg (attempt $((4 - remaining_attempts))/3)"
                if [ "$pkg" = "ros2arduino_bridge" ]; then
                    echo "🔧 Using special handling for ros2arduino_bridge..."
                    cd "$WORKSPACE/src/ros2arduino_bridge"
                    pip3 install --user -e .
                    if [ $? -eq 0 ]; then
                        built_something=true
                        built_packages+=("$pkg")
                        cd "$WORKSPACE"
                        # Create a dummy package.sh to satisfy dependency checks
                        mkdir -p "$WORKSPACE/install/ros2arduino_bridge/share/ros2arduino_bridge"
                        touch "$WORKSPACE/install/ros2arduino_bridge/share/ros2arduino_bridge/package.sh"
                        chmod +x "$WORKSPACE/install/ros2arduino_bridge/share/ros2arduino_bridge/package.sh"
                    else
                        success=false
                    fi
                    cd "$WORKSPACE"
                else
                    if ! build_package "$pkg"; then
                        success=false
                    else
                        built_something=true
                        built_packages+=("$pkg")
                    fi
                fi
            fi
        done
        
        # If we didn't build anything this round, we're either done or stuck
        if [ "$built_something" = false ]; then
            if [ "$success" = true ]; then
                echo "✅ All buildable packages have been built successfully"
                break
            else
                remaining_attempts=$((remaining_attempts - 1))
                if [ $remaining_attempts -gt 0 ]; then
                    echo "⚠️  No progress made this round, but there are still build failures"
                    echo "   Remaining attempts: $remaining_attempts"
                else
                    echo "❌ Giving up after maximum retry attempts"
                    success=false
                    break
                fi
            fi
        fi
    done
    
    if [ "$success" = true ]; then
        echo "✨ Build completed successfully!"
        echo "Source the workspace with:"
        echo "  source $WORKSPACE/install/setup.bash"
    else
        echo "❌ Build failed for some packages"
        exit 1
    fi
}

# Run the build
build_workspace
