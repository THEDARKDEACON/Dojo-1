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
    
    # Try to build the package with proper installation paths
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
    
    # Get list of all packages
    local all_packages=($(colcon list -n))
    
    # Define build order with proper dependencies
    local packages=(
        # Core packages with no dependencies
        "robot_description"
        "robot_sensors"
        "arduino_bridge"
        
        # Packages that depend on the above
        "robot_control"
        "ros2arduino_bridge"
        
        # Vision system (depends on core packages)
        "vision_system"
        
        # Launch package (depends on all others)
        "robot_launch"
    )
    
    # Filter out packages that don't exist in the workspace
    local filtered_packages=()
    for pkg in "${packages[@]}"; do
        if [[ " ${all_packages[*]} " =~ " $pkg " ]]; then
            filtered_packages+=("$pkg")
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
    
    # First, build all ament_cmake packages with proper installation paths
    if colcon build \
        --symlink-install \
        --packages-skip-build-finished \
        --event-handlers console_cohesion+ \
        --cmake-args \
            -DCMAKE_INSTALL_PREFIX=install \
            -DCMAKE_INSTALL_LIBDIR=lib \
            -DCMAKE_INSTALL_BINDIR=lib/$pkg \
            -DCMAKE_INSTALL_INCLUDEDIR=include \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=1" \
            --no-warn-unused-cli \
            -Wno-dev \
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
