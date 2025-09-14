#!/bin/bash

# Build script for ROS 2 Humble workspace in Docker
# Single file solution for building the workspace
# Usage: ./build_script.sh [--clean]

set -e  # Exit on error

# Configuration
WORKSPACE="/root/Dojo"
MAX_RETRIES=3
BUILD_FLAGS="--symlink-install --executor sequential --event-handlers console_direct+"

# Set environment
export COLCON_HOME=$WORKSPACE
export MAKEFLAGS="-j$(($(nproc) / 2))"  # Use half the available cores

# Fix permissions
fix_permissions() {
    echo "🔧 Fixing permissions..."
    find $WORKSPACE -user root -exec chown -R $(whoami):$(id -gn) {} \; || true
    chmod -R a+rw $WORKSPACE
}

# Install dependencies
install_dependencies() {
    echo "📦 Installing dependencies..."
    
    # Update pip and install basic packages
    python3 -m pip install --upgrade pip
    pip3 install -U setuptools wheel
    
    # Install from requirements.txt if it exists
    if [ -f "$WORKSPACE/requirements.txt" ]; then
        pip3 install -r $WORKSPACE/requirements.txt
    fi
    
    # Install ROS dependencies
    echo "  Installing ROS dependencies..."
    rosdep install --from-paths $WORKSPACE/src --ignore-src -r -y || true
}

# Build the workspace
build_workspace() {
    echo "🚀 Building workspace..."
    cd $WORKSPACE
    
    # Clean if requested
    if [[ "$*" == *"--clean"* ]]; then
        echo "🧹 Cleaning build directories..."
        rm -rf build/ install/ log/
    fi
    
    # Create necessary directories
    mkdir -p $WORKSPACE/build $WORKSPACE/install $WORKSPACE/log
    
    # Build packages one by one with retries
    for pkg in $(colcon list -n); do
        echo "📦 Building package: $pkg"
        local attempt=1
        local pkg_flags="$BUILD_FLAGS"
        
        # Special handling for arduino_bridge
        if [ "$pkg" = "arduino_bridge" ]; then
            echo "  Applying workaround for arduino_bridge..."
            # Remove any --editable flag from setup.py if it exists
            if [ -f "src/arduino_bridge/setup.py" ]; then
                sed -i 's/--editable//g' src/arduino_bridge/setup.py
            fi
            # Add --no-deps to prevent pip from trying to install in editable mode
            pkg_flags="$pkg_flags --symlink-install"
        fi
        
        while [ $attempt -le $MAX_RETRIES ]; do
            if colcon build --packages-select $pkg $pkg_flags; then
                echo "✅ Successfully built $pkg"
                break
            else
                if [ $attempt -eq $MAX_RETRIES ]; then
                    echo "❌ Failed to build $pkg after $MAX_RETRIES attempts"
                    # For arduino_bridge, try installing with pip directly as a fallback
                    if [ "$pkg" = "arduino_bridge" ] && [ -f "src/arduino_bridge/setup.py" ]; then
                        echo "  Trying alternative installation method for arduino_bridge..."
                        cd src/arduino_bridge
                        if python3 setup.py install; then
                            echo "✅ Successfully installed $pkg using direct setup.py"
                            cd $WORKSPACE
                            continue 2  # Continue with next package
                        fi
                        cd $WORKSPACE
                    fi
                    return 1
                fi
                echo "⚠️  Attempt $attempt failed, retrying $pkg..."
                rm -rf build/$pkg install/$pkg
                ((attempt++))
            fi
        done
    done
    
    # Final build to catch any remaining packages
    echo "🏗️  Final build to catch any remaining packages..."
    colcon build $BUILD_FLAGS --packages-skip-build-finished
}

# Main execution
main() {
    echo "=== ROS 2 Workspace Build Script ==="
    echo "Workspace: $WORKSPACE"
    
    # Source ROS 2 environment
    source /opt/ros/humble/setup.bash
    
    # Fix permissions first
    fix_permissions
    
    # Install dependencies
    install_dependencies
    
    # Build the workspace
    if build_workspace "$@"; then
        echo "✅ Build completed successfully!"
        echo "Source the workspace with:"
        echo "  source $WORKSPACE/install/setup.bash"
    else
        echo "❌ Build failed!"
        exit 1
    fi
}

# Run the main function
main "$@"
