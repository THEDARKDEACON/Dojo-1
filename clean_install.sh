#!/bin/bash

# Clean Install Directory Script
# Removes legacy packages from install directory

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Legacy packages to remove from install directory
legacy_packages=(
    "arduino_bridge"
    "ros2arduino_bridge"
    "robot_sensors"
    "vision_system"
    "camera_ws"
    "nv21_converter_pkg"
    "robot_launch"
)

log_info "Cleaning legacy packages from install directory..."

# Check if install directory exists
if [ ! -d "install" ]; then
    log_warning "No install directory found"
    exit 0
fi

# Remove legacy packages
removed_count=0
for pkg in "${legacy_packages[@]}"; do
    if [ -d "install/$pkg" ]; then
        log_info "Removing legacy package: $pkg"
        rm -rf "install/$pkg"
        ((removed_count++))
    fi
done

# Also clean build and log directories for these packages
if [ -d "build" ]; then
    for pkg in "${legacy_packages[@]}"; do
        if [ -d "build/$pkg" ]; then
            log_info "Removing build artifacts for: $pkg"
            rm -rf "build/$pkg"
        fi
    done
fi

if [ -d "log" ]; then
    for pkg in "${legacy_packages[@]}"; do
        if [ -d "log/latest_build/$pkg" ]; then
            rm -rf "log/latest_build/$pkg"
        fi
    done
fi

log_success "Removed $removed_count legacy packages from install directory"

# Regenerate setup files
if [ -f "install/setup.bash" ]; then
    log_info "Regenerating setup files..."
    # The setup files will be regenerated on next build
    log_success "Setup files will be regenerated on next build"
fi

log_success "Install directory cleanup complete!"
echo ""
log_info "Next steps:"
echo "  1. Run: ./build_ros2.sh --clean"
echo "  2. Source: source install/setup.bash"
echo "  3. Test: ros2 launch robot_gazebo gazebo.launch.py"