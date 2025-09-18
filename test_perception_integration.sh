#!/bin/bash

# Test script for perception package integration
# This script tests if the perception package is properly integrated

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

log_info "Testing Perception Package Integration"

# Check if perception package exists
if [ ! -d "src/robot_perception" ]; then
    log_error "Perception package not found!"
    exit 1
fi

log_success "Perception package found"

# Check if package.xml exists and is valid
if [ ! -f "src/robot_perception/package.xml" ]; then
    log_error "package.xml not found!"
    exit 1
fi

log_success "package.xml found"

# Check if setup.py exists
if [ ! -f "src/robot_perception/setup.py" ]; then
    log_error "setup.py not found!"
    exit 1
fi

log_success "setup.py found"

# Check if main modules exist
if [ ! -f "src/robot_perception/robot_perception/camera_processor.py" ]; then
    log_error "camera_processor.py not found!"
    exit 1
fi

if [ ! -f "src/robot_perception/robot_perception/object_detector.py" ]; then
    log_error "object_detector.py not found!"
    exit 1
fi

log_success "Main perception modules found"

# Check if launch files exist
if [ ! -f "src/robot_perception/launch/perception.launch.py" ]; then
    log_error "perception.launch.py not found!"
    exit 1
fi

log_success "Launch files found"

# Check if config file exists
if [ ! -f "src/robot_perception/config/robot_perception_params.yaml" ]; then
    log_error "Configuration file not found!"
    exit 1
fi

log_success "Configuration file found"

# Test if package can be built
log_info "Testing package build..."

if colcon build --packages-select robot_perception --event-handlers console_direct+; then
    log_success "Perception package builds successfully"
else
    log_error "Failed to build perception package"
    exit 1
fi

# Source the workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    log_success "Workspace sourced"
else
    log_warning "install/setup.bash not found, skipping sourcing"
fi

# Test if nodes can be found
log_info "Testing node executables..."

if ros2 pkg executables robot_perception | grep -q "camera_processor"; then
    log_success "camera_processor executable found"
else
    log_error "camera_processor executable not found"
    exit 1
fi

if ros2 pkg executables robot_perception | grep -q "object_detector"; then
    log_success "object_detector executable found"
else
    log_error "object_detector executable not found"
    exit 1
fi

# Test launch file syntax
log_info "Testing launch file syntax..."

if ros2 launch robot_perception perception.launch.py --show-args > /dev/null 2>&1; then
    log_success "Launch file syntax is valid"
else
    log_error "Launch file has syntax errors"
    exit 1
fi

# Check integration with bringup
log_info "Checking integration with main bringup..."

if grep -q "robot_perception" src/robot_bringup/launch/bringup.launch.py; then
    log_success "Perception integrated in main bringup"
else
    log_warning "Perception not found in main bringup launch file"
fi

# Check topic compatibility
log_info "Checking topic compatibility..."

# Check if camera topics are consistent
if grep -q "image_raw" src/robot_perception/launch/perception.launch.py && \
   grep -q "image_raw" src/robot_hardware/robot_hardware/drivers/camera_driver.py; then
    log_success "Camera topics are compatible"
else
    log_warning "Camera topic compatibility issues detected"
fi

log_success "Perception package integration test completed successfully!"

echo ""
log_info "To test perception with camera:"
echo "  ros2 launch robot_bringup bringup.launch.py use_perception:=true use_camera:=true"
echo ""
log_info "To test perception in simulation:"
echo "  ros2 launch robot_gazebo simulation.launch.py use_perception:=true"
echo ""
log_info "To monitor perception topics:"
echo "  ros2 topic list | grep perception"
echo "  ros2 topic echo /perception/detection_info"