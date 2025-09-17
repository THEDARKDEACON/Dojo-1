#!/bin/bash

# ROS 2 Build Script for Local Development
# Updated for the new Dojo robot architecture

set -e  # Exit on error

# Configuration
WORKSPACE_DIR="$(pwd)"
MAX_ATTEMPTS=3
BUILD_FLAGS="--symlink-install --event-handlers console_direct+"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
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

# Function to check if we're in a ROS workspace
check_workspace() {
    if [ ! -d "src" ]; then
        log_error "Not in a ROS workspace (no 'src' directory found)"
        log_info "Please run this script from your ROS workspace root"
        exit 1
    fi
    
    log_info "Building workspace: $WORKSPACE_DIR"
}

# Function to source ROS environment
source_ros() {
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_info "Sourcing ROS 2 Humble environment"
        source "/opt/ros/humble/setup.bash"
    else
        log_error "ROS 2 Humble not found. Please install ROS 2 Humble first."
        exit 1
    fi
}

# Function to install dependencies
install_dependencies() {
    log_info "Installing/updating dependencies..."
    
    # Check if rosdep is initialized
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        log_info "Initializing rosdep..."
        sudo rosdep init || true
    fi
    
    # Update rosdep
    rosdep update || log_warning "Failed to update rosdep"
    
    # Install workspace dependencies
    log_info "Installing ROS package dependencies..."
    rosdep install --from-paths src --ignore-src -r -y || log_warning "Some dependencies may not have been installed"
    
    # Install Python dependencies if requirements.txt exists
    if [ -f "requirements.txt" ]; then
        log_info "Installing Python dependencies from requirements.txt..."
        pip3 install -r requirements.txt || log_warning "Failed to install some Python dependencies"
    fi
}

# Function to clean build artifacts
clean_build() {
    log_info "Cleaning previous build artifacts..."
    rm -rf build/ install/ log/
    mkdir -p build install log
}

# Function to get package list in dependency order
get_build_order() {
    # Packages that should be excluded from build (legacy/redundant)
    local excluded_packages=(
        "arduino_bridge"        # Replaced by robot_hardware
        "ros2arduino_bridge"    # Replaced by robot_hardware  
        "robot_sensors"         # Replaced by robot_hardware
        "vision_system"         # Replaced by robot_perception
        "camera_ws"             # Replaced by robot_hardware
        "nv21_converter_pkg"    # Legacy package
        "robot_launch"          # Replaced by robot_bringup

    )
    
    # Get all packages in workspace
    local all_packages=($(colcon list -n 2>/dev/null || echo ""))
    
    # Filter out excluded packages
    local filtered_packages=()
    for pkg in "${all_packages[@]}"; do
        local exclude=false
        for excluded in "${excluded_packages[@]}"; do
            if [ "$pkg" = "$excluded" ]; then
                exclude=true
                log_warning "Excluding legacy/redundant package: $pkg"
                break
            fi
        done
        if [ "$exclude" = false ]; then
            filtered_packages+=("$pkg")
        fi
    done
    
    # Recommended build order for new architecture
    local build_order=(
        "robot_interfaces"      # Custom messages (no dependencies)
        "robot_description"     # Robot models (depends on interfaces)
        "robot_hardware"        # Hardware drivers (depends on interfaces)
        "robot_control"         # High-level control (depends on hardware)
        "robot_perception"      # Computer vision (optional)
        "robot_navigation"      # Navigation (optional)
        "robot_gazebo"          # Simulation (depends on description)
        "robot_bringup"         # System orchestration (depends on all)
    )
    
    # Build packages in order, then add any remaining
    local ordered_packages=()
    
    # Add packages in preferred order
    for pkg in "${build_order[@]}"; do
        if [[ " ${filtered_packages[*]} " =~ " $pkg " ]]; then
            ordered_packages+=("$pkg")
        fi
    done
    
    # Add any remaining packages not in the predefined order
    for pkg in "${filtered_packages[@]}"; do
        if [[ ! " ${ordered_packages[*]} " =~ " $pkg " ]]; then
            ordered_packages+=("$pkg")
            log_info "Adding package not in predefined order: $pkg"
        fi
    done
    
    echo "${ordered_packages[@]}"
}

# Function to build workspace
build_workspace() {
    log_info "Starting build process..."
    
    # Get packages to build
    local packages=($(get_build_order))
    
    if [ ${#packages[@]} -eq 0 ]; then
        log_warning "No packages found to build"
        return 0
    fi
    
    log_info "Found ${#packages[@]} packages to build:"
    for pkg in "${packages[@]}"; do
        echo "  - $pkg"
    done
    
    # Try building all packages at once first (fastest)
    log_info "Attempting to build all packages..."
    if colcon build $BUILD_FLAGS --packages-select "${packages[@]}"; then
        log_success "All packages built successfully!"
        return 0
    fi
    
    log_warning "Batch build failed, trying individual package builds..."
    
    # Build packages individually if batch build fails
    local failed_packages=()
    
    for pkg in "${packages[@]}"; do
        log_info "Building package: $pkg"
        
        local attempt=1
        local success=false
        
        while [ $attempt -le $MAX_ATTEMPTS ] && [ "$success" = false ]; do
            if [ $attempt -gt 1 ]; then
                log_info "Retry attempt $attempt for $pkg"
                # Clean package-specific build artifacts
                rm -rf "build/$pkg" "install/$pkg"
            fi
            
            if colcon build $BUILD_FLAGS --packages-select "$pkg"; then
                log_success "Successfully built $pkg"
                success=true
                
                # Source the workspace after each successful build
                if [ -f "install/setup.bash" ]; then
                    source install/setup.bash
                fi
            else
                log_warning "Build attempt $attempt failed for $pkg"
                ((attempt++))
            fi
        done
        
        if [ "$success" = false ]; then
            log_error "Failed to build $pkg after $MAX_ATTEMPTS attempts"
            failed_packages+=("$pkg")
        fi
    done
    
    # Report results
    if [ ${#failed_packages[@]} -eq 0 ]; then
        log_success "All packages built successfully!"
    else
        log_error "Failed to build ${#failed_packages[@]} packages:"
        for pkg in "${failed_packages[@]}"; do
            echo "  - $pkg"
        done
        return 1
    fi
}

# Function to run tests (optional)
run_tests() {
    if [ "$1" = "--test" ]; then
        log_info "Running tests..."
        colcon test --packages-select $(get_build_order | tr ' ' '\n' | head -5 | tr '\n' ' ') || log_warning "Some tests failed"
        colcon test-result --verbose || true
    fi
}

# Function to display usage information
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --clean     Clean build artifacts before building"
    echo "  --deps      Install dependencies before building"
    echo "  --test      Run tests after building"
    echo "  --help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Standard build"
    echo "  $0 --clean --deps     # Clean build with dependency installation"
    echo "  $0 --test             # Build and run tests"
}

# Main execution
main() {
    local clean_build_flag=false
    local install_deps_flag=false
    local run_tests_flag=false
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                clean_build_flag=true
                shift
                ;;
            --deps)
                install_deps_flag=true
                shift
                ;;
            --test)
                run_tests_flag=true
                shift
                ;;
            --help)
                show_usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    # Execute build steps
    check_workspace
    source_ros
    
    if [ "$install_deps_flag" = true ]; then
        install_dependencies
    fi
    
    if [ "$clean_build_flag" = true ]; then
        clean_build
    fi
    
    build_workspace
    
    if [ "$run_tests_flag" = true ]; then
        run_tests --test
    fi
    
    # Final instructions
    log_success "Build completed!"
    echo ""
    log_info "To use the workspace, run:"
    echo "  source install/setup.bash"
    echo ""
    log_info "To launch the robot system:"
    echo "  ros2 launch robot_bringup bringup.launch.py"
    echo ""
    log_info "To visualize the robot:"
    echo "  ros2 launch robot_description display.launch.py"
}

# Run main function with all arguments
main "$@"