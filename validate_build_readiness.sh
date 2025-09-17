#!/bin/bash

# ROS2 Build Readiness Validator for Raspberry Pi
# Run this before attempting to build to catch issues early

set -e

WORKSPACE="/home/robosync/Robot/Dojo"
ERRORS=0
WARNINGS=0

echo "🔍 ROS2 Build Readiness Validator"
echo "=================================="

# Color codes for output
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_error() {
    echo -e "${RED}❌ ERROR: $1${NC}"
    ERRORS=$((ERRORS + 1))
}

log_warning() {
    echo -e "${YELLOW}⚠️  WARNING: $1${NC}"
    WARNINGS=$((WARNINGS + 1))
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# Check if we're in the right directory
check_workspace() {
    echo "📁 Checking workspace structure..."
    
    if [ ! -d "$WORKSPACE" ]; then
        log_error "Workspace directory $WORKSPACE does not exist"
        return 1
    fi
    
    cd "$WORKSPACE"
    
    if [ ! -d "src" ]; then
        log_error "No 'src' directory found in workspace"
        return 1
    fi
    
    log_success "Workspace structure looks good"
}

# Check ROS2 environment
check_ros_environment() {
    echo "🤖 Checking ROS2 environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS_DISTRO environment variable not set"
        echo "  Run: source /opt/ros/humble/setup.bash"
        return 1
    fi
    
    if [ "$ROS_DISTRO" != "humble" ]; then
        log_warning "Expected ROS_DISTRO=humble, found: $ROS_DISTRO"
    fi
    
    if ! command -v colcon &> /dev/null; then
        log_error "colcon build tool not found"
        echo "  Install with: sudo apt install python3-colcon-common-extensions"
        return 1
    fi
    
    log_success "ROS2 environment configured"
}

# Check Python environment
check_python_environment() {
    echo "🐍 Checking Python environment..."
    
    local python_version=$(python3 --version 2>&1 | cut -d' ' -f2)
    local major=$(echo $python_version | cut -d. -f1)
    local minor=$(echo $python_version | cut -d. -f2)
    
    if [[ $major -lt 3 ]] || { [[ $major -eq 3 ]] && [[ $minor -lt 8 ]]; }; then
        log_error "Python 3.8+ required, found: $python_version"
        return 1
    fi
    
    # Check critical Python packages
    local required_pkgs=("setuptools" "wheel" "packaging")
    for pkg in "${required_pkgs[@]}"; do
        if ! python3 -c "import $pkg" &>/dev/null; then
            log_error "Missing Python package: $pkg"
        fi
    done
    
    # Check for empy (common issue)
    if ! python3 -c "import em" &>/dev/null; then
        log_warning "empy package not found - this may cause build issues"
        echo "  Install with: sudo apt install python3-empy"
    fi
    
    log_success "Python environment looks good"
}

# Check package structure
check_package_structure() {
    echo "📦 Checking package structure..."
    
    local packages=($(colcon list --names-only 2>/dev/null))
    
    if [ ${#packages[@]} -eq 0 ]; then
        log_error "No packages found in workspace"
        return 1
    fi
    
    log_info "Found ${#packages[@]} packages: ${packages[*]}"
    
    # Check each package for required files
    for pkg in "${packages[@]}"; do
        local pkg_path=$(find src -name "$pkg" -type d | head -1)
        
        if [ -z "$pkg_path" ]; then
            log_warning "Package $pkg not found in src directory"
            continue
        fi
        
        # Check for package.xml
        if [ ! -f "$pkg_path/package.xml" ]; then
            log_error "Package $pkg missing package.xml"
        fi
        
        # Check build type and required files
        local build_type=$(grep -o 'ament_cmake\|ament_python' "$pkg_path/package.xml" | head -1)
        
        if [ "$build_type" = "ament_cmake" ]; then
            if [ ! -f "$pkg_path/CMakeLists.txt" ]; then
                log_error "CMake package $pkg missing CMakeLists.txt"
            fi
        elif [ "$build_type" = "ament_python" ]; then
            if [ ! -f "$pkg_path/setup.py" ] && [ ! -f "$pkg_path/setup.cfg" ]; then
                log_error "Python package $pkg missing setup.py or setup.cfg"
            fi
        fi
    done
    
    log_success "Package structure validation complete"
}

# Check system dependencies
check_system_dependencies() {
    echo "🔧 Checking system dependencies..."
    
    # Check if rosdep is initialized
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        log_warning "rosdep not initialized"
        echo "  Run: sudo rosdep init && rosdep update"
    fi
    
    # Try to check dependencies (but don't fail if some are missing)
    log_info "Running rosdep check..."
    if rosdep check --from-paths src --ignore-src -r &>/dev/null; then
        log_success "All system dependencies satisfied"
    else
        log_warning "Some system dependencies may be missing"
        echo "  Run: rosdep install --from-paths src --ignore-src -r -y"
    fi
}

# Check for common Pi-specific issues
check_pi_specific() {
    echo "🥧 Checking Raspberry Pi specific considerations..."
    
    # Check available memory
    local mem_total=$(grep MemTotal /proc/meminfo | awk '{print $2}')
    local mem_gb=$((mem_total / 1024 / 1024))
    
    if [ $mem_gb -lt 4 ]; then
        log_warning "Only ${mem_gb}GB RAM detected. Consider using --parallel-workers 1 for colcon build"
    fi
    
    # Check for swap
    local swap_total=$(grep SwapTotal /proc/meminfo | awk '{print $2}')
    if [ $swap_total -eq 0 ]; then
        log_warning "No swap space configured. This may cause build failures on Pi"
    fi
    
    # Check disk space
    local disk_avail=$(df . | tail -1 | awk '{print $4}')
    local disk_gb=$((disk_avail / 1024 / 1024))
    
    if [ $disk_gb -lt 2 ]; then
        log_warning "Less than 2GB disk space available"
    fi
    
    log_success "Pi-specific checks complete"
}

# Check for conflicting packages
check_conflicts() {
    echo "🔍 Checking for potential conflicts..."
    
    # Check for duplicate camera packages
    if [ -d "src/camera_ws" ] && [ -d "src/robot_sensors" ]; then
        log_warning "Found both camera_ws and robot_sensors - potential duplicate camera nodes"
    fi
    
    # Check for Gazebo packages on Pi (usually not needed)
    if [ -d "src/robot_gazebo" ]; then
        log_info "Found robot_gazebo package - consider excluding on Pi for faster builds"
    fi
    
    log_success "Conflict check complete"
}

# Generate build recommendations
generate_recommendations() {
    echo ""
    echo "📋 Build Recommendations"
    echo "========================"
    
    if [ $ERRORS -gt 0 ]; then
        echo -e "${RED}🚫 BUILD NOT RECOMMENDED${NC}"
        echo "   Fix $ERRORS error(s) before building"
    elif [ $WARNINGS -gt 0 ]; then
        echo -e "${YELLOW}⚠️  BUILD WITH CAUTION${NC}"
        echo "   $WARNINGS warning(s) detected - build may succeed but could have issues"
    else
        echo -e "${GREEN}✅ READY TO BUILD${NC}"
        echo "   All checks passed!"
    fi
    
    echo ""
    echo "Recommended build command for Pi:"
    echo "  colcon build --symlink-install --parallel-workers 2 --packages-skip robot_gazebo"
    echo ""
    echo "If build fails, try:"
    echo "  colcon build --symlink-install --parallel-workers 1 --continue-on-error"
}

# Main execution
main() {
    check_workspace || exit 1
    check_ros_environment
    check_python_environment
    check_package_structure
    check_system_dependencies
    check_pi_specific
    check_conflicts
    
    generate_recommendations
    
    if [ $ERRORS -gt 0 ]; then
        exit 1
    else
        exit 0
    fi
}

# Run the validation
main "$@"