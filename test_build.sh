#!/bin/bash
# Quick test script to validate the new architecture build

echo "🧪 Testing Dojo Robot Build..."

# Check if we're in the right directory
if [ ! -f "build_ros2_pi.sh" ]; then
    echo "❌ Error: Run this script from the Dojo workspace root"
    exit 1
fi

# Check for required packages
echo "📦 Checking for required packages..."
required_packages=("robot_hardware" "robot_interfaces" "robot_control" "robot_bringup")
missing_packages=()

for pkg in "${required_packages[@]}"; do
    if [ ! -d "src/$pkg" ]; then
        missing_packages+=("$pkg")
    else
        echo "✅ Found: $pkg"
    fi
done

if [ ${#missing_packages[@]} -gt 0 ]; then
    echo "❌ Missing required packages: ${missing_packages[*]}"
    echo "   Make sure you have the latest repository with new architecture"
    exit 1
fi

# Check for legacy packages that should be excluded
echo "🔍 Checking for legacy packages..."
legacy_packages=("arduino_bridge" "ros2arduino_bridge" "robot_sensors" "vision_system")
found_legacy=()

for pkg in "${legacy_packages[@]}"; do
    if [ -d "src/$pkg" ]; then
        found_legacy+=("$pkg")
    fi
done

if [ ${#found_legacy[@]} -gt 0 ]; then
    echo "⚠️  Found legacy packages: ${found_legacy[*]}"
    echo "   These will be excluded from build (this is normal)"
fi

# Test colcon package discovery
echo "🔍 Testing package discovery..."
if command -v colcon >/dev/null 2>&1; then
    packages=$(colcon list -n 2>/dev/null)
    if [ $? -eq 0 ]; then
        echo "✅ Colcon package discovery working"
        echo "📋 Discovered packages:"
        echo "$packages" | tr ' ' '\n' | sort | sed 's/^/   - /'
    else
        echo "❌ Colcon package discovery failed"
        exit 1
    fi
else
    echo "❌ Colcon not found. Install with: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Test ROS2 environment
echo "🔍 Testing ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS_DISTRO not set. Source ROS2 setup: source /opt/ros/humble/setup.bash"
else
    echo "✅ ROS2 environment: $ROS_DISTRO"
fi

# Check for common dependencies
echo "🔍 Checking Python dependencies..."
python_deps=("serial" "numpy" "cv2")
for dep in "${python_deps[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        echo "✅ Python module: $dep"
    else
        echo "⚠️  Missing Python module: $dep"
    fi
done

echo ""
echo "🎯 Test Summary:"
echo "   Required packages: ${#required_packages[@]}/${#required_packages[@]} found"
echo "   Legacy packages: ${#found_legacy[@]} found (will be excluded)"
echo ""
echo "🚀 Ready to build! Run: ./build_ros2_pi.sh"