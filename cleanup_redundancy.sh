#!/bin/bash
# Cleanup script to remove redundant packages from Dojo Robot codebase

echo "🧹 Cleaning up redundant packages..."

# Function to safely remove a package
remove_package() {
    local package_name=$1
    local replacement=$2
    
    if [ -d "src/$package_name" ]; then
        echo "❌ Removing redundant package: $package_name (replaced by $replacement)"
        
        # Create backup first
        if [ ! -d "backup_packages" ]; then
            mkdir backup_packages
        fi
        
        echo "   📦 Creating backup in backup_packages/$package_name"
        cp -r "src/$package_name" "backup_packages/"
        
        # Remove from src
        rm -rf "src/$package_name"
        
        echo "   ✅ Removed $package_name"
    else
        echo "   ℹ️  Package $package_name not found (already removed?)"
    fi
}

# Function to analyze a package before removal
analyze_package() {
    local package_name=$1
    echo "🔍 Analyzing $package_name..."
    
    if [ -d "src/$package_name" ]; then
        echo "   📁 Files in package:"
        find "src/$package_name" -name "*.py" -o -name "*.launch.py" -o -name "*.yaml" | head -5
        echo "   📊 Total files: $(find "src/$package_name" -type f | wc -l)"
    fi
}

echo ""
echo "=== ANALYSIS PHASE ==="

# Analyze packages before removal
analyze_package "arduino_bridge"
analyze_package "ros2arduino_bridge" 
analyze_package "camera_ws"
analyze_package "robot_sensors"
analyze_package "vision_system"

echo ""
echo "=== CLEANUP PHASE ==="

# Remove redundant Arduino packages
remove_package "arduino_bridge" "robot_hardware"
remove_package "ros2arduino_bridge" "robot_hardware"

# Remove redundant camera/sensor packages  
remove_package "camera_ws" "robot_hardware"
remove_package "robot_sensors" "robot_hardware"

# Remove redundant vision package
remove_package "vision_system" "robot_perception"

echo ""
echo "=== CLEANUP SUMMARY ==="

echo "✅ Remaining packages:"
ls src/ | grep -E "^robot_|^src" | sort

echo ""
echo "📦 Backup packages created in: backup_packages/"
if [ -d "backup_packages" ]; then
    ls backup_packages/
fi

echo ""
echo "🎯 Next steps:"
echo "   1. Build the workspace: ./build_ros2_pi.sh"
echo "   2. Test functionality: ros2 launch robot_bringup bringup.launch.py"
echo "   3. If everything works, you can delete backup_packages/"
echo ""
echo "⚠️  If you need to restore a package:"
echo "   cp -r backup_packages/PACKAGE_NAME src/"