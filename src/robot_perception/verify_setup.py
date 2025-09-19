#!/usr/bin/env python3
"""
Verification script for robot_perception package setup.py configuration.

This script verifies that the setup.py is properly configured for ROS2 installation
without the complex symlink creation logic that was causing issues.

Run this after building the package to verify proper installation.
"""

import os
import sys
import importlib.util

def check_package_structure():
    """Check if the package has the correct structure."""
    print("Checking package structure...")
    
    required_files = [
        'setup.py',
        'package.xml',
        'resource/robot_perception',
        'robot_perception/__init__.py',
        'robot_perception/camera_processor.py',
        'robot_perception/object_detector.py'
    ]
    
    missing_files = []
    for file_path in required_files:
        if not os.path.exists(file_path):
            missing_files.append(file_path)
    
    if missing_files:
        print(f"❌ Missing files: {missing_files}")
        return False
    else:
        print("✅ All required files present")
        return True

def check_entry_points():
    """Check if the entry points are properly configured."""
    print("Checking entry points configuration...")
    
    try:
        # Check if modules can be imported
        spec = importlib.util.spec_from_file_location(
            "robot_perception.camera_processor", 
            "robot_perception/camera_processor.py"
        )
        camera_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(camera_module)
        
        spec = importlib.util.spec_from_file_location(
            "robot_perception.object_detector", 
            "robot_perception/object_detector.py"
        )
        detector_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(detector_module)
        
        # Check if main functions exist
        if hasattr(camera_module, 'main') and hasattr(detector_module, 'main'):
            print("✅ Entry point functions found")
            return True
        else:
            print("❌ Main functions not found in modules")
            return False
            
    except Exception as e:
        print(f"❌ Error importing modules: {e}")
        return False

def check_setup_py():
    """Check if setup.py has the correct configuration."""
    print("Checking setup.py configuration...")
    
    try:
        with open('setup.py', 'r') as f:
            content = f.read()
        
        # Check for removed symlink logic
        if 'create_symlinks' in content or 'PostDevelopCommand' in content:
            print("❌ Custom symlink logic still present")
            return False
        
        # Check for proper entry points
        if 'console_scripts' in content and 'camera_processor' in content and 'object_detector' in content:
            print("✅ Entry points properly configured")
            return True
        else:
            print("❌ Entry points not properly configured")
            return False
            
    except Exception as e:
        print(f"❌ Error reading setup.py: {e}")
        return False

def main():
    """Main verification function."""
    print("Robot Perception Package Setup Verification")
    print("=" * 50)
    
    # Change to package directory
    package_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(package_dir)
    
    checks = [
        check_package_structure,
        check_setup_py,
        check_entry_points
    ]
    
    all_passed = True
    for check in checks:
        if not check():
            all_passed = False
        print()
    
    if all_passed:
        print("🎉 All verification checks passed!")
        print("\nNext steps:")
        print("1. Build the package: colcon build --packages-select robot_perception")
        print("2. Source the workspace: source install/setup.bash")
        print("3. Test executables: ros2 run robot_perception camera_processor")
        print("4. Test executables: ros2 run robot_perception object_detector")
    else:
        print("❌ Some verification checks failed. Please review the issues above.")
        sys.exit(1)

if __name__ == '__main__':
    main()