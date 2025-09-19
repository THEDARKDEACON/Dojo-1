#!/bin/bash

echo "🔧 Fixing robot_perception symlinks..."

# Create the robot_perception directory in install/lib if it doesn't exist
mkdir -p install/lib/robot_perception

# Create symlinks for the executables
if [ -f "src/robot_perception/robot_perception/object_detector.py" ]; then
    echo "Creating symlink for object_detector..."
    ln -sf "$(pwd)/src/robot_perception/robot_perception/object_detector.py" install/lib/robot_perception/object_detector
    chmod +x install/lib/robot_perception/object_detector
    echo "✅ object_detector symlink created"
else
    echo "❌ object_detector.py not found"
fi

# List the created symlinks
echo "📋 Current robot_perception executables:"
ls -la install/lib/robot_perception/ 2>/dev/null || echo "Directory not found"

echo "🎯 robot_perception symlinks fixed!"