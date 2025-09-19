# Design Document

## Overview

The robot_perception package installation issue stems from a mismatch between how ROS2 expects Python executables to be installed and how the current setup.py is configured. The package builds successfully but the executables are not placed in the correct directory structure that ROS2 launch files expect.

## Architecture

The fix involves three main components:

1. **Package Structure Correction**: Ensure the setup.py properly installs executables in the expected ROS2 directory structure
2. **Installation Verification**: Add checks to verify that executables are properly installed
3. **Launch File Integration**: Ensure the launch files can find and execute the perception nodes

## Components and Interfaces

### 1. Setup.py Configuration
- **Purpose**: Configure proper installation of Python executables
- **Key Changes**:
  - Fix entry_points configuration
  - Ensure executables are installed in lib/robot_perception/
  - Remove complex symlink creation that may be causing issues
  - Use standard ROS2 Python package structure

### 2. Package.xml Dependencies
- **Purpose**: Ensure all required dependencies are declared
- **Dependencies to verify**:
  - rclpy
  - sensor_msgs
  - std_msgs
  - geometry_msgs
  - cv_bridge
  - opencv-python

### 3. Installation Verification Script
- **Purpose**: Verify that the package installs correctly
- **Functions**:
  - Check if executables exist in expected locations
  - Verify entry points are working
  - Test basic node functionality

### 4. Launch File Updates
- **Purpose**: Ensure launch files can properly start perception nodes
- **Approach**:
  - Use standard ROS2 node launching
  - Add proper error handling
  - Include fallback options if perception fails

## Data Models

### Installation Structure
```
install/robot_perception/
├── lib/
│   └── robot_perception/
│       ├── camera_processor
│       └── object_detector
├── share/
│   └── robot_perception/
│       ├── launch/
│       └── config/
└── bin/  (optional, for direct execution)
```

### Node Communication
```
Camera Topics:
- Input: /image_raw (sensor_msgs/Image)
- Input: /camera_info (sensor_msgs/CameraInfo)
- Output: /perception/image_processed (sensor_msgs/Image)
- Output: /perception/detections (sensor_msgs/Image)
- Output: /perception/detection_info (std_msgs/String)
- Output: /perception/object_count (geometry_msgs/Point)
```

## Error Handling

### Installation Errors
- **Missing Dependencies**: Clear error messages with installation instructions
- **Permission Issues**: Guidance on proper build environment setup
- **Path Issues**: Automatic path correction where possible

### Runtime Errors
- **Camera Connection**: Graceful handling of missing camera topics
- **Processing Errors**: Continue operation even if individual frames fail
- **Resource Constraints**: Adaptive processing based on system capabilities

## Testing Strategy

### Unit Tests
- Test individual perception algorithms
- Verify node initialization and shutdown
- Test topic publishing and subscribing

### Integration Tests
- Test with simulated camera data
- Verify integration with complete simulation
- Test launch file execution

### Installation Tests
- Verify package builds correctly
- Check executable installation
- Test ros2 run commands

### System Tests
- Full simulation with perception enabled
- Performance testing under load
- Error recovery testing

## Implementation Approach

### Phase 1: Fix Setup.py
1. Simplify setup.py configuration
2. Remove custom symlink creation
3. Use standard ROS2 Python package structure
4. Test basic installation

### Phase 2: Verify Dependencies
1. Check package.xml dependencies
2. Ensure all Python dependencies are available
3. Test import statements in nodes

### Phase 3: Test Installation
1. Create verification script
2. Test ros2 run commands
3. Verify launch file integration

### Phase 4: Integration Testing
1. Test with complete simulation
2. Verify all topics are working
3. Test error handling scenarios

## Design Decisions

### Use Standard ROS2 Structure
**Decision**: Remove custom symlink creation and use standard ROS2 Python package installation
**Rationale**: Custom installation logic is error-prone and not necessary for standard ROS2 packages
**Alternative**: Keep custom logic but fix the bugs
**Chosen**: Standard approach for better maintainability

### Graceful Degradation
**Decision**: Allow simulation to continue even if perception fails
**Rationale**: Perception is an optional component for basic simulation functionality
**Implementation**: Use conditional launching and proper error handling

### Modular Design
**Decision**: Keep camera_processor and object_detector as separate nodes
**Rationale**: Allows for independent testing and development of each component
**Benefits**: Better debugging, easier maintenance, flexible deployment