# Build and Fix Script

This is a consolidated script that handles all building and fixing tasks for the ROS 2 workspace.

## Features

- **Unified Build System**: Single script for all build and fix operations
- **Environment Detection**: Automatically detects Docker and Raspberry Pi environments
- **Comprehensive Fixes**: Handles common issues including:
  - Permission fixes
  - Missing directories
  - Python package issues
  - CMake cache problems
  - ROS dependency resolution
- **Clean Builds**: Supports clean builds with `--clean` flag
- **CI/CD Friendly**: Works well in automated environments

## Usage

### Basic Usage
```bash
./build_and_fix.sh
```

### Options
- `--docker`: Force Docker mode (auto-detected if not specified)
- `--pi`: Force Raspberry Pi mode (auto-detected if not specified)
- `--clean`: Perform a clean build

### Examples

**Standard build:**
```bash
./build_and_fix.sh
```

**Clean build:**
```bash
./build_and_fix.sh --clean
```

**Explicit Docker build:**
```bash
./build_and_fix.sh --docker
```

**Explicit Raspberry Pi build:**
```bash
./build_and_fix.sh --pi
```

## Cleanup

All old build and fix scripts have been consolidated into this single script. The following scripts have been removed:

- `build_ros2.sh`
- `create_missing_dirs.sh`
- `create_nav_configs.sh`
- `fix_cmake.sh`
- `fix_python_pkgs.sh`
- `fix_workspace.sh`

Test scripts (`test_*.sh`) have been kept as they serve a different purpose.

## Requirements

- Bash 4.0 or higher
- ROS 2 Humble (or compatible)
- Python 3.8+
- pip
- colcon
- rosdep
