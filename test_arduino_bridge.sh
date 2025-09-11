#!/bin/bash

# Test script for arduino_bridge

# Exit on error
set -e

# Default values
PORT="/dev/ttyACM0"
BAUD_RATE=115200
TEST_DURATION=10  # seconds

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        -b|--baud)
            BAUD_RATE="$2"
            shift 2
            ;;
        -d|--duration)
            TEST_DURATION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -p, --port PORT       Serial port (default: /dev/ttyACM0)"
            echo "  -b, --baud BAUD_RATE  Baud rate (default: 115200)"
            echo "  -d, --duration SEC    Test duration in seconds (default: 10)"
            echo "  -h, --help            Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/Dojo/Dojo/install/setup.bash

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "Error: Port $PORT does not exist"
    exit 1
fi

# Function to run tests
run_tests() {
    echo "=== Starting Arduino Bridge Tests ==="
    echo "Port: $PORT"
    echo "Baud Rate: $BAUD_RATE"
    echo "Test Duration: $TEST_DURATION seconds"
    echo ""
    
    # Test 1: Check serial communication
    echo "[TEST 1/3] Testing serial communication..."
    if timeout 5s stty -F $PORT $BAUD_RATE -echo -icrnl -onlcr; then
        echo "✓ Serial port $PORT is accessible"
    else
        echo "✗ Failed to access serial port $PORT"
        exit 1
    fi
    
    # Test 2: Launch arduino_bridge node
    echo -e "\n[TEST 2/3] Launching arduino_bridge node..."
    ros2 launch ros2arduino_bridge arduino_bridge.launch.py \
        port:=$PORT \
        baud_rate:=$BAUD_RATE \
        --no-daemon &
    
    # Give it some time to initialize
    sleep 5
    
    # Check if node is running
    if ros2 node list | grep -q 'arduino_bridge'; then
        echo "✓ arduino_bridge node is running"
    else
        echo "✗ Failed to start arduino_bridge node"
        exit 1
    fi
    
    # Test 3: Run test node
    echo -e "\n[TEST 3/3] Running test sequence for $TEST_DURATION seconds..."
    ros2 launch ros2arduino_bridge test_arduino_bridge.launch.py \
        test_duration:=$TEST_DURATION \
        --no-daemon
    
    echo -e "\n=== All tests completed successfully! ==="
}

# Run tests and handle errors
if ! run_tests; then
    echo "\n=== Test Failed! ==="
    # Clean up any remaining processes
    pkill -f "ros2 launch" || true
    exit 1
fi

exit 0
