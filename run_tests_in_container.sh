#!/bin/bash

# Exit on error
set -e

# Default values
CONTAINER_NAME="Dojo_Env"
TEST_DURATION=10  # seconds

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--container)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        -d|--duration)
            TEST_DURATION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -c, --container NAME  Docker container name (default: Dojo_Env)"
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

# Check if container exists
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' not found"
    exit 1
fi

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting container '${CONTAINER_NAME}'..."
    docker start ${CONTAINER_NAME}
    sleep 2  # Give it time to start
fi

# Run tests in the container
echo "Running tests in container '${CONTAINER_NAME}' for ${TEST_DURATION} seconds..."

docker exec -it ${CONTAINER_NAME} /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /home/Dojo/Dojo/install/setup.bash && \
    cd /home/Dojo/Dojo && \
    python3 test_arduino_bridge.py"

echo "Tests completed successfully!"
exit 0
