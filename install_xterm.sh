#!/bin/bash

# Quick script to install xterm in Docker container

echo "🔧 Installing xterm for teleop support..."

# Update package list and install xterm
apt-get update && apt-get install -y xterm

echo "✅ xterm installed successfully!"
echo ""
echo "Now you can run the full simulation:"
echo "  ./run_full_simulation.sh"