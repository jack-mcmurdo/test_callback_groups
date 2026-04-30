#!/bin/bash

# Build script for ROS2 workspace
# Runs colcon build with merge-install and symlink-install flags

echo "Building ROS2 workspace..."
cd ~/ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Run colcon build
colcon build --merge-install --symlink-install

if [ $? -eq 0 ]; then
    echo "Build completed successfully!"
    echo "Don't forget to source the install setup: source ~/ws/install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi