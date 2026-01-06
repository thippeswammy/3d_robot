#!/bin/bash

# Default bag name if none provided
BAG_NAME=${1:-"slam_data_$(date +%Y%m%d_%H%M%S)"}

echo "Starting ROS2 bag recording for SLAM..."
echo "Recording to: $BAG_NAME"
echo "Required topics: /input_cloud, /imu, /odom, /tf, /tf_static"

# Record the topics
# Exclude /clock to avoid conflicts during playback with --clock
ros2 bag record \
    /input_cloud \
    /imu \
    /odom \
    /tf \
    /tf_static \
    -o "$BAG_NAME"

echo "Recording stopped."
