#!/bin/bash

# Script to generate a grid map and occupancy grid from a PCD file
# and save it using the map_saver_cli for persistent use.

# Exit on error
set -e

# Package and Paths
PKG_NAME="3D_vehicle_navigation"
WORKSPACE_DIR="/media/thippe/SDV/Ubuntu/github_testing/mesh_navigation"
MAPS_DIR="${WORKSPACE_DIR}/src/${PKG_NAME}/maps"
# Default to map.pcd if no argument provided
PCD_FILE="${1:-"${MAPS_DIR}/map.pcd"}"
MAP_NAME="map"

echo "Creating Grid Map from ${PCD_FILE}..."

# 1. Source the workspace
source "${WORKSPACE_DIR}/install/setup.bash"

# 2. Run the pcd_to_gridmap_node in the background
# This node publishes /grid_map and /map (OccupancyGrid)
ros2 run "${PKG_NAME}" pcd_to_gridmap_node --ros-args \
    -p pcd_filename:="${PCD_FILE}" \
    -p resolution:=0.1 \
    -p map_frame_id:="map" \
    &

NODE_PID=$!

# 3. Wait for the node to initialize and publish the map
echo "Waiting for map to be published..."
sleep 5

# 4. Save the occupancy grid using Nav2 map_saver
echo "Saving map to ${MAPS_DIR}/${MAP_NAME}..."
ros2 run nav2_map_server map_saver_cli -f "${MAPS_DIR}/${MAP_NAME}" --ros-args -p save_map_timeout:=10.0

# 5. Kill the node
echo "Cleaning up..."
kill $NODE_PID

echo "Grid Map generated and saved successfully!"
echo "Files created:"
ls -l "${MAPS_DIR}/${MAP_NAME}.yaml" "${MAPS_DIR}/${MAP_NAME}.pgm"
