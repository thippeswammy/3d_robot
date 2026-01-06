# Mesh Navigation Pipeline Guide

This guide provides a step-by-step workflow to go from a Gazebo simulation to a fully navigable Mesh Map.

## Prerequisites
Ensure the workspace is built and sourced:
```bash
cd /media/thippe/SDV/Ubuntu/github_testing/mesh_navigation
colcon build --symlink-install --packages-select 3D_vehicle_navigation
source install/setup.bash
```

## 1. Launch Simulation
Launch your custom world (`basic_ramp.world`) with the robot. This starts Gazebo, the robot state publisher, and the bridge.

```bash
ros2 launch 3D_vehicle_navigation mapping_3d.launch.py world_name:=basic_ramp.world
```
*Wait for Gazebo and RViz to open.*

## 2. Scan the Environment (Data Collection)
The goal is to cover the entire environment with the LiDAR to generate a dense point cloud.

1.  **Open a new terminal** and run the teleop node:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
2.  **Drive the robot**: Move slowly over ramps and obstacles.
3.  **Visualize**: In RViz, ensure the `/input_cloud` (or `/map_cloud` from SLAM) covers all surfaces. Gaps in the point cloud will become holes in the mesh.

## 3. Save Point Cloud (PCD)
When satisfied with the scan, save the map using the `lidarslam_ros2` service.

1.  **Run the save service**:
    ```bash
    ros2 service call /map_save std_srvs/srv/Empty {}
    ```
    *Output*: This saves a file named `map.pcd` in the directory where you launched `mapping_3d.launch.py`.

2.  **Move and Rename**:
    ```bash
    mkdir -p src/3D_vehicle_navigation/maps
    mv map.pcd src/3D_vehicle_navigation/maps/ramp_raw.pcd
    ```

## 4. Convert PCD to Mesh Map (Critical Step)
This process converts the raw Point Cloud (`.pcd`) into the Navigable Mesh Map (`.h5`) required by `mesh_navigation`.

### Step 4a: Reconstruction (PCD -> PLY)
Use `lvr2_reconstruct` to create a surface mesh.

*   **Command**:
    ```bash
    install/lvr2/bin/lvr2_reconstruct -v 0.1 -i src/3D_vehicle_navigation/maps/ramp_raw.pcd -o ramp_mesh.ply
    ```
*   **Flags Explanation**:
    *   `-v 0.1`: **Voxel Size**. This sets the resolution to 10cm.
        *   *Decrease* (e.g., `0.05`) for higher detail (smoother ramps).
        *   *Increase* (e.g., `0.2`) for faster processing/lower detail.
    *   `-i`: Input file path (`.pcd`).
    *   `-o`: Output file name (`.ply`).

### Step 4b: Map Conversion (PLY -> H5)
Use `lvr2_hdf5_mesh_tool` to convert the standard mesh (`.ply`) into the HDF5 map format (`.h5`) and compute cost layers.

*   **Command**:
    ```bash
    install/lvr2/bin/lvr2_hdf5_mesh_tool -i ramp_mesh.ply -o src/3D_vehicle_navigation/maps/ramp_map.h5
    ```
*   **Flags Explanation**:
    *   `-i`: Input mesh (`.ply`).
    *   `-o`: Output map (`.h5`).
    *   *Optional*: `-r 0.3`: Sets the local radius for roughness calculation (default is usually fine).

## 5. Configure Navigation
Update the navigation config file to point to your new map.

1.  **Edit File**: `src/3D_vehicle_navigation/config/mesh_map.yaml` (or custom config).
2.  **Modify Path**:
    ```yaml
    mesh_map:
      ros__parameters:
        mesh_map:
          mesh_file: "/media/thippe/SDV/Ubuntu/github_testing/mesh_navigation/src/3D_vehicle_navigation/maps/ramp_map.h5"
          # ... existing layer configuration ...
    ```

## 6. Launch Navigation
Launch the navigation stack with the new map.

```bash
ros2 launch 3D_vehicle_navigation navigation_3d.launch.py
```

## 7. Verify Navigation
1.  **Initial Pose**:
    *   In RViz, select the **"2D Pose Estimate"** tool.
    *   Click and drag on the map surface where the robot is located in Gazebo to initialize localization.
2.  **Send Goal**:
    *   Select the **"Nav2 Goal"** tool (or "Mesh Goal" via `rviz_mesh_tools_plugins`).
    *   Click on a target location (e.g., top of the ramp).
3.  **Result**:
    *   `move_base_flex` should compute a path.
    *   The robot should drive up the ramp following the mesh surface.
