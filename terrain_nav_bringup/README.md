# 3D Vehicle Navigation

This package provides tools for 3D SLAM, terrain analysis, and autonomous navigation for vehicles in uneven environments using ROS 2 and Gazebo.

## Quick Start

### 1. Build the Workspace
```bash
colcon build --symlink-install --packages-select 3D_vehicle_navigation
source install/setup.bash
```

### 2. Mapping Flow (Data Collection)

1.  **Launch Simulation**:
    ```bash
    ros2 launch 3D_vehicle_navigation mapping_3d.launch.py
    ```
2.  **Record SLAM Data**:
    ```bash
    ./src/3D_vehicle_navigation/record_slam.sh my_session
    ```
3.  **Teleoperate**: Use `teleop_twist_keyboard` to cover the environment.

### 3. Map Generation (Post-Processing)

1.  **Run SLAM Playback**:
    ```bash
    ros2 launch 3D_vehicle_navigation mapping_from_bag.launch.py
    ```
2.  **Play Bag**:
    ```bash
    ros2 bag play my_session --clock --remap /clock:=/clock_recorded
    ```
3.  **Save PCD Map**:
    Once mapping is complete in RViz, run:
    ```bash
    ros2 service call /map_save std_srvs/srv/Empty {}
    ```

### 4. Autonomous Navigation

1.  **Install the Map**:
    ```bash
    mkdir -p maps/
    mv map.pcd maps/
    colcon build --symlink-install --packages-select 3D_vehicle_navigation
    ```
2.  **Launch Navigation**:
    ```bash
    ros2 launch 3D_vehicle_navigation navigation_3d.launch.py
    ```
3.  **Set Goal**: Use the **Nav2 Goal** tool in RViz to plan paths over ramps and uneven terrain.

## Package Structure

- `config/`: Param files for Nav2, Lidarslam, and RViz.
- `launch/`: Unified and modular launch files.
- `src/`: Custom nodes (e.g., PCD to Traversability Grid).
- `urdf/`: Vehicle model with sensors.
- `worlds/`: Gazebo terrain environments.

## Prerequisite Topics
The SLAM system requires: `/input_cloud`, `/imu`, `/odom`, `/tf`, and `/tf_static`.
