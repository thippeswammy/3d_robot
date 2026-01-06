# Mesh Navigation Ecosystem Overview

This document explains the components that make up the `mesh_navigation` stack.

## 1. mesh_navigation
**What it is**: The core ROS 2 package for 3D navigation.
**Role**:
-   **Mesh Map**: Instead of a 2D occupancy grid, it uses a 3D Triangle Mesh (from `lvr2`) to represent the world.
-   **Layered Costmap**: It projects costs onto this mesh.
    -   *Steepness Layer*: Calculates slope; too steep = lethal obstacle.
    -   *Roughness Layer*: uneven terrain = higher cost.
    -   *Height Diff Layer*: Steps/cliffs = lethal.
-   **Planners**: Provides path planning specifically for meshes (e.g., Vector Field Planner).

## 2. move_base_flex (MBF)
**What it is**: A highly flexible replacement for the standard `move_base` / `nav2`.
**Role**:
-   Acts as the **Execution Server**. It coordinates:
    -   **Planners**: Calculating the path.
    -   **Controllers**: Driving the robot along the path.
    -   **Recovery Behaviors**: Unstucking the robot.
-   `mesh_navigation` implements plugins for MBF. So, MBF asks `mesh_navigation` "Give me a path on this mesh", and `mesh_navigation` computes it.

## 3. lvr2 (Las Vegas Reconstruction 2)
**What it is**: A standalone C++ library (not just ROS) for 3D processing.
**Role**:
-   **The Engine Room**: It handles the heavy math.
-   **Reconstruction**: Converts Point Clouds -> Meshes (`lvr2_reconstruct`).
-   **IO**: Efficiently reads/writes the HDF5 (`.h5`) map files (`lvr2_hdf5_mesh_tool`).
-   `mesh_navigation` links against `lvr2` to load and query the map efficiently in real-time.

## 4. mesh_tools
**What it is**: Helper utilities and visualizations.
**Role**:
-   **RViz Plugins**: Adds "Mesh Display" types to RViz so you can see the 3D terrain and cost layers colored on the mesh.
-   **Tools**: Provides the `rviz_mesh_tools_plugins` which are essential for visualizing what the robot "thinks" the world looks like.

## 5. mesh_navigation_tutorials
**What it is**: Example configurations.
**Role**:
-   Provides "gold standard" examples of how to configure the stack (YAML files, launch files).
-   Use this to check "How should I set the steepness threshold?" or "How do I structure my launch file?".
