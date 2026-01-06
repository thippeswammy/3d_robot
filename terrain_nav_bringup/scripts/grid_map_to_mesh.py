#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from std_srvs.srv import Empty, Trigger
import numpy as np
import h5py
import os

class GridMapToMesh(Node):
    def __init__(self):
        super().__init__('grid_map_to_mesh')
        
        self.declare_parameter('grid_map_topic', '/elevation_mapping/map')
        self.declare_parameter('output_file', 'map.h5')
        self.declare_parameter('elevation_layer', 'elevation')
        
        self.map_topic = self.get_parameter('grid_map_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.layer = self.get_parameter('elevation_layer').value
        
        self.latest_map = None
        
        self.subscription = self.create_subscription(
            GridMap,
            self.map_topic,
            self.map_callback,
            1)
            
        self.srv = self.create_service(Trigger, 'save_mesh', self.save_mesh_callback)
        self.get_logger().info(f'GridMapToMesh node started. Listening on {self.map_topic}. Call /save_mesh to save to {self.output_file}')

    def map_callback(self, msg):
        self.latest_map = msg

    def save_mesh_callback(self, request, response):
        if self.latest_map is None:
            response.success = False
            response.message = "No GridMap received yet."
            return response
            
        try:
            self.convert_and_save(self.latest_map, self.output_file)
            response.success = True
            response.message = f"Mesh saved to {self.output_file}"
        except Exception as e:
            self.get_logger().error(f"Conversion failed: {str(e)}")
            response.success = False
            response.message = f"Failed: {str(e)}"
            
        return response

    def convert_and_save(self, grid_map, filename):
        # 1. Extract Data
        layers = grid_map.layers
        if self.layer not in layers:
            raise ValueError(f"Layer {self.layer} not found in GridMap.")
            
        idx = layers.index(self.layer)
        # Data is in row-major order? GridMultiArray
        # layout info
        
        # GridMap msg: data is MultiArray.
        # layout.dim[0] is usually x (rows? cols?), dim[1] is y.
        # stride is used to access.
        
        # Parse MultiArray
        # Assuming row-major, size X * Y
        
        dim_x = grid_map.data[idx].layout.dim[0].size  # Outer 
        dim_y = grid_map.data[idx].layout.dim[1].size  # Inner
        
        data = np.array(grid_map.data[idx].data, dtype=np.float32)
        
        # Reshape
        # Standard GridMap ros message: data[0] is (x0, y0), data[1] is (x0, y1)...?
        # Actually usually it's (X, Y)
        # Let's try reshaping to (dim_x, dim_y)
        
        # IMPORTANT: GridMap internal storage can be circular buffer, but the message is usually linear?
        # "The data is stored in row-major order, starting from (0,0)."
        
        grid = data.reshape((dim_x, dim_y))
        
        resolution = grid_map.info.resolution
        length_x = grid_map.info.length_x
        length_y = grid_map.info.length_y
        pose_x = grid_map.info.pose.position.x
        pose_y = grid_map.info.pose.position.y
        
        # Calculate origin (top-left or bottom-left?)
        # GridMap usually center-aligned around pose.
        # Top-left corner:
        start_x = pose_x + length_x / 2.0
        start_y = pose_y + length_y / 2.0
        
        # Vertices
        vertices = []
        vertex_map = {} # (i, j) -> vertex_index
        
        # Generate vertices
        # We assume grid[i, j] corresponds to position.
        # Actually need to check GridMap coordinate convention closely.
        # Usually index (0,0) is top-left.
        # x_i = start_x - i * res
        # y_j = start_y - j * res
        # OR rotated.
        
        # Simplified assumption: aligned with world.
        
        cnt = 0
        for i in range(dim_x):
            for j in range(dim_y):
                z = grid[i, j]
                if np.isnan(z):
                    continue
                    
                # Compute x, y
                # GridMap msg definition:
                # "The data is stored in row-major order, starting from the cell with index (0, 0)."
                # index (i,j) -> position
                # Position = Pose - Length/2 + (Index + 0.5) * Res ??
                
                # Standard conversion:
                # position = origin + index * resolution
                # But GridMap origin is typically center.
                
                # Let's simple-case it:
                # Top-left (max X, max Y) is index (0,0)?
                # GridMap documentation: "Index (0,0) corresponds to the top-left corner of the grid."
                # Top-left usually means +X, +Y in map frame? Or generally?
                # Usually:
                # x = center_x + (length_x / 2) - (i * resolution) - (resolution / 2)
                # y = center_y + (length_y / 2) - (j * resolution) - (resolution / 2)
                
                x = pose_x + (length_x / 2.0) - (i * resolution) - (resolution / 2.0)
                y = pose_y + (length_y / 2.0) - (j * resolution) - (resolution / 2.0)
                
                vertices.append([x, y, z])
                vertex_map[(i, j)] = cnt
                cnt += 1
                
        vertices = np.array(vertices, dtype=np.float32)
        
        faces = []
        
        # Generate faces
        for i in range(dim_x - 1):
            for j in range(dim_y - 1):
                # Quad (i,j) - (i+1, j+1)
                # v00 -- v01 (i, j+1)
                #  |      |
                # v10 -- v11
                
                # Check if all relevant vertices exist (not NaN)
                p00 = (i, j)
                p01 = (i, j+1)
                p10 = (i+1, j)
                p11 = (i+1, j+1)
                
                if p00 in vertex_map and p10 in vertex_map and p01 in vertex_map:
                    faces.append([vertex_map[p00], vertex_map[p10], vertex_map[p01]])
                    
                if p10 in vertex_map and p11 in vertex_map and p01 in vertex_map:
                    faces.append([vertex_map[p10], vertex_map[p11], vertex_map[p01]])
                    
        faces = np.array(faces, dtype=np.uint32)
        
        # Save to HDF5 (LVR2 format)
        with h5py.File(filename, 'w') as f:
            grp = f.create_group("mesh")
            geo = grp.create_group("geometry")
            geo.create_dataset("vertices", data=vertices)
            geo.create_dataset("face_indices", data=faces)
            
            # Optional attributes
            attrs = grp.create_group("vertex_attributes")
            # We could compute roughness etc here or just leave empty
            
        self.get_logger().info(f"Saved {len(vertices)} vertices and {len(faces)} faces to {filename}")

        # Save to PLY
        ply_file = filename.replace('.h5', '.ply')
        self.save_ply(vertices, faces, ply_file)
        
        # Save to PCD
        pcd_file = filename.replace('.h5', '.pcd')
        self.save_pcd(vertices, pcd_file)

    def save_ply(self, vertices, faces, filename):
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(vertices)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write(f"element face {len(faces)}\n")
            f.write("property list uchar int vertex_indices\n")
            f.write("end_header\n")
            
            for v in vertices:
                f.write(f"{v[0]} {v[1]} {v[2]}\n")
            
            for face in faces:
                f.write(f"3 {face[0]} {face[1]} {face[2]}\n")
        
        self.get_logger().info(f"Saved PLY to {filename}")

    def save_pcd(self, vertices, filename):
        with open(filename, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(vertices)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(vertices)}\n")
            f.write("DATA ascii\n")
            
            for v in vertices:
                f.write(f"{v[0]} {v[1]} {v[2]}\n")
                
        self.get_logger().info(f"Saved PCD to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = GridMapToMesh()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
