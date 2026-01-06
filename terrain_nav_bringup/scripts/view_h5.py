#!/usr/bin/env python3

import sys
import h5py
import numpy as np
import os

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 view_h5.py <path_to_h5_file>")
        sys.exit(1)

    h5_path = sys.argv[1]
    if not os.path.exists(h5_path):
        print(f"Error: File '{h5_path}' not found.")
        sys.exit(1)

    print(f"Opening {h5_path}...")
    
    try:
        with h5py.File(h5_path, 'r') as f:
            # Check structure based on grid_map_to_mesh.py
            if 'mesh' not in f:
                print("Error: 'mesh' group not found in HDF5 file.")
                print(f"Groups present: {list(f.keys())}")
                sys.exit(1)
            
            mesh_group = f['mesh']
            if 'geometry' not in mesh_group:
                print("Error: 'geometry' group not found in 'mesh'.")
                sys.exit(1)
                
            geo_group = mesh_group['geometry']
            if 'vertices' not in geo_group or 'face_indices' not in geo_group:
                print("Error: 'vertices' or 'face_indices' not found in 'geometry'.")
                print(f"Datasets present: {list(geo_group.keys())}")
                sys.exit(1)
                
            vertices = np.array(geo_group['vertices'])
            faces = np.array(geo_group['face_indices'])
            
            print(f"Loaded {len(vertices)} vertices and {len(faces)} faces.")
            
            try:
                import open3d as o3d
                
                print("Visualizing with Open3D...")
                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector(vertices)
                mesh.triangles = o3d.utility.Vector3iVector(faces)
                mesh.compute_vertex_normals()
                
                # Create a coordinate frame for reference
                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
                
                o3d.visualization.draw_geometries([mesh, coord_frame], 
                                                window_name=f"H5 Mesh Viewer: {os.path.basename(h5_path)}",
                                                width=1024, height=768)
            except ImportError:
                print("Warning: open3d not installed. Cannot visualize 3D mesh.")
                print("You can install it with: pip install open3d")
                print("\nMesh statistics:")
                print(f"Vertices shape: {vertices.shape}")
                print(f"Faces shape: {faces.shape}")
                print(f"X range: [{vertices[:,0].min()}, {vertices[:,0].max()}]")
                print(f"Y range: [{vertices[:,1].min()}, {vertices[:,1].max()}]")
                print(f"Z range: [{vertices[:,2].min()}, {vertices[:,2].max()}]")

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
