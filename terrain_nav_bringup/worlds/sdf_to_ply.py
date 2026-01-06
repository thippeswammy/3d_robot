import xml.etree.ElementTree as ET
import trimesh
import numpy as np
from scipy.spatial.transform import Rotation as R

def parse_pose(pose_str):
    """Converts Gazebo pose string 'x y z roll pitch yaw' to a 4x4 matrix."""
    vals = [float(x) for x in pose_str.split()]
    translation = vals[:3]
    rotation = vals[3:]
    
    mat = np.eye(4)
    # Gazebo uses intrinsic Euler angles (static XYZ or active ZYX)
    r = R.from_euler('xyz', rotation).as_matrix()
    mat[:3, :3] = r
    mat[:3, 3] = translation
    return mat

def create_mesh_from_sdf(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    world = root.find('world')
    
    meshes = []

    for model in world.findall('model'):
        model_name = model.get('name')
        # Skip the infinite ground plane visual if desired
        # if model_name == 'ground_plane':
            # continue

        # Get Model Pose [cite: 8, 13, 26, 40]
        model_pose_str = model.find('pose').text if model.find('pose') is not None else "0 0 0 0 0 0"
        model_matrix = parse_pose(model_pose_str)

        for link in model.findall('link'):
            visual = link.find('visual')
            if visual is None: continue
            
            geom = visual.find('geometry')
            mesh_obj = None

            # Handle Box Geometry [cite: 9, 14, 28, 41]
            box = geom.find('box')
            if box is not None:
                size = [float(x) for x in box.find('size').text.split()]
                mesh_obj = trimesh.creation.box(extents=size)

            # Handle Sphere Geometry [cite: 46, 56, 67]
            sphere = geom.find('sphere')
            if sphere is not None:
                radius = float(sphere.find('radius').text)
                mesh_obj = trimesh.creation.uv_sphere(radius=radius)

            # Handle Cylinder Geometry [cite: 51, 77]
            cylinder = geom.find('cylinder')
            if cylinder is not None:
                radius = float(cylinder.find('radius').text)
                length = float(cylinder.find('length').text)
                mesh_obj = trimesh.creation.cylinder(radius=radius, height=length)

            if mesh_obj:
                # Apply pose relative to the world
                mesh_obj.apply_transform(model_matrix)
                meshes.append(mesh_obj)

    # Combine all individual meshes into one
    combined_mesh = trimesh.util.concatenate(meshes)
    return combined_mesh

# Execute conversion
full_mesh = create_mesh_from_sdf('uneven_terrain.world')
full_mesh.export('uneven_terrain.ply')
print("Mesh exported successfully as uneven_terrain.ply")