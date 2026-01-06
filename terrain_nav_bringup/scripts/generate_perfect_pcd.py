import numpy as np
from lxml import etree
import sys
import os
from scipy.spatial.transform import Rotation as R

def sample_box(size, density=100):
    dx, dy, dz = size
    points = []
    
    # Top and Bottom faces
    x = np.linspace(-dx/2, dx/2, int(dx*density))
    y = np.linspace(-dy/2, dy/2, int(dy*density))
    xv, yv = np.meshgrid(x, y)
    points.append(np.stack([xv.flatten(), yv.flatten(), np.full_like(xv.flatten(), dz/2)], axis=1)) # Top
    points.append(np.stack([xv.flatten(), yv.flatten(), np.full_like(xv.flatten(), -dz/2)], axis=1)) # Bottom
    
    # Front and Back (XZ plane)
    x = np.linspace(-dx/2, dx/2, int(dx*density))
    z = np.linspace(-dz/2, dz/2, int(dz*density))
    xv, zv = np.meshgrid(x, z)
    points.append(np.stack([xv.flatten(), np.full_like(xv.flatten(), dy/2), zv.flatten()], axis=1)) # Front
    points.append(np.stack([xv.flatten(), np.full_like(xv.flatten(), -dy/2), zv.flatten()], axis=1)) # Back
    
    # Left and Right (YZ plane)
    y = np.linspace(-dy/2, dy/2, int(dy*density))
    z = np.linspace(-dz/2, dz/2, int(dz*density))
    yv, zv = np.meshgrid(y, z)
    points.append(np.stack([np.full_like(yv.flatten(), dx/2), yv.flatten(), zv.flatten()], axis=1)) # Right
    points.append(np.stack([np.full_like(yv.flatten(), -dx/2), yv.flatten(), zv.flatten()], axis=1)) # Left

    return np.concatenate(points, axis=0)

def sample_sphere(radius, density=50):
    # Sample surface of sphere
    points = []
    phi = np.linspace(0, np.pi, int(np.pi * radius * density))
    theta = np.linspace(0, 2 * np.pi, int(2 * np.pi * radius * density))
    for p in phi:
        for t in theta:
            x = radius * np.sin(p) * np.cos(t)
            y = radius * np.sin(p) * np.sin(t)
            z = radius * np.cos(p)
            points.append([x, y, z])
    return np.array(points)

def sample_cylinder(radius, length, density=50):
    # Sample top face and side
    points = []
    # Top face
    r = np.linspace(0, radius, int(radius*density))
    theta = np.linspace(0, 2*np.pi, int(2*np.pi*radius*density))
    for ri in r:
        for ti in theta:
            points.append([ri * np.cos(ti), ri * np.sin(ti), length/2])
    
    # Side
    z = np.linspace(-length/2, length/2, int(length*density))
    for zi in z:
        for ti in theta:
            points.append([radius * np.cos(ti), radius * np.sin(ti), zi])
    return np.array(points)

def parse_pose(pose_str):
    if not pose_str:
        return np.zeros(3), np.zeros(3)
    parts = [float(p) for p in pose_str.split()]
    return np.array(parts[:3]), np.array(parts[3:])

def main(world_file, output_pcd):
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(world_file, parser)
    root = tree.getroot()
    world = root.find("world")
    
    all_points = []
    
    # Define Region of Interest (ROI) [min_x, max_x, min_y, max_y, min_z, max_z]
    # Adjust these values to crop the "outside" area
    ROI = [-6.0, 6.0, -6.0, 6.0, -1.0, 10.0] 

    # Static models
    IGNORE_MODELS = [] # Empty list: Process all models (including ground_plane), but filter by ROI later

    for model in world.findall("model"):
        model_name = model.get("name")
        if model_name in IGNORE_MODELS:
            print(f"Skipping ignored model: {model_name}")
            continue

        static = model.find("static")
        # if static is not None and static.text == "false":
        #    continue # Skip dynamic for now if needed, but here we want everything ground-truth
            
        model_pose_vec, model_euler = parse_pose(model.findtext("pose", "0 0 0 0 0 0"))
        model_rot = R.from_euler('xyz', model_euler).as_matrix()
        
        for link in model.findall("link"):
            link_pose_vec, link_euler = parse_pose(link.findtext("pose", "0 0 0 0 0 0"))
            link_rot = R.from_euler('xyz', link_euler).as_matrix()
            
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                geom_points = None
                
                if geometry.find("box") is not None:
                    size = [float(s) for s in geometry.find("box").findtext("size").split()]
                    geom_points = sample_box(size)
                elif geometry.find("sphere") is not None:
                    radius = float(geometry.find("sphere").findtext("radius"))
                    geom_points = sample_sphere(radius)
                elif geometry.find("cylinder") is not None:
                    radius = float(geometry.find("cylinder").findtext("radius"))
                    length = float(geometry.find("cylinder").findtext("length"))
                    geom_points = sample_cylinder(radius, length)
                elif geometry.find("plane") is not None:
                    size = [float(s) for s in geometry.find("plane").findtext("size").split()]
                    # Sample plane at z=0
                    x = np.linspace(-size[0]/2, size[0]/2, int(size[0]*5))
                    y = np.linspace(-size[1]/2, size[1]/2, int(size[1]*5))
                    xv, yv = np.meshgrid(x, y)
                    geom_points = np.stack([xv.flatten(), yv.flatten(), np.zeros_like(xv.flatten())], axis=1)
                
                if geom_points is not None:
                    # Apply link transform then model transform
                    # Link local
                    geom_points = (link_rot @ geom_points.T).T + link_pose_vec
                    # Model local
                    geom_points = (model_rot @ geom_points.T).T + model_pose_vec
                    
                    # Filter ROI immediately
                    if ROI:
                         mask = (geom_points[:,0] >= ROI[0]) & (geom_points[:,0] <= ROI[1]) & \
                                (geom_points[:,1] >= ROI[2]) & (geom_points[:,1] <= ROI[3]) & \
                                (geom_points[:,2] >= ROI[4]) & (geom_points[:,2] <= ROI[5])
                         geom_points = geom_points[mask]

                    if len(geom_points) > 0:
                        all_points.append(geom_points)

    if not all_points:
        print("No points generated.")
        return

    final_points = np.concatenate(all_points, axis=0)
    print(f"Total points generated: {len(final_points)}")

    if len(final_points) == 0:
        print("No points generated (all filtered out?).")
        return
    
    # Save as PCD
    with open(output_pcd, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH {}\n".format(len(final_points)))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS {}\n".format(len(final_points)))
        f.write("DATA ascii\n")
        for p in final_points:
            f.write("{} {} {}\n".format(p[0], p[1], p[2]))

    # Save as PLY (Alternative for LVR2)
    output_ply = output_pcd.replace(".pcd", ".ply")
    with open(output_ply, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex {}\n".format(len(final_points)))
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in final_points:
            f.write("{} {} {}\n".format(p[0], p[1], p[2]))

    print(f"Generated {output_pcd} and {output_ply}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 generate_perfect_pcd.py <world_file> <output_pcd>")
    else:
        main(sys.argv[1], sys.argv[2])
