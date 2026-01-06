import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Output path
    mesh_file = os.path.join(pkg_dir, 'map', 'map.h5')
    
    # Grid Map to Mesh Node
    # Assuming 'grid_map_to_mesh' executable exists in 'mesh_map' or 'mesh_tools' package
    # Usage typically: input topic (grid_map), output file.
    
    grid_map_to_mesh = Node(
        package='mesh_map', # or mesh_tools?
        executable='grid_map_to_mesh_node', 
        name='grid_map_to_mesh',
        output='screen',
        remappings=[
            ('grid_map', '/elevation_map')
        ],
        parameters=[
            {'mesh_file': mesh_file},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        grid_map_to_mesh
    ])
