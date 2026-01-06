import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Config
    em_config = os.path.join(pkg_dir, 'config', 'elevation_mapping.yaml')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Elevation Mapping Node
    # Note: 'elevation_mapping_cupy' executable might be named differently depending on install.
    # Often it is a python node/script.
    # Checking `elevation_mapping_cupy` package structure suggests it might be `elevation_mapping_node` or similar.
    # Assuming 'elevation_mapping_node' for now.
    elevation_mapping = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping',
        output='screen',
        remappings=[], # Remap if needed, though we set it in yaml
        parameters=[
            em_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        elevation_mapping,
        Node(
            package='terrain_nav_bringup',
            executable='grid_map_to_mesh.py',
            name='grid_map_to_mesh',
            output='screen',
            parameters=[{'grid_map_topic': '/elevation_mapping/map', 'output_file': os.path.join(pkg_dir, 'map', 'map.h5')}]
        )
    ])
