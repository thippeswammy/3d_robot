import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Defaults
    default_pcd = os.path.join(pkg_dir, 'maps', 'map.pcd')

    # Arguments
    pcd_file = LaunchConfiguration('pcd_file', default=default_pcd)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Node
    pcd_to_gridmap = Node(
        package=pkg_name,
        executable='pcd_to_gridmap_node',
        name='pcd_to_gridmap',
        parameters=[{
            'pcd_filename': pcd_file,
            'resolution': 0.1,
            'map_frame_id': 'map',
            'max_slope_deg': 25.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # RViz (verify processing)
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_mapping.rviz') # Re-use mapping rviz for check
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('pcd_file', default_value=default_pcd, description='Path to PCD file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        pcd_to_gridmap,
        rviz
    ])
