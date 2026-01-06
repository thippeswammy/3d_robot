import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)

    # Paths
    rviz_config = os.path.join(pkg_dir, 'config', 'grid_map_view.rviz')
    default_pcd = os.path.join(pkg_dir, 'maps', 'map.pcd')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pcd_path = LaunchConfiguration('pcd_path', default=default_pcd)

    # Map Processing Node
    pcd_to_gridmap = Node(
        package=pkg_name,
        executable='pcd_to_gridmap_node',
        name='pcd_to_gridmap_node',
        parameters=[{
            'pcd_filename': pcd_path,
            'resolution': 0.1,
            'map_frame_id': 'map',
            'max_slope_deg': 25.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Static TF for visualization (map -> base_link at origin if needed)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pcd_path', default_value=default_pcd),
        pcd_to_gridmap,
        static_tf,
        rviz
    ])
