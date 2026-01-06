import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Configs
    mbf_config = os.path.join(pkg_dir, 'config', 'mbf_mesh.yaml')
    mesh_file = os.path.join(pkg_dir, 'map', 'map.h5')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Move Base Flex (Mesh Navigation)
    mbf_mesh_nav = Node(
        package='mbf_mesh_nav',
        executable='mbf_mesh_nav',
        name='move_base_flex',
        output='screen',
        parameters=[
            mbf_config,
            {
                'mesh_map.mesh_file': mesh_file,
                'use_sim_time': use_sim_time
            }
        ],
        remappings=[
            ('/move_base_flex/cmd_vel', '/cmd_vel'),
            ('/move_base_flex/odom', '/odom'),
        ]
    )
    
    # Goal Bridge (Simple Action Client to test)
    # goal_bridge = Node(...)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        mbf_mesh_nav
    ])
