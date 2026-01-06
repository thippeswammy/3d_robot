import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    lidarslam_config = os.path.join(pkg_dir, 'config', 'lidarslam_3d.yaml')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Localization (Lidarslam)
    scan_matcher = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scan_matcher',
        parameters=[lidarslam_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('input_cloud', '/input_cloud'),
            ('imu', '/imu'),
            ('odom', '/odom')
        ],
        output='screen'
    )

    graph_based_slam = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        parameters=[lidarslam_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Use a local rviz config for mapping visualization
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        scan_matcher,
        graph_based_slam,
        rviz
    ])
