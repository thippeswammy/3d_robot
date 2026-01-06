import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)

    # Set Ignition Gazebo IP to localhost to avoid multicast issues
    os.environ['IGN_IP'] = '127.0.0.1'
    
    # Paths
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_navigation.rviz') # TODO create this
    lidarslam_config = os.path.join(pkg_dir, 'config', 'lidarslam_3d.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_3d.yaml')
    world_path = os.path.join(pkg_dir, 'worlds', 'uneven_terrain.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'mesh_bot.urdf.xacro')
    default_pcd = os.path.join(pkg_dir, 'maps', 'map.pcd')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_pcd = LaunchConfiguration('map_pcd', default=default_pcd)

    # Process URDF
    doc = xacro.process_file(urdf_path)
    robot_description = doc.toxml()

    # 1. Gazebo & Spawning (re-using logic from mapping_3d, ideally shared)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.5'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_sensors',
        arguments=[
            '/model/my_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/my_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/my_bot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/my_bot/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/model/my_bot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/my_bot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/my_bot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/world/uneven_terrain/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        remappings=[
            ('/model/my_bot/points/points', '/model/my_bot/cloud'),
            ('/model/my_bot/joint_states', '/joint_states'),
            ('/model/my_bot/cmd_vel', '/cmd_vel'),
            ('/model/my_bot/imu', '/imu'),
            ('/world/uneven_terrain/clock', '/clock')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_tf',
        arguments=['/model/my_bot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[('/model/my_bot/tf', '/tf')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Localization (Lidarslam)
    # Provides map->odom
    scan_matcher = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scan_matcher',
        parameters=[lidarslam_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('input_cloud', '/model/my_bot/cloud'), 
            ('imu', '/model/my_bot/imu'),
            ('odom', '/model/my_bot/odometry'),
            ('map', '/map_cloud')
        ],
        output='screen'
    )

    graph_based_slam = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        parameters=[lidarslam_config, {'use_sim_time': use_sim_time}],
        remappings=[('map', '/map_cloud')],
        output='screen'
    )

    # 3. Map Processing
    # Analyzes PCD and publishes /map (OccupancyGrid) for Nav2
    pcd_to_gridmap = Node(
        package=pkg_name,
        executable='pcd_to_gridmap_node',
        parameters=[{
            'pcd_filename': map_pcd,
            'resolution': 0.1,
            'map_frame_id': 'map',
            'max_slope_deg': 25.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # 4. Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config,
            'autostart': 'true',
        }.items(),
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_pcd', default_value=default_pcd),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        bridge_tf,
        scan_matcher,
        graph_based_slam,
        pcd_to_gridmap,
        nav2,
        rviz
    ])
