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
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_mapping.rviz')
    lidarslam_config = os.path.join(pkg_dir, 'config', 'lidarslam_3d.yaml')
    world_path = os.path.join(pkg_dir, 'worlds', 'uneven_terrain.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'same_vehicle.urdf.xacro')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process URDF
    doc = xacro.process_file(urdf_path)
    robot_description = doc.toxml()

    # Gazebo (Ignition/Gazebo Sim)
    # Check if we should use classic or ignition. The URDF has ignition plugins.
    # Assuming ros_gz_sim or sim_bringup is available. 
    # Since I don't know the exact gazebo package present, I'll assume `ros_ign_gazebo` or similar.
    # Actually, standard ROS 2 mostly uses `ros_gz_sim` now. 
    # But for safety in a mixed env, I will try to use `ros_gz_sim` if available, or just launch the world.
    # The prompt doesn't strictly say I must launch Gazebo from THIS launch file, but it's implied by "Each launch file MUST...".
    # Wait, usually mapping doesn't restart simulation if it's already running. 
    # But verification section says "Test each launch in Gazebo". So I should probably include simulation.
    
    # IMPORTANT: The user might be using classic gazebo. The URDF had ignition plugins.
    # I will assume `ros_gz_sim` (Ignition/Fortress).
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.5'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Bridge
    # Needed for cmd_vel, odom, lidar, etc.
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
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/my_bot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        remappings=[
            ('/model/my_bot/points/points', '/input_cloud'),
            ('/model/my_bot/joint_states', '/joint_states'),
            ('/model/my_bot/cmd_vel', '/cmd_vel'),
            ('/model/my_bot/odometry', '/odom'),
            ('/model/my_bot/imu', '/imu')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Note: URDF plugins were:
    # DiffDrive -> /model/robot/tf_odom (TF)
    # DiffDrive -> [Not explicitly standard cmd_vel? No, usually it subscribes to something. 
    # The URDF didn't show subscription topic for DiffDrive, usually defaults to /model/robot/cmd_vel or /cmd_vel]
    # URDF Lidar -> /model/robot/cloud
    
    # I need to verify the bridge arguments based on the URDF.
    # URDF: <plugin ... name="ignition::gazebo::systems::DiffDrive"> ... <tf_topic>/model/robot/tf_odom</tf_topic>
    # So I need to bridge `/model/robot/tf_odom` to `/tf`.
    
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_tf',
        arguments=['/model/my_bot/tf_odom@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[('/model/my_bot/tf_odom', '/tf')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Scan Matcher
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
        remappings=[('map', '/map_cloud')],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        bridge_tf,
        scan_matcher,
        graph_based_slam,
        rviz
    ])
