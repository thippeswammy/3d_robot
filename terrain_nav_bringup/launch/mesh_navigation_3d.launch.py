import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)

    # Set Ignition Gazebo IP
    os.environ['IGN_IP'] = '127.0.0.1'

    # Configuration Files
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_navigation.rviz') 
    mbf_config = os.path.join(pkg_dir, 'config', 'mbf_mesh.yaml')
    world_path = os.path.join(pkg_dir, 'worlds', 'uneven_terrain.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'mesh_bot.urdf.xacro')

    # Mesh Map Paths
    default_mesh_map = os.path.join(pkg_dir, 'map', 'uneven_terrain.ply')
    default_mesh_working = os.path.join(pkg_dir, 'map', 'map.h5')

    # Launch Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    mesh_map_path = LaunchConfiguration('mesh_map_path')
    mesh_map_working_path = LaunchConfiguration('mesh_map_working_path')

    # Process URDF
    doc = xacro.process_file(urdf_path)
    robot_description = doc.toxml()

    # 1. Gazebo & Spawning
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

    # ROS GZ Bridge
    # Bridges sensors and TF from Gazebo to ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_sensors',
        arguments=[
            '/model/my_bot/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
            '/model/my_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/my_bot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/my_bot/points/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/model/my_bot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/my_bot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/world/uneven_terrain/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/my_bot/ground_truth@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/my_bot/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/my_bot/points/points', '/model/my_bot/cloud'),
            ('/model/my_bot/joint_states', '/joint_states'),
            ('/model/my_bot/cmd_vel', '/cmd_vel'),
            ('/model/my_bot/imu', '/imu'),
            ('/world/uneven_terrain/clock', '/clock'),
            ('/model/my_bot/pose_static', '/tf_gt')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Bridge TF: Requires separate bridge for performance or message type handling sometimes
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_tf',
        arguments=['/model/my_bot/tf_odom@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[('/model/my_bot/tf_odom', '/tf')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Localization
    # simple_localization_node provides map -> odom transform using Ground Truth Odometry from Gazebo
    simple_localization = Node(
        package=pkg_name,
        executable='simple_localization_node.py',
        name='simple_localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )

    # 3. Mesh Navigation (Move Base Flex)
    # The heart of the stack
    mesh_nav_server = Node(
        name="move_base_flex",
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        remappings=[
            ("/move_base_flex/cmd_vel", "/cmd_vel"),
            ("/move_base_flex/odom", "/model/my_bot/odometry"), # Ensure it gets the bridge Odom topic
        ],
        parameters=[
            mbf_config,
            {
                "mesh_map.mesh_file": mesh_map_path,
                "mesh_map.mesh_working_file": mesh_map_working_path,
                "use_sim_time": use_sim_time
            }
        ],
        output='screen'
    )

    # 4. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Goal Bridge (RViz 2D Nav Goal -> MBF Goal)
    goal_bridge = Node(
        package=pkg_name,
        executable='simple_mbf_goal_bridge.py',
        name='simple_mbf_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # EKF Localization Node
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('mesh_map_path', default_value=default_mesh_map, description='Path to .ply mesh file'),
        DeclareLaunchArgument('mesh_map_working_path', default_value=default_mesh_working, description='Path to .h5 working map file'),
        
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        ekf_node, # Replaces bridge_tf for odom->base_footprint
        
        simple_localization,
        
        mesh_nav_server,
        goal_bridge,
        rviz
    ])
