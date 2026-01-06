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

    # Set Ignition Gazebo IP to localhost
    os.environ['IGN_IP'] = '127.0.0.1'

    # Paths
    world_path = os.path.join(pkg_dir, 'worlds', 'uneven_terrain.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'mesh_bot.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_simulation.rviz')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
        arguments=['-topic', 'robot_description', '-name', 'mesh_bot', '-z', '0.5'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_sensors',
        arguments=[
            '/model/mesh_bot/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
            '/model/mesh_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/mesh_bot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/mesh_bot/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/model/mesh_bot/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/mesh_bot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/mesh_bot/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/world/uneven_terrain/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        remappings=[
            ('/model/mesh_bot/points', '/points'),
            ('/model/mesh_bot/joint_states', '/joint_states'),
            ('/model/mesh_bot/cmd_vel', '/cmd_vel'),
            ('/model/mesh_bot/imu', '/imu'),
            ('/world/uneven_terrain/clock', '/clock'),
            ('/model/mesh_bot/scan', '/scan'),
            ('/model/mesh_bot/odometry', '/odom')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # TF Bridge (Odom -> Base Link if provided by Gazebo plugin)
    # The plugin publishes /model/mesh_bot/tf_odom which we bridge to /tf
    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_tf',
        arguments=['/model/mesh_bot/tf_odom@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[('/model/mesh_bot/tf_odom', '/tf')], # Bridge directly to /tf
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Optional: RViz
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        bridge,
        bridge_tf,
        # rviz
    ])
