import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'terrain_nav_bringup'
    pkg_dir = get_package_share_directory(pkg_name)

    # 1. Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'simulation.launch.py'))
    )

    # 2. Perception (Elevation Mapping)
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'perception.launch.py'))
    )

    # 3. Mesh Generation (Optional: Can be run manually or triggered)
    # mesh_gen = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'mesh_gen.launch.py'))
    # )

    # 4. Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'navigation.launch.py'))
    )

    return LaunchDescription([
        simulation,
        perception,
        # mesh_gen, 
        navigation
    ])
