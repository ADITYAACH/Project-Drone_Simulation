import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('drone_simulation')
    world_file = os.path.join(pkg_dir, 'world', 'flooded_city.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'drone_again.urdf')

    # Launch Gazebo with the flooded city world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn the drone in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', urdf_file,
                                   '-entity', 'drone',
                                   '-x', '0', '-y', '0', '-z', '5'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
