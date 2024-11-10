#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_flood = get_package_share_directory('drone_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # World file
    world = os.path.join(pkg_flood, 'world', 'flooded_city_with_drone.world')

    # URDF file
    urdf_file = os.path.join(pkg_flood, 'urdf', 'drone_again.urdf')

    # Process the URDF file
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0')
    y_pose = LaunchConfiguration('y_pose', default='0')
    z_pose = LaunchConfiguration('z_pose', default='0.04')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial x-position of the drone')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial y-position of the drone')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Initial z-position of the drone')

    # Start Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Start Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn the drone in Gazebo
    # spawn_drone_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'drone_again', 
    #         '-topic', '/robot_description',
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', z_pose
    #     ],
    #     output='screen'
    # )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_drone_cmd)

    return ld
