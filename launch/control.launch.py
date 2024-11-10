import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('drone_simulation')

    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'drone_again.urdf')

    # Control configuration file path
    control_file = os.path.join(pkg_dir, 'config', 'drone.yaml')

    # Process the URDF file
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'drone_again', '-topic', '/robot_description'],
        output='screen'
    )

    # Launch RViz
    rviz_config_file = os.path.join(pkg_dir, 'config', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Load ROS 2 Control parameters
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, control_file],
        output='screen'
    )

    # Joint State Publisher (GUI)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Spawner nodes for controllers (e.g., joint_state_broadcaster, position_controller)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        ros2_control_node,
        spawn_entity,
        joint_state_publisher_gui,
        rviz,
        joint_state_broadcaster_spawner,
        position_controller_spawner
    ])

