import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('drone_simulation')

    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'drone_again.urdf')

    # Process the URDF file
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.04')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial x-position of the drone')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial y-position of the drone')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Initial z-position of the drone')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'drone_again', 
            '-topic', '/robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
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

    return LaunchDescription([
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_z_position_cmd,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
