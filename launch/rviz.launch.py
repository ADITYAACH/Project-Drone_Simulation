import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('drone_simulation')

    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'city.urdf')

    # Process the URDF file
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml()}

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

    # Joint State Publisher (GUI)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
