import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]
    
    if controller == "joystick":
        node = Node(
            package="drone_simulation",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )
    elif controller == "auto":
        node = Node(
            package="drone_simulation",
            executable="auto",
            namespace=namespace,
            output="screen",
        )
    else:
        node = Node(
            package="drone_simulation",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]

def generate_launch_description():
    flood_pkg_path = get_package_share_directory('drone_simulation')

    rviz_path = os.path.join(
        flood_pkg_path, "config", "rviz.rviz"
    )
    
    yaml_file_path = os.path.join(
        get_package_share_directory('drone_simulation'),
        'config', 'drone.yaml'
    )

    model_ns = "drone"

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", rviz_path
            ],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(flood_pkg_path, 'launch', 'dronecontrol.py')
            )
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),
    ])
