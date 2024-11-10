import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue

def get_cartographer_node(context, *_, **kwargs):  # Cartographer node setup based on launch config
    use_trajectory = context.launch_configurations["use_trajectory"]
    
    if use_trajectory == "true":
        node = Node(
            package='cartographer_ros',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[{'use_sim_time': kwargs['use_sim_time']}],
        )
    else:
        node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer',
            output='screen',
            parameters=[kwargs['cartographer_config_file'], {'use_sim_time': kwargs['use_sim_time']}],
            remappings=[('scan', '/lidar/scan'), ('imu', '/imu'), ('gps/fix', '/gps/data')]
        )
    return [node]

def get_teleop_controller(context, *_, **kwargs):  # Teleop controller setup
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]
    
    if controller == "joystick":
        node = Node(
            package="drone_simulation",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )
    else:
        node = Node(
            package="drone_simulation",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",  # Make sure xterm is installed
        )
    return [node]

def generate_launch_description():
    # Get package paths
    package_name = 'drone_simulation'
    package_dir = get_package_share_directory(package_name)
    
    # Define file paths
    urdf_file = os.path.join(package_dir, 'urdf', 'drone_again.urdf')
    world_file = os.path.join(package_dir, 'urdf', 'city.urdf')
    rviz_config_file = os.path.join(package_dir, 'config', 'rviz.rviz')
    cartographer_config_file = os.path.join(package_dir, 'config', 'cartographer_config.yaml')
    yaml_file_path = os.path.join(package_dir, 'config', 'drone.yaml')
    
    # Get namespace from YAML with error handling
    model_ns = "drone"  # default namespace
    try:
        with open(yaml_file_path, 'r') as f:
            yaml_dict = yaml.safe_load(f)
            model_ns = yaml_dict.get("namespace", "drone")
    except Exception as e:
        print(f"Warning: Error loading YAML file: {e}. Using default namespace: {model_ns}")

    # Launch configuration parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_trajectory = LaunchConfiguration('use_trajectory', default='false')
    controller = LaunchConfiguration('controller', default='keyboard')

    # Create robot_description parameter
    robot_description = ParameterValue(urdf_file, value_type=str)
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    use_trajectory_arg = DeclareLaunchArgument(
        'use_trajectory',
        default_value='false',
        description='Use the trajectory node for Cartographer if true'
    )
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='keyboard',
        description='Type of controller: keyboard (default) or joystick',
        choices=['keyboard', 'joystick']
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Spawn drone entity in Gazebo
    spawn_drone_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drone', '-topic', 'robot_description'],
        output='screen'
    )

    # Joy node - only launch if using joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        namespace=model_ns,
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'joystick'"]))
    )

    # Use OpaqueFunction for conditional Cartographer node selection
    cartographer_node = OpaqueFunction(
        function=get_cartographer_node,
        kwargs={'cartographer_config_file': cartographer_config_file, 'use_sim_time': use_sim_time}
    )

    # Teleop controller
    teleop_controller = OpaqueFunction(
        function=get_teleop_controller,
        kwargs={'model_ns': model_ns},
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time_arg,
        use_trajectory_arg,
        controller_arg,
        gazebo_launch,
        robot_state_publisher_node,
        rviz_node,
        spawn_drone_entity_node,
        joy_node,
        cartographer_node,
        teleop_controller,
    ])

