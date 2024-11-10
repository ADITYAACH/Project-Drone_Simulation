import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"],
                                    description="Whether to execute gzclient")
    xacro_file_name = "drone_again.urdf.xacro"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file = os.path.join(
        get_package_share_directory("drone_simulation"),
        "urdf", xacro_file_name
    )
    yaml_file_path = os.path.join(
        get_package_share_directory('drone_simulation'),
        'config', 'drone.yaml'
    )   
    
    robot_description_config = xacro.process_file(urdf_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()
    # get ns from yaml
    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"] #+ "/"
    print("namespace: ", model_ns)

    world_file = os.path.join(
        get_package_share_directory("drone_simulation"),
        "world", "flooded_city.world"
    )

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    # Define spawn coordinates
    spawn_x = LaunchConfiguration('spawn_x', default='-11.0')
    spawn_y = LaunchConfiguration('spawn_y', default='-25.0')
    spawn_z = LaunchConfiguration('spawn_z', default='8.0')

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x', default_value='-11.0', description='Spawn X coordinate'),
        DeclareLaunchArgument('spawn_y', default_value='-25.0', description='Spawn Y coordinate'),
        DeclareLaunchArgument('spawn_z', default_value='8.0', description='Spawn Z coordinate'),
        use_gui,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc, "frame_prefix": model_ns + "/"}],
            arguments=[robot_desc]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient),

        Node(
            package="drone_simulation",
            executable="spawn_drone",
            arguments=[robot_desc, model_ns, spawn_x, spawn_y, spawn_z],
            output="screen"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),
    ])
