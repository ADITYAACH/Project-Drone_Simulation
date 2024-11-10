import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the directory paths for the package, URDF, and configuration files
    package_name = 'drone_simulation'
    package_dir = get_package_share_directory(package_name)
    
    # File paths
    urdf_file = os.path.join(package_dir, 'urdf', 'drone_again.urdf')
    world = os.path.join(package_dir, 'urdf', 'city.urdf')
    control_file = os.path.join(package_dir, 'config', 'drone.yaml')
    config_file = os.path.join(package_dir, 'config', 'rtabmap_config.yaml')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Create robot_description parameter
    robot_description = ParameterValue(
    	urdf_file,
    	value_type=str
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Load URDF of the drone
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Launch RTAB-Map
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            config_file,
            {
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': True,
                'subscribe_imu': True,
                'subscribe_rgbd': False,
                'approx_sync': True,
                'queue_size': 10
            }
        ],
        remappings=[
            # Update these according to your URDF sensor topics
            ('rgb/image', '/drone/camera/image_raw'),
            ('rgb/camera_info', '/drone/camera/camera_info'),
            ('depth/image', '/depth_camera/depth/image_depth'),
            ('depth/camera_info', '/depth_camera/depth/camera_info'),
            ('scan', '/lidar/scan'),
            ('imu', '/imu'),
            ('gps/fix', '/gps/data')
        ]
    )
    
    # RTAB-Map visualization
    rtabmapviz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'subscribe_imu': True,
            'approx_sync': True,
            'queue_size': 10
        }],
        remappings=[
            # Same remappings as rtabmap node
            ('rgb/image', '/drone/camera/image_raw'),
            ('rgb/camera_info', '/drone/camera/camera_info'),
            ('depth/image', '/depth_camera/depth/image_depth'),
            ('depth/camera_info', '/depth_camera/depth/camera_info'),
            ('scan', '/lidar/scan'),
            ('imu', '/imu')
        ]
    )
    
    # RViz configuration
    rviz_config_file = os.path.join(package_dir, 'config', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Spawn the drone in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drone', '-topic', 'robot_description'],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Nodes
        gazebo_launch,
        robot_state_publisher,
        rtabmap_node,
        rtabmapviz_node,
        rviz,
        spawn_entity
    ])
