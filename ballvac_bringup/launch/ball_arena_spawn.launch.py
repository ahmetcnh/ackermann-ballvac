"""
Ball Arena Spawn Launch File

Launches the complete ball collection system:
- Gazebo simulation with ball_arena world
- Robot spawn
- Nav2 with SLAM
- Ball perception and collection nodes

Usage:
    ros2 launch saye_bringup ball_arena_spawn.launch.py
    
    # Without SLAM (needs pre-saved map):
    ros2 launch saye_bringup ball_arena_spawn.launch.py slam:=False map_file:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction, LogInfo
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ballvac_bringup')
    pkg_project_description = get_package_share_directory('ballvac_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Path to the SDF file
    sdf_file = os.path.join(pkg_project_description, 'models', 'ballvac', 'model.sdf')
    
    # Path to ball arena world
    world_file = os.path.join(pkg_project_description, 'worlds', 'ball_arena.sdf')
    
    # Nav2 params file
    nav_params_file = os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map_file')

    # Setup to launch the simulator with ball_arena world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file}.items(),
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'ballvac.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    # Service bridge for entity deletion (for ball collection)
    # Note: world name is 'ball_arena'
    delete_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/ball_arena/remove@ros_gz_interfaces/srv/DeleteEntity'
        ],
        output='screen'
    )
    
    # Service bridge for entity spawning (for ball respawning)
    spawn_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/ball_arena/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        output='screen'
    )
    
    # Spawn the robot at a reasonable starting position
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', sdf_file,
            '-name', 'ballvac',
            '-allow_renaming', 'true',
            '-x', '0',
            '-y', '0',
            '-z', '0.35'
        ]
    )
    
    # Static TF: odom -> saye (base_link)
    # This is needed because Gazebo's odometry plugin may not publish this transform
    # We'll use the odometry data to provide this transform dynamically
    # For now, publish a static identity transform that will be overridden
    
    # Robot state publisher (if URDF available) or static TF for robot frames
    # Since we're using SDF directly, we need to ensure the TF tree is complete
    
    # Odom to TF node - converts odometry messages to TF transforms
    # This is essential for SLAM to work properly
    odom_to_tf = Node(
        package='ballvac_bringup',
        executable='odom_to_tf_node.py',
        name='odom_to_tf_node',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'robot_frame': 'ballvac',
        }],
        output='screen',
    )
    
    # ==========================================================================
    # Nav2 Bringup (with SLAM support)
    # ==========================================================================
    
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'map': map_file,
            'params_file': nav_params_file,
            'slam': slam,
        }.items(),
    )
    
    # Static TF: map -> odom (only when not using SLAM)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(slam),
    )
    
    # ==========================================================================
    # Ball Perception Node
    # ==========================================================================
    
    ball_perception_node = Node(
        package='ballvac_ball_collector',
        executable='ball_perception_node',
        name='ball_perception_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/front_raw',
            'detection_topic': '/ball_detections',
            'debug_image': True,
            'red_h_low': 0,
            'red_h_high': 10,
            'red_h_low2': 170,
            'red_h_high2': 180,
            'green_h_low': 35,
            'green_h_high': 85,
            'blue_h_low': 100,
            'blue_h_high': 130,
            'yellow_h_low': 20,
            'yellow_h_high': 35,
            'saturation_low': 100,
            'saturation_high': 255,
            'value_low': 100,
            'value_high': 255,
            'min_radius': 10,
            'max_radius': 200,
            'min_circularity': 0.6,
        }],
    )
    
    # ==========================================================================
    # Navigation-Enabled Ball Collector Node
    # ==========================================================================
    
    nav_ball_collector_node = Node(
        package='ballvac_ball_collector',
        executable='nav_ball_collector_node',
        name='nav_ball_collector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/scan',
            'detection_topic': '/ball_detections',
            'cmd_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'delete_service': '/world/ball_arena/remove',
            'spawn_service': '/world/ball_arena/create',
            'map_frame': 'map',
            'robot_frame': 'ballvac',
            'camera_frame': 'ballvac/camera_link',
        }],
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Use simulation time'),
        DeclareLaunchArgument('slam', default_value='True',
                              description='Run SLAM instead of using pre-saved map'),
        DeclareLaunchArgument('map_file', 
                              default_value=os.path.join(pkg_project_bringup, 'maps', 'map.yaml'),
                              description='Path to map YAML file'),
        
        # Info
        LogInfo(msg=['Starting Ball Arena with Navigation and Ball Collection']),
        
        # Simulation
        gz_sim,
        bridge,
        delete_bridge,
        spawn_bridge,
        gz_spawn_entity,
        
        # Start odom_to_tf with a small delay to ensure bridge is ready
        TimerAction(
            period=2.0,
            actions=[odom_to_tf],
        ),
        
        # Start Nav2 after simulation is ready (5 second delay)
        TimerAction(
            period=5.0,
            actions=[
                nav2_bringup_cmd,
                static_tf_map_odom,
            ],
        ),
        
        # Start ball collection after Nav2 is ready (10 second delay)
        TimerAction(
            period=10.0,
            actions=[
                ball_perception_node,
                nav_ball_collector_node,
                LogInfo(msg=['Ball collection started!']),
            ],
        ),
        
        rviz,
    ])
