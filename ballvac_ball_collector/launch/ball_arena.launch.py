"""
Ball Arena Launch File

This launch file starts:
1. Gazebo simulation with ball arena (room with walls and obstacles)
2. Robot spawning
3. SLAM Toolbox for mapping
4. Ball launcher (spawns balls at random positions)
5. Ball collector (explores room, collects balls)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_ballvac_bringup = get_package_share_directory('ballvac_bringup')
    pkg_ballvac_description = get_package_share_directory('ballvac_description')
    pkg_ballvac_ball_collector = get_package_share_directory('ballvac_ball_collector')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Robot model
    robot_sdf = os.path.join(pkg_ballvac_description, 'models', 'ballvac', 'model.sdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # =========================================================================
    # Gazebo Simulation using ros_gz_sim launcher
    # =========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_ballvac_description,
            'worlds',
            'ball_arena.sdf'
        ])}.items(),
    )
    
    # =========================================================================
    # Spawn Robot
    # =========================================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ballvac',
            '-file', robot_sdf,
            '-x', '0.0',
            '-y', '-3.0',
            '-z', '0.2',
            '-Y', '1.57'  # Face towards room center
        ],
        output='screen'
    )
    
    # =========================================================================
    # ROS-Gazebo Bridges (using existing config from ballvac_bringup)
    # =========================================================================
    bridge_config = os.path.join(pkg_ballvac_bringup, 'config', 'ros_gz_bridge.yaml')
    
    # Topic bridges using config file
    bridge_topics = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    # Service bridges for entity management
    delete_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='delete_bridge',
        arguments=[
            '/world/ball_arena/remove@ros_gz_interfaces/srv/DeleteEntity'
        ],
        output='screen'
    )
    
    spawn_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='spawn_bridge',
        arguments=[
            '/world/ball_arena/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        output='screen'
    )
    
    # =========================================================================
    # Ball Perception Node
    # =========================================================================
    ball_perception = Node(
        package='ballvac_ball_collector',
        executable='ball_perception_node',
        name='ball_perception_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/front_raw',
            'detection_topic': '/ball_detections',
            'publish_debug_image': True,
            'min_contour_area': 200,
        }]
    )
    
    # =========================================================================
    # Ball Launcher Node (spawns balls at random positions)
    # =========================================================================
    ball_launcher = Node(
        package='ballvac_ball_collector',
        executable='ball_launcher_node',
        name='ball_launcher_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'spawn_service': '/world/ball_arena/create',
            'launcher_x': -4.3,
            'launcher_y': 0.0,
            'launcher_z': 1.2,
            'min_launch_speed': 3.0,
            'max_launch_speed': 5.0,
            'launch_interval': 10.0,
            'max_balls': 15,
            'auto_launch': True,
        }]
    )
    
    # =========================================================================
    # Ball Collector Node (uses LiDAR for obstacle avoidance)
    # =========================================================================
    ball_collector = Node(
        package='ballvac_ball_collector',
        executable='ball_collector_node',
        name='ball_collector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/scan',
            'cmd_topic': '/cmd_vel',
            'detection_topic': '/ball_detections',
            'delete_service': '/world/ball_arena/remove',
            'spawn_service': '/world/ball_arena/create',
            'linear_speed': 0.4,
            'angular_speed': 0.3,
            'max_steer': 0.5,
            'obstacle_stop_distance': 0.5,
            'obstacle_slow_distance': 1.5,
            'collection_distance': 0.6,
            'min_ball_radius': 25.0,
            'max_ball_radius': 160.0,
            'collection_cooldown': 2.0,
        }]
    )
    
    # =========================================================================
    # Odom TF Publisher - Publishes odom->base_link TF for SLAM
    # =========================================================================
    odom_tf_publisher = Node(
        package='ballvac_ball_collector',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'ballvac',
        }]
    )
    
    # =========================================================================
    # SLAM Toolbox for mapping
    # =========================================================================
    slam_config = os.path.join(pkg_ballvac_ball_collector, 'config', 'slam_params.yaml')
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': True}
        ],
    )
    
    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Start simulation
        gazebo,
        spawn_robot,
        
        # Bridges
        bridge_topics,
        delete_bridge,
        spawn_bridge,
        
        # Odom TF publisher (needed for SLAM)
        TimerAction(
            period=2.0,
            actions=[odom_tf_publisher]
        ),
        
        # SLAM (delayed to let TF be ready)
        TimerAction(
            period=4.0,
            actions=[slam_toolbox]
        ),
        
        # Ball handling (delayed to let simulation start)
        TimerAction(
            period=5.0,
            actions=[ball_perception, ball_launcher, ball_collector]
        ),
    ])
