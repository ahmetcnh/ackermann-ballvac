"""
Integrated Ball Collection Launch File with Nav2 and SLAM

This launch file provides a complete, integrated system for:
1. Gazebo simulation with ball arena
2. Robot spawning with proper TF setup
3. SLAM Toolbox for continuous mapping
4. Nav2 stack for path planning and obstacle avoidance
5. Ball perception and collection behavior

Usage:
    ros2 launch saye_ball_collector ball_collection_full.launch.py

Arguments:
    use_sim_time: Use simulation time (default: true)
    world_name: Gazebo world name (default: ball_arena)
    use_rviz: Launch RViz2 for visualization (default: true)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    GroupAction,
    RegisterEventHandler,
    LogInfo,
    ExecuteProcess
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # =========================================================================
    # Package paths
    # =========================================================================
    pkg_ballvac_bringup = get_package_share_directory('ballvac_bringup')
    pkg_ballvac_description = get_package_share_directory('ballvac_description')
    pkg_ballvac_ball_collector = get_package_share_directory('ballvac_ball_collector')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # =========================================================================
    # File paths
    # =========================================================================
    robot_sdf = os.path.join(pkg_ballvac_description, 'models', 'ballvac', 'model.sdf')
    world_sdf = os.path.join(pkg_ballvac_description, 'worlds', 'ball_arena.sdf')
    bridge_config = os.path.join(pkg_ballvac_bringup, 'config', 'ros_gz_bridge.yaml')
    nav2_params = os.path.join(pkg_ballvac_ball_collector, 'config', 'nav2_ball_collector_params.yaml')
    rviz_config = os.path.join(pkg_ballvac_bringup, 'rviz', 'navigation.rviz')
    
    # =========================================================================
    # Launch arguments
    # =========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='ball_arena')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='ball_arena',
        description='Gazebo world name (must match SDF)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    # =========================================================================
    # Set use_sim_time globally
    # =========================================================================
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # =========================================================================
    # 1. Gazebo Simulation
    # =========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf}.items(),
    )
    
    # =========================================================================
    # 2. Spawn Robot
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
    # 3. ROS-Gazebo Bridges
    # =========================================================================
    bridge_topics = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
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
    # 4. TF Publishers
    # =========================================================================
    # Odom -> base_link TF from Gazebo odometry
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
    
    # Static TF for lidar frame (lidar is attached to base_link)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['0.05', '0', '0.106', '0', '0', '0', 'ballvac', 'ballvac/lidar_link'],
        parameters=[{'use_sim_time': True}],
    )
    
    # =========================================================================
    # 5. SLAM Toolbox
    # =========================================================================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
    )
    
    # =========================================================================
    # 6. Nav2 Stack (without map_server since we use SLAM)
    # =========================================================================
    nav2_nodes = GroupAction([
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav')  # Output to collision monitor
            ]
        ),
        
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': True}],
        ),
        
        # Behavior server (recoveries)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': True}],
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': True}],
        ),
        
        # Smoother server
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': True}],
        ),
        
        # Velocity smoother - outputs directly to cmd_vel (collision monitor disabled)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[nav2_params, {'use_sim_time': True}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')  # Output directly to robot
            ]
        ),
        
        # Lifecycle manager for Nav2 nodes
        # NOTE: collision_monitor removed due to parameter compatibility issues
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'smoother_server',
                    'velocity_smoother',
                ],
                'bond_timeout': 0.0,  # Disable bond for simulation
            }],
        ),
    ])
    
    # =========================================================================
    # 7. Ball Perception Node
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
    # 8. Ball Launcher Node
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
    # 9. Navigation-Enabled Ball Collector Node
    # =========================================================================
    nav_ball_collector = Node(
        package='ballvac_ball_collector',
        executable='nav_ball_collector_node',
        name='nav_ball_collector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Topics
            'scan_topic': '/scan',
            'detection_topic': '/ball_detections',
            'cmd_topic': '/cmd_vel',
            'odom_topic': '/odom',
            # Services
            'delete_service': '/world/ball_arena/remove',
            'spawn_service': '/world/ball_arena/create',
            # Frames
            'map_frame': 'map',
            'robot_frame': 'ballvac',
            'camera_frame': 'ballvac/camera_link',
            # Navigation parameters
            'nav_to_approach_distance': 0.6,
            'collect_distance_m': 0.40,
            # Speed and steering - INCREASED for visible movement
            'approach_speed': 0.8,
            'obstacle_stop_m': 0.40,
            'obstacle_slow_m': 1.0,
            'obstacle_avoid_m': 1.8,
            'max_steer': 1.8,
            'control_rate': 20.0,
            'steering_gain': 3.0,
            # Ball detection parameters
            'approach_radius_threshold': 120.0,
            'min_ball_radius': 12.0,
            'max_ball_radius': 200.0,
            'collection_cooldown': 0.5,
            'target_lost_timeout': 8.0,  # INCREASED - don't give up so fast
            # Camera parameters
            'camera_fov_horizontal': 1.3962634,
            'camera_resolution_width': 640.0,
            'ball_actual_diameter': 0.15,
            # Exploration parameters
            'exploration_waypoint_distance': 2.0,
            'exploration_timeout': 25.0,
            # Recovery parameters
            'recover_duration': 1.5,
            'recover_speed': 0.6,
        }],
    )
    
    # =========================================================================
    # 10. RViz2 Visualization
    # =========================================================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )
    
    # =========================================================================
    # Launch Description - Staged startup for proper initialization
    # =========================================================================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_world_name,
        declare_use_rviz,
        
        # Global parameter
        set_use_sim_time,
        
        # Log startup info
        LogInfo(msg=['=== Starting Ball Collection System ===']),
        LogInfo(msg=['World: ball_arena']),
        LogInfo(msg=['Robot: ballvac (Ackermann steering)']),
        
        # Stage 1: Gazebo simulation
        gazebo,
        spawn_robot,
        
        # Stage 2: Bridges (1s delay)
        TimerAction(
            period=1.0,
            actions=[bridge_topics, delete_bridge, spawn_bridge]
        ),
        
        # Stage 3: TF publishers (2s delay)
        TimerAction(
            period=2.0,
            actions=[odom_tf_publisher, static_tf_lidar]
        ),
        
        # Stage 4: SLAM Toolbox (3s delay)
        TimerAction(
            period=3.0,
            actions=[slam_toolbox]
        ),
        
        # Stage 5: Nav2 stack (5s delay - wait for SLAM)
        TimerAction(
            period=5.0,
            actions=[nav2_nodes]
        ),
        
        # Stage 6: Ball handling (8s delay - wait for Nav2)
        TimerAction(
            period=8.0,
            actions=[ball_perception, ball_launcher]
        ),
        
        # Stage 7: Ball collector (10s delay - wait for everything)
        TimerAction(
            period=10.0,
            actions=[nav_ball_collector]
        ),
        
        # Stage 8: RViz (2s delay)
        TimerAction(
            period=2.0,
            actions=[rviz]
        ),
    ])
