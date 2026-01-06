"""
Navigation-Enabled Ball Collection Launch File

This launch file starts the navigation-enabled ball collector along with
Nav2 stack for intelligent path planning.

Note: This launch file assumes the simulation is already running via:
    ros2 launch saye_bringup ball_arena_spawn.launch.py

Usage:
    # With SLAM (recommended):
    ros2 launch saye_ball_collector nav_ball_collect.launch.py slam:=True

    # With pre-saved map:
    ros2 launch saye_ball_collector nav_ball_collect.launch.py map_file:=/path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for navigation-enabled ball collection."""
    
    # Get package directories
    pkg_ballvac_bringup = get_package_share_directory('ballvac_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # ==========================================================================
    # Declare launch arguments
    # ==========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Run SLAM instead of using pre-saved map'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_ballvac_bringup, 'maps', 'map.yaml'),
        description='Path to map YAML file'
    )
    
    nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_ballvac_bringup, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 parameters file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',  # Disabled by default since simulation already has RViz
        description='Launch RViz for visualization'
    )
    
    # ==========================================================================
    # Get launch configurations
    # ==========================================================================
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # ==========================================================================
    # Nav2 Bringup
    # ==========================================================================
    
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
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
        parameters=[{'use_sim_time': use_sim_time}],
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
            'use_sim_time': use_sim_time,
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
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
            'detection_topic': '/ball_detections',
            'cmd_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'delete_service': '/world/ball_arena/remove',
            'spawn_service': '/world/ball_arena/create',
            'map_frame': 'map',
            'robot_frame': 'ballvac',
            'camera_frame': 'ballvac/camera_link',
            'nav_to_approach_distance': 1.0,
            'collect_distance_m': 0.4,
            'approach_speed': 0.8,
            'obstacle_stop_m': 0.25,
            'obstacle_slow_m': 0.5,
            'max_steer': 0.8,
            'control_rate': 30.0,
            'steering_gain': 3.0,
            'approach_radius_threshold': 120.0,
            'min_ball_radius': 12.0,
            'max_ball_radius': 180.0,
            'collection_cooldown': 0.3,
            'target_lost_timeout': 1.5,
            'camera_fov_horizontal': 1.3962634,
            'camera_resolution_width': 640.0,
            'ball_actual_diameter': 0.15,
            'exploration_waypoint_distance': 2.5,
            'exploration_timeout': 15.0,
            'recover_duration': 0.4,
            'recover_speed': 0.8,
        }],
    )
    
    # ==========================================================================
    # RViz
    # ==========================================================================
    
    rviz_config_file = os.path.join(pkg_ballvac_bringup, 'rviz', 'navigation.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )
    
    # ==========================================================================
    # Return launch description
    # ==========================================================================
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        slam_arg,
        map_file_arg,
        nav_params_arg,
        use_rviz_arg,
        
        # Info
        LogInfo(msg=['Starting Navigation-Enabled Ball Collection System']),
        LogInfo(msg=['Make sure simulation is running: ros2 launch ballvac_bringup ball_arena_spawn.launch.py']),
        
        # Nav2 (includes SLAM if enabled)
        nav2_bringup_cmd,
        static_tf_map_odom,
        
        # Ball collection nodes
        ball_perception_node,
        nav_ball_collector_node,
        
        # Visualization (optional, simulation already has RViz)
        rviz_node,
    ])
