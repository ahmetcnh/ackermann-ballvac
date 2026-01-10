"""
Ball Collection Launch File

This launch file starts the ball perception and collector nodes for the
autonomous ball collecting demo. It assumes the simulation is already running
(via ros2 launch ballvac_bringup ballvac_spawn.launch.py).

Usage:
    ros2 launch ballvac_ball_collector ball_collect.launch.py

    With custom parameters:
    ros2 launch ballvac_ball_collector ball_collect.launch.py \
        collect_distance:=0.6 \
        search_speed:=0.4

Author: Ball Collector Package
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for ball collection nodes."""
    
    # ==========================================================================
    # Declare launch arguments (parameters that can be overridden)
    # ==========================================================================
    
    # Topic configuration
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/front_raw',
        description='Camera image topic for ball detection'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LiDAR scan topic for obstacle avoidance'
    )
    
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cmd_vel_in',
        description='Velocity command topic'
    )
    
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='/ball_detections',
        description='Topic for publishing/subscribing ball detections'
    )
    
    # Delete service - world name must match the SDF world name ("my_world")
    delete_service_arg = DeclareLaunchArgument(
        'delete_service',
        default_value='/world/my_world/remove',
        description='Gazebo entity deletion service name'
    )
    
    # Spawn service - world name must match the SDF world name ("my_world")
    spawn_service_arg = DeclareLaunchArgument(
        'spawn_service',
        default_value='/world/my_world/create',
        description='Gazebo entity spawn service name'
    )
    
    # Control parameters
    collect_distance_arg = DeclareLaunchArgument(
        'collect_distance',
        default_value='0.5',
        description='Distance (meters) at which to collect a ball'
    )
    
    obstacle_stop_arg = DeclareLaunchArgument(
        'obstacle_stop',
        default_value='0.5',
        description='Distance (meters) at which to stop for obstacles'
    )
    
    obstacle_slow_arg = DeclareLaunchArgument(
        'obstacle_slow',
        default_value='1.0',
        description='Distance (meters) at which to slow down for obstacles'
    )
    
    search_speed_arg = DeclareLaunchArgument(
        'search_speed',
        default_value='0.5',
        description='Linear speed (m/s) during search state'
    )
    
    approach_speed_arg = DeclareLaunchArgument(
        'approach_speed',
        default_value='0.4',
        description='Linear speed (m/s) during approach state'
    )
    
    max_steer_arg = DeclareLaunchArgument(
        'max_steer',
        default_value='0.5',
        description='Maximum steering angular velocity (rad/s)'
    )
    
    steering_gain_arg = DeclareLaunchArgument(
        'steering_gain',
        default_value='2.0',
        description='Proportional gain for steering towards ball'
    )
    
    radius_threshold_arg = DeclareLaunchArgument(
        'radius_threshold',
        default_value='150.0',
        description='Ball radius (pixels) threshold for collection - higher = closer'
    )
    
    # Perception parameters
    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area',
        default_value='100',
        description='Minimum contour area (pixels^2) for ball detection'
    )
    
    publish_debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Whether to publish debug image with detections'
    )
    
    camera_hfov_arg = DeclareLaunchArgument(
        'camera_hfov',
        default_value='1.658',
        description='Camera horizontal field of view (radians)'
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
            'image_topic': LaunchConfiguration('image_topic'),
            'detection_topic': LaunchConfiguration('detection_topic'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'camera_hfov': LaunchConfiguration('camera_hfov'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    # ==========================================================================
    # Motion Controller Node
    # ==========================================================================
    motion_controller_node = Node(
        package='ballvac_control',
        executable='motion_controller_node',
        name='motion_controller_node',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel_in',
            'output_topic': '/cmd_vel',
        }]
    )

    # ==========================================================================
    # Ball Collector Node
    # ==========================================================================
    ball_collector_node = Node(
        package='ballvac_ball_collector',
        executable='ball_collector_node',
        name='ball_collector_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'detection_topic': LaunchConfiguration('detection_topic'),
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'delete_service': LaunchConfiguration('delete_service'),
            'spawn_service': LaunchConfiguration('spawn_service'),
            'collect_distance_m': LaunchConfiguration('collect_distance'),
            'obstacle_stop_m': LaunchConfiguration('obstacle_stop'),
            'obstacle_slow_m': LaunchConfiguration('obstacle_slow'),
            'search_speed': LaunchConfiguration('search_speed'),
            'approach_speed': LaunchConfiguration('approach_speed'),
            'max_steer': LaunchConfiguration('max_steer'),
            'steering_gain': LaunchConfiguration('steering_gain'),
            'approach_radius_threshold': LaunchConfiguration('radius_threshold'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    # ==========================================================================
    # Log startup info
    # ==========================================================================
    startup_info = LogInfo(
        msg=['Starting Ball Collection Demo...\n',
             '  - Make sure simulation is running: ros2 launch ballvac_bringup ballvac_spawn.launch.py\n',
             '  - Check /ball_detections topic for detected balls\n',
             '  - Check /ball_perception/debug_image for visualization']
    )

    # ==========================================================================
    # Return launch description
    # ==========================================================================
    return LaunchDescription([
        # Launch arguments
        image_topic_arg,
        scan_topic_arg,
        cmd_topic_arg,
        detection_topic_arg,
        delete_service_arg,
        spawn_service_arg,
        collect_distance_arg,
        obstacle_stop_arg,
        obstacle_slow_arg,
        search_speed_arg,
        approach_speed_arg,
        max_steer_arg,
        steering_gain_arg,
        radius_threshold_arg,
        min_contour_area_arg,
        publish_debug_image_arg,
        camera_hfov_arg,
        
        # Info message
        startup_info,
        
        # Nodes
        ball_perception_node,
        motion_controller_node,
        ball_collector_node,
    ])
