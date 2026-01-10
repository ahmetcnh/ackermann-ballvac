"""
Multi-Robot Ball Collection Launch File

Launches 3 Ackermann robots (ballvac1, ballvac2, ballvac3) with:
- Unique namespaces and TF prefixes
- One shared map from SLAM (run by ballvac1)
- Individual Nav2 stacks per robot
- Fleet coordinator for ball assignment

Usage:
    ros2 launch ballvac_ball_collector multi_robot_ball_collection.launch.py

Arguments:
    use_sim_time: Use simulation time (default: true)
    use_rviz: Launch RViz2 for visualization (default: true)
"""

import os
import tempfile
import re
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_robot_nav2_params(robot_name, nav2_params_path):
    """Generate a per-robot Nav2 params file with correct frame/topic names."""
    with open(nav2_params_path, 'r') as f:
        data = yaml.safe_load(f)

    odom_frame = f"{robot_name}/odom"
    lidar_frame = f"{robot_name}/lidar_link"

    # SLAM toolbox
    slam_params = data.setdefault('slam_toolbox', {}).setdefault('ros__parameters', {})
    slam_params['base_frame'] = robot_name
    slam_params['odom_frame'] = odom_frame
    slam_params['scan_topic'] = f"/{robot_name}/scan"

    # AMCL
    amcl_params = data.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl_params['base_frame_id'] = robot_name
    amcl_params['odom_frame_id'] = odom_frame
    amcl_params['scan_topic'] = f"/{robot_name}/scan"

    # BT Navigator
    bt_params = data.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
    bt_params['robot_base_frame'] = robot_name
    bt_params['global_frame'] = 'map'
    bt_params['odom_topic'] = f"/{robot_name}/odom"

    # Behavior server - no spin for Ackermann, NO wait (causes pauses)
    behavior_params = data.setdefault('behavior_server', {}).setdefault('ros__parameters', {})
    behavior_params['robot_base_frame'] = robot_name
    behavior_params['local_frame'] = odom_frame
    behavior_params['global_frame'] = 'map'
    behavior_params['behavior_plugins'] = ['backup', 'drive_on_heading']  # Removed wait - causes pauses

    # Velocity smoother
    smoother_params = data.setdefault('velocity_smoother', {}).setdefault('ros__parameters', {})
    smoother_params['odom_topic'] = f"/{robot_name}/odom"

    # Controller server
    controller_params = data.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    controller_params['robot_base_frame'] = robot_name
    controller_params['odom_topic'] = f"/{robot_name}/odom"

    # Local costmap
    local_params = (
        data.setdefault('local_costmap', {})
        .setdefault('local_costmap', {})
        .setdefault('ros__parameters', {})
    )
    local_params['global_frame'] = odom_frame
    local_params['robot_base_frame'] = robot_name
    local_params['plugins'] = ['obstacle_layer', 'inflation_layer']
    local_scan = local_params.setdefault('obstacle_layer', {}).setdefault('scan', {})
    local_scan['topic'] = f"/{robot_name}/scan"
    local_scan['sensor_frame'] = lidar_frame

    # Global costmap
    global_params = (
        data.setdefault('global_costmap', {})
        .setdefault('global_costmap', {})
        .setdefault('ros__parameters', {})
    )
    global_params['robot_base_frame'] = robot_name
    global_params['global_frame'] = 'map'
    global_params.setdefault('static_layer', {})['map_topic'] = '/map'
    global_scan = global_params.setdefault('obstacle_layer', {}).setdefault('scan', {})
    global_scan['topic'] = f"/{robot_name}/scan"
    global_scan['sensor_frame'] = lidar_frame

    # Planner server
    planner_params = data.setdefault('planner_server', {}).setdefault('ros__parameters', {})
    planner_params['robot_base_frame'] = robot_name

    temp_params_dir = tempfile.mkdtemp(prefix=f'{robot_name}_nav2_params_')
    temp_params_path = os.path.join(temp_params_dir, f'{robot_name}_nav2_params.yaml')
    with open(temp_params_path, 'w') as f:
        yaml.safe_dump({robot_name: data}, f, default_flow_style=False)

    return temp_params_path


def parse_initial_balls(world_sdf_path):
    """Parse ball model names and poses from a world SDF."""
    entries = []
    try:
        with open(world_sdf_path, 'r') as f:
            content = f.read()
    except OSError:
        return entries

    pattern = re.compile(
        r'<model\s+name="(ball_[^"]+)">.*?<pose>\s*([^<]+)\s*</pose>',
        re.DOTALL
    )
    for match in pattern.finditer(content):
        name = match.group(1)
        pose_parts = match.group(2).strip().split()
        if len(pose_parts) < 2:
            continue
        try:
            x = float(pose_parts[0])
            y = float(pose_parts[1])
        except ValueError:
            continue
        entries.append(f"{name},{x:.3f},{y:.3f}")
    return entries


def generate_robot_nodes(context, robot_name, robot_index, spawn_x, spawn_y, spawn_yaw, use_sim_time):
    """Generate all nodes for a single robot with proper namespacing."""
    
    pkg_ballvac_bringup = get_package_share_directory('ballvac_bringup')
    pkg_ballvac_description = get_package_share_directory('ballvac_description')
    pkg_ballvac_ball_collector = get_package_share_directory('ballvac_ball_collector')
    
    robot_sdf_original = os.path.join(pkg_ballvac_description, 'models', 'ballvac', 'model.sdf')
    nav2_params = os.path.join(pkg_ballvac_ball_collector, 'config', 'nav2_multi_robot_params.yaml')
    bt_xml_dir = os.path.join(pkg_ballvac_ball_collector, 'behavior_trees')
    bt_nav_to_pose_xml = os.path.join(bt_xml_dir, 'navigate_to_pose_no_spin.xml')
    bt_nav_through_poses_xml = os.path.join(bt_xml_dir, 'navigate_through_poses_no_spin.xml')
    
    # =========================================================================
    # Create modified SDF with robot-specific topics
    # =========================================================================
    with open(robot_sdf_original, 'r') as f:
        sdf_content = f.read()
    
    # Replace generic topic names with robot-specific ones
    # LiDAR topic: scan -> /model/{robot_name}/scan
    sdf_content = re.sub(
        r'<topic>scan</topic>',
        f'<topic>/model/{robot_name}/scan</topic>',
        sdf_content
    )
    # Camera topic: camera_front -> /model/{robot_name}/camera_front
    sdf_content = re.sub(
        r'<topic>camera_front</topic>',
        f'<topic>/model/{robot_name}/camera_front</topic>',
        sdf_content
    )
    # IMU topic: imu -> /model/{robot_name}/imu
    sdf_content = re.sub(
        r'<topic>imu</topic>',
        f'<topic>/model/{robot_name}/imu</topic>',
        sdf_content
    )
    # Cmd vel topic: /cmd_vel -> /{robot_name}/cmd_vel (per-robot control)
    sdf_content = re.sub(
        r'<topic>/cmd_vel</topic>',
        f'<topic>/{robot_name}/cmd_vel</topic>',
        sdf_content
    )
    # LiDAR frame_id: ballvac -> {robot_name}/lidar_link (for proper TF)
    sdf_content = re.sub(
        r'<gz_frame_id>ballvac</gz_frame_id>',
        f'<gz_frame_id>{robot_name}/lidar_link</gz_frame_id>',
        sdf_content
    )
    # Also fix base_link frame reference
    sdf_content = re.sub(
        r'<gz_frame_id>ballvac/base_link</gz_frame_id>',
        f'<gz_frame_id>{robot_name}</gz_frame_id>',
        sdf_content
    )
    
    # Write modified SDF to temp file
    temp_sdf_dir = tempfile.mkdtemp(prefix=f'{robot_name}_sdf_')
    robot_sdf = os.path.join(temp_sdf_dir, f'{robot_name}_model.sdf')
    with open(robot_sdf, 'w') as f:
        f.write(sdf_content)
    
    # Create per-robot Nav2 params with correct frames/topics
    robot_nav2_params = generate_robot_nav2_params(robot_name, nav2_params)

    # Frame names for this robot
    base_frame = f"{robot_name}"
    odom_frame = f"{robot_name}/odom"
    lidar_frame = f"{robot_name}/lidar_link"
    camera_frame = f"{robot_name}/camera_link"
    tf_remaps = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
    ]
    
    nodes = []
    
    # =========================================================================
    # 1. Spawn Robot in Gazebo
    # =========================================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-file', robot_sdf,
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', '0.2',
            '-Y', str(spawn_yaw)
        ],
        output='screen'
    )
    nodes.append(spawn_robot)
    
    # =========================================================================
    # 2. ROS-Gazebo Bridge for this robot
    # =========================================================================
    # Topic bridges - all namespaced under robot_name
    bridge_args = [
        # Cmd vel
        f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        # Odometry
        f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # Lidar
        f'/model/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # Camera  
        f'/model/{robot_name}/camera_front@sensor_msgs/msg/Image[gz.msgs.Image',
        # TF (pose)
        f'/model/{robot_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_bridge',
        arguments=bridge_args,
        remappings=[
            (f'/model/{robot_name}/odometry', f'/{robot_name}/odom'),
            (f'/model/{robot_name}/scan', f'/{robot_name}/scan'),
            (f'/model/{robot_name}/camera_front', f'/{robot_name}/camera/front_raw'),
            (f'/model/{robot_name}/pose', f'/{robot_name}/pose'),
        ],
        output='screen'
    )
    nodes.append(bridge_node)
    
    # =========================================================================
    # 3. Odom -> base_link TF Publisher
    # =========================================================================
    odom_tf_publisher = Node(
        package='ballvac_ball_collector',
        executable='odom_tf_publisher',
        name='odom_tf_publisher',
        namespace=robot_name,
        output='screen',
        remappings=tf_remaps,
        parameters=[{
            'use_sim_time': True,
            'odom_topic': f'/{robot_name}/odom',
            'odom_frame': odom_frame,
            'base_frame': base_frame,
        }]
    )
    nodes.append(odom_tf_publisher)
    
    # =========================================================================
    # 4. Static TF for lidar
    # =========================================================================
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{robot_name}_static_tf_lidar',
        arguments=['0.05', '0', '0.106', '0', '0', '0', base_frame, lidar_frame],
        parameters=[{'use_sim_time': True}],
        remappings=tf_remaps,
    )
    nodes.append(static_tf_lidar)
    
    # Static TF for camera
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{robot_name}_static_tf_camera',
        arguments=['0.1', '0', '0.1', '0', '0', '0', base_frame, camera_frame],
        parameters=[{'use_sim_time': True}],
        remappings=tf_remaps,
    )
    nodes.append(static_tf_camera)
    
    # =========================================================================
    # 5. SLAM / Localization
    # =========================================================================
    if robot_index == 0:
        # First robot runs SLAM and publishes map
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot_name,
            output='screen',
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    'base_frame': base_frame,
                    'odom_frame': odom_frame,
                    'scan_topic': f'/{robot_name}/scan',
                    'map_frame': 'map',
                }
            ],
            remappings=[
                ('map', '/map'),
                ('map_metadata', '/map_metadata'),
                *tf_remaps,
            ]
        )
        nodes.append(slam_node)

        # NOTE: slam_toolbox is NOT a lifecycle node, so no lifecycle manager needed
        # It will start publishing map and TF automatically
    else:
        # Other robots use AMCL for localization on the shared map
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=robot_name,
            output='screen',
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    'base_frame_id': base_frame,
                    'odom_frame_id': odom_frame,
                    'global_frame_id': 'map',
                    'scan_topic': f'/{robot_name}/scan',
                    'set_initial_pose': True,
                    'initial_pose.x': spawn_x,
                    'initial_pose.y': spawn_y,
                    'initial_pose.yaw': spawn_yaw,
                }
            ],
            remappings=[
                ('map', '/map'),
                *tf_remaps,
            ]
        )
        nodes.append(amcl_node)
        
        # AMCL lifecycle manager
        amcl_lifecycle = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['amcl'],
                'bond_timeout': 0.0,
            }]
        )
        nodes.append(amcl_lifecycle)
    
    # =========================================================================
    # 6. Nav2 Stack
    # =========================================================================
    nav2_nodes = [
        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    'robot_base_frame': base_frame,
                    'odom_topic': f'/{robot_name}/odom',
                    # Controller plugins
                    'controller_plugins': ['FollowPath'],
                    'goal_checker_plugins': ['goal_checker'],
                    'progress_checker_plugins': ['progress_checker'],
                    # Progress checker
                    'progress_checker.plugin': 'nav2_controller::SimpleProgressChecker',
                    'progress_checker.required_movement_radius': 0.15,
                    'progress_checker.movement_time_allowance': 10.0,
                    # Goal checker
                    'goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
                    'goal_checker.xy_goal_tolerance': 0.35,
                    'goal_checker.yaw_goal_tolerance': 0.5,
                    'goal_checker.stateful': True,
                    # Inline local costmap config to ensure it overrides any defaults
                    'local_costmap.local_costmap.use_sim_time': True,
                    'local_costmap.local_costmap.rolling_window': True,
                    'local_costmap.local_costmap.width': 6,
                    'local_costmap.local_costmap.height': 6,
                    'local_costmap.local_costmap.resolution': 0.05,
                    'local_costmap.local_costmap.global_frame': odom_frame,
                    'local_costmap.local_costmap.robot_base_frame': base_frame,
                    'local_costmap.local_costmap.footprint': '[[0.18, 0.08], [0.18, -0.08], [-0.18, -0.08], [-0.18, 0.08]]',
                    'local_costmap.local_costmap.plugins': ['obstacle_layer', 'inflation_layer'],
                    'local_costmap.local_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
                    'local_costmap.local_costmap.obstacle_layer.enabled': True,
                    'local_costmap.local_costmap.obstacle_layer.observation_sources': 'scan',
                    'local_costmap.local_costmap.obstacle_layer.footprint_clearing_enabled': True,
                    'local_costmap.local_costmap.obstacle_layer.scan.topic': f'/{robot_name}/scan',
                    'local_costmap.local_costmap.obstacle_layer.scan.sensor_frame': lidar_frame,
                    'local_costmap.local_costmap.obstacle_layer.scan.data_type': 'LaserScan',
                    'local_costmap.local_costmap.obstacle_layer.scan.marking': True,
                    'local_costmap.local_costmap.obstacle_layer.scan.clearing': True,
                    'local_costmap.local_costmap.obstacle_layer.scan.obstacle_max_range': 4.0,
                    'local_costmap.local_costmap.obstacle_layer.scan.obstacle_min_range': 0.25,
                    'local_costmap.local_costmap.obstacle_layer.scan.raytrace_max_range': 5.0,
                    'local_costmap.local_costmap.obstacle_layer.scan.raytrace_min_range': 0.25,
                    # Filter balls from costmap (balls are ~0.05m tall)
                    'local_costmap.local_costmap.obstacle_layer.scan.min_obstacle_height': 0.10,
                    'local_costmap.local_costmap.obstacle_layer.scan.max_obstacle_height': 2.5,
                    'local_costmap.local_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
                    'local_costmap.local_costmap.inflation_layer.enabled': True,
                    'local_costmap.local_costmap.inflation_layer.inflation_radius': 0.40,
                    'local_costmap.local_costmap.inflation_layer.cost_scaling_factor': 3.0,
                    # Controller params - RPP with collision detection disabled - FAST
                    'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                    'FollowPath.desired_linear_vel': 1.8,       # FAST!
                    'FollowPath.lookahead_dist': 1.2,
                    'FollowPath.min_lookahead_dist': 0.5,
                    'FollowPath.max_lookahead_dist': 2.0,
                    'FollowPath.lookahead_time': 1.2,
                    'FollowPath.rotate_to_heading_angular_vel': 3.0,  # Fast turns
                    'FollowPath.transform_tolerance': 0.3,
                    'FollowPath.use_velocity_scaled_lookahead_dist': True,
                    'FollowPath.min_approach_linear_velocity': 0.2,
                    'FollowPath.approach_velocity_scaling_dist': 0.5,
                    # Collision detection - DISABLED (use costmap inflation only)
                    'FollowPath.use_collision_detection': False,
                    'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
                    'FollowPath.use_regulated_linear_velocity_scaling': True,
                    'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
                    'FollowPath.regulated_linear_scaling_min_radius': 0.4,
                    'FollowPath.regulated_linear_scaling_min_speed': 0.3,
                    'FollowPath.use_rotate_to_heading': True,
                    'FollowPath.allow_reversing': False,
                    'FollowPath.rotate_to_heading_min_angle': 0.4,
                    'FollowPath.max_angular_accel': 6.0,
                    'FollowPath.max_robot_pose_search_dist': 10.0
                }
            ],
            remappings=[
                *tf_remaps,
                # Feed controller output into velocity_smoother input
                ('cmd_vel', f'/{robot_name}/cmd_vel_nav'),
            ]
        ),
        
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    # Global costmap config inline
                    'global_costmap.global_costmap.use_sim_time': True,
                    'global_costmap.global_costmap.rolling_window': False,
                    'global_costmap.global_costmap.width': 20,
                    'global_costmap.global_costmap.height': 20,
                    'global_costmap.global_costmap.resolution': 0.05,
                    'global_costmap.global_costmap.origin_x': -10.0,
                    'global_costmap.global_costmap.origin_y': -10.0,
                    'global_costmap.global_costmap.global_frame': 'map',
                    'global_costmap.global_costmap.robot_base_frame': base_frame,
                    'global_costmap.global_costmap.footprint': '[[0.18, 0.08], [0.18, -0.08], [-0.18, -0.08], [-0.18, 0.08]]',
                    'global_costmap.global_costmap.track_unknown_space': True,
                    'global_costmap.global_costmap.plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                    'global_costmap.global_costmap.static_layer.plugin': 'nav2_costmap_2d::StaticLayer',
                    'global_costmap.global_costmap.static_layer.map_subscribe_transient_local': True,
                    'global_costmap.global_costmap.static_layer.map_topic': '/map',
                    'global_costmap.global_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
                    'global_costmap.global_costmap.obstacle_layer.enabled': True,
                    'global_costmap.global_costmap.obstacle_layer.observation_sources': 'scan',
                    'global_costmap.global_costmap.obstacle_layer.footprint_clearing_enabled': True,
                    'global_costmap.global_costmap.obstacle_layer.scan.topic': f'/{robot_name}/scan',
                    'global_costmap.global_costmap.obstacle_layer.scan.sensor_frame': lidar_frame,
                    'global_costmap.global_costmap.obstacle_layer.scan.data_type': 'LaserScan',
                    'global_costmap.global_costmap.obstacle_layer.scan.marking': True,
                    'global_costmap.global_costmap.obstacle_layer.scan.clearing': True,
                    'global_costmap.global_costmap.obstacle_layer.scan.obstacle_max_range': 4.0,
                    'global_costmap.global_costmap.obstacle_layer.scan.obstacle_min_range': 0.25,
                    # Filter balls from costmap (balls are ~0.05m tall)
                    'global_costmap.global_costmap.obstacle_layer.scan.min_obstacle_height': 0.10,
                    'global_costmap.global_costmap.obstacle_layer.scan.max_obstacle_height': 2.5,
                    'global_costmap.global_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
                    'global_costmap.global_costmap.inflation_layer.enabled': True,
                    'global_costmap.global_costmap.inflation_layer.inflation_radius': 0.40,
                    'global_costmap.global_costmap.inflation_layer.cost_scaling_factor': 3.0,
                }
            ],
            remappings=tf_remaps,
        ),
        
        # Behavior server - NO SPIN for Ackermann
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    'robot_base_frame': base_frame,
                    'local_frame': odom_frame,
                    'global_frame': 'map',
                    'behavior_plugins': ['backup', 'drive_on_heading', 'wait'],
                    'backup.plugin': 'nav2_behaviors/BackUp',
                    'drive_on_heading.plugin': 'nav2_behaviors/DriveOnHeading',
                    'wait.plugin': 'nav2_behaviors/Wait',
                }
            ],
            remappings=tf_remaps,
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    'robot_base_frame': base_frame,
                    'global_frame': 'map',
                    'odom_topic': f'/{robot_name}/odom',
                    'default_nav_to_pose_bt_xml': bt_nav_to_pose_xml,
                    'default_nav_through_poses_bt_xml': bt_nav_through_poses_xml,
                }
            ],
            remappings=tf_remaps,
        ),
        
        # Velocity smoother - faster speeds
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                robot_nav2_params,
                {
                    'use_sim_time': True,
                    'odom_topic': f'/{robot_name}/odom',
                    'smoothing_frequency': 30.0,
                    'scale_velocities': False,
                    'feedback': 'OPEN_LOOP',
                    'max_velocity': [2.0, 0.0, 3.5],     # FAST!
                    'min_velocity': [-1.2, 0.0, -3.5],
                    'max_accel': [4.0, 0.0, 5.0],
                    'max_decel': [-4.0, 0.0, -5.0],
                    'deadband_velocity': [0.0, 0.0, 0.0],
                    'velocity_timeout': 1.0,
                }
            ],
            remappings=[
                *tf_remaps,
                ('cmd_vel', f'/{robot_name}/cmd_vel_nav'),
                ('cmd_vel_smoothed', f'/{robot_name}/cmd_vel'),
            ]
        ),
        
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother',
                ],
                'bond_timeout': 0.0,
            }],
        ),
    ]
    nodes.extend(nav2_nodes)
    
    # =========================================================================
    # 7. Ball Perception Node
    # =========================================================================
    ball_perception = Node(
        package='ballvac_ball_collector',
        executable='ball_perception_node',
        name='ball_perception_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': f'/{robot_name}/camera/front_raw',
            'detection_topic': f'/{robot_name}/ball_detections',
            'publish_debug_image': True,
            'min_contour_area': 200,
        }]
    )
    nodes.append(ball_perception)
    
    # =========================================================================
    # 8. Motion Controller (routes cmd_vel_in -> cmd_vel)
    # =========================================================================
    motion_controller = Node(
        package='ballvac_control',
        executable='motion_controller_node',
        name='motion_controller_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'input_topic': 'cmd_vel_in',
            'output_topic': 'cmd_vel',
        }]
    )
    nodes.append(motion_controller)

    # =========================================================================
    # 9. Nav Ball Collector Node (Modified for multi-robot)
    # =========================================================================
    nav_ball_collector = Node(
        package='ballvac_ball_collector',
        executable='nav_ball_collector_node',
        name='nav_ball_collector_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Robot identification
            'robot_id': robot_name,
            'use_fleet_coordinator': True,
            # Topics (namespaced)
            'scan_topic': f'/{robot_name}/scan',
            'detection_topic': f'/{robot_name}/ball_detections',
            'cmd_topic': f'/{robot_name}/cmd_vel_in',
            'odom_topic': f'/{robot_name}/odom',
            # Fleet topics
            'assignment_topic': f'/{robot_name}/assignment',
            'robot_status_topic': '/fleet/robot_status',
            # Services
            'delete_service': '/world/ball_arena/remove',
            'spawn_service': '/world/ball_arena/create',
            # Frames
            'map_frame': 'map',
            'robot_frame': base_frame,
            'camera_frame': camera_frame,
            # Navigation parameters
            'nav_to_approach_distance': 0.7,   # Get closer before reactive control
            'collect_distance_m': 0.45,        # Trigger collection a bit earlier
            # Speed and steering - FAST MOVEMENT
            'approach_speed': 1.8,             # FAST!
            'obstacle_stop_m': 0.7,            # Wall stop distance
            'obstacle_slow_m': 1.8,            # Start slowing
            'obstacle_avoid_m': 3.0,           # Start steering early
            'max_steer': 3.2,                  # Sharp turns for wall avoidance
            'control_rate': 30.0,              # Fast control loop
            'steering_gain': 4.0,              # Responsive steering
            # Ball detection parameters
            'approach_radius_threshold': 80.0,   # Ball pixel size to trigger collection (larger = closer)
            'min_ball_radius': 8.0,              # Detect smaller balls further away
            'max_ball_radius': 250.0,
            'collection_cooldown': 0.3,        # Faster cooldown
            'target_lost_timeout': 15.0,       # INCREASED: Don't cancel navigation too quickly
            'ball_visible_timeout': 1.0,       # INCREASED: More tolerance
            # Camera/ball calibration (match model.sdf)
            'camera_fov_horizontal': 1.658,
            'camera_resolution_width': 720.0,
            'ball_actual_diameter': 0.20,
            # Exploration bounds - STAY AWAY FROM WALLS
            'exploration_min_x': -7.0,         # Reduced bounds
            'exploration_max_x': 7.0,
            'exploration_min_y': -7.0,
            'exploration_max_y': 7.0,
            'explore_with_nav2': True,
            # Ball lifecycle
            'respawn_balls': False,
            # Recovery parameters - faster
            'recover_duration': 1.5,
            'recover_speed': 1.2,
            # Exploration timeout - don't give up too quickly
            'exploration_timeout': 25.0,       # INCREASED: More time for waypoints
        }],
    )
    nodes.append(nav_ball_collector)
    
    return nodes


def generate_launch_description():
    # =========================================================================
    # Package paths
    # =========================================================================
    pkg_ballvac_description = get_package_share_directory('ballvac_description')
    pkg_ballvac_ball_collector = get_package_share_directory('ballvac_ball_collector')
    pkg_ballvac_bringup = get_package_share_directory('ballvac_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # =========================================================================
    # File paths
    # =========================================================================
    world_sdf = os.path.join(pkg_ballvac_description, 'worlds', 'ball_arena.sdf')
    rviz_config = os.path.join(pkg_ballvac_bringup, 'rviz', 'navigation.rviz')
    initial_balls = parse_initial_balls(world_sdf)
    
    # =========================================================================
    # Launch arguments
    # =========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Disable Fast DDS shared memory transport (avoids SHM init_port errors)
    disable_fastdds_shm = SetEnvironmentVariable(
        name='RMW_FASTRTPS_USE_SHM',
        value='0'
    )

    fastdds_profile = os.path.join(pkg_ballvac_ball_collector, 'config', 'fastdds_no_shm.xml')
    use_fastdds_profile = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_profile
    )
    
    # =========================================================================
    # Set use_sim_time globally
    # =========================================================================
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # =========================================================================
    # Robot spawn positions (spread apart to avoid collisions)
    # Triangle formation - 120 degrees apart, 4m from center
    # =========================================================================
    robots = [
        {'name': 'ballvac1', 'x': 0.0, 'y': -4.0, 'yaw': 1.57},     # South position
        {'name': 'ballvac2', 'x': -3.5, 'y': 2.0, 'yaw': -0.52},    # Northwest position
        {'name': 'ballvac3', 'x': 3.5, 'y': 2.0, 'yaw': -2.62},     # Northeast position
    ]
    
    # =========================================================================
    # 1. Gazebo Simulation
    # =========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_sdf}.items(),
    )
    
    # =========================================================================
    # 2. Service Bridges for entity management
    # =========================================================================
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
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # =========================================================================
    # 3. Ball Launcher Node
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
            'auto_launch': False,
            'respawn_on_delete': False,
        }]
    )
    
    # =========================================================================
    # 4. Fleet Coordinator
    # =========================================================================
    fleet_coordinator = Node(
        package='ballvac_ball_collector',
        executable='fleet_coordinator_node',
        name='fleet_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_names': ['ballvac1', 'ballvac2', 'ballvac3'],  # 3 robots
            'color_priority': ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'cyan'],
            'claim_timeout_sec': 20.0,        # Faster reassignment
            'heartbeat_timeout_sec': 8.0,
            'assignment_rate_hz': 4.0,        # Faster assignment checks
            'standoff_distance': 0.5,
            'use_planner_for_cost': False,
            'map_bounds_min_x': -10.0,
            'map_bounds_max_x': 10.0,
            'map_bounds_min_y': -10.0,
            'map_bounds_max_y': 10.0,
            'wall_safety_margin': 1.5,        # Stay away from walls
            'robot_conflict_radius': 2.0,     # Don't target same balls
            'initial_balls': initial_balls,
        }]
    )
    
    # =========================================================================
    # 5. RViz2 Visualization
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
    # Generate robot nodes with delays
    # =========================================================================
    robot_groups = []
    
    for i, robot in enumerate(robots):
        robot_nodes = generate_robot_nodes(
            None,
            robot['name'],
            i,
            robot['x'],
            robot['y'],
            robot['yaw'],
            use_sim_time,
        )
        
        # Wrap in TimerAction for staged startup
        # First robot at 2s, second at 4s, third at 6s
        delay = 2.0 + i * 2.0
        robot_groups.append(
            TimerAction(
                period=delay,
                actions=robot_nodes
            )
        )
    
    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_use_rviz,

        # DDS transport tweaks
        disable_fastdds_shm,
        use_fastdds_profile,
        
        # Global parameter
        set_use_sim_time,
        
        # Log startup info
        LogInfo(msg=['=== Starting Multi-Robot Ball Collection System ===']),
        LogInfo(msg=['Robots: ballvac1, ballvac2, ballvac3 (3-ROBOT FLEET)']),
        LogInfo(msg=['Color Priority: red > blue > green > yellow > orange > purple > cyan']),
        
        # Stage 1: Gazebo simulation
        gazebo,
        
        # Stage 2: Bridges (1s delay)
        TimerAction(
            period=1.0,
            actions=[clock_bridge, delete_bridge, spawn_bridge]
        ),
        
        # Stage 3: Robot groups (staggered)
        *robot_groups,
        
        # Stage 4: Ball launcher (10s delay)
        TimerAction(
            period=10.0,
            actions=[ball_launcher]
        ),
        
        # Stage 5: Fleet coordinator (12s delay)
        TimerAction(
            period=12.0,
            actions=[fleet_coordinator]
        ),
        
        # Stage 6: RViz (2s delay)
        TimerAction(
            period=2.0,
            actions=[rviz]
        ),
    ])
