import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ballvac_bringup')
    pkg_project_description = get_package_share_directory('ballvac_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    

    # Path to the SDF file
    sdf_file = os.path.join(pkg_project_description, 'models', 'ballvac', 'model.sdf')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_description,
            'worlds',
            'ballvac_world.sdf'
        ])}.items(),
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'ballvac.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
    # Bridge ROS topics and Gazebo messages for establishing communication
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    # Service bridge for entity deletion (needed for ball collection)
    # Bridges the Gazebo /world/my_world/remove service to ROS 2
    delete_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/my_world/remove@ros_gz_interfaces/srv/DeleteEntity'
        ],
        output='screen'
    )
    
    # Service bridge for entity spawning (needed for ball respawning)
    # Bridges the Gazebo /world/my_world/create service to ROS 2
    spawn_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/my_world/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        output='screen'
    )
    
    # Spawn the robot directly from SDF file (not from topic, since SDF != URDF)
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', sdf_file,
            '-name', 'ballvac',
            '-allow_renaming', 'true',
            '-z', '0.35'
        ]
    )
    
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        delete_bridge,
        spawn_bridge,
        gz_spawn_entity,
        rviz
    ])