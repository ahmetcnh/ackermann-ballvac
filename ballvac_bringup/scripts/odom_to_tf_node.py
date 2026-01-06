#!/usr/bin/env python3
"""
Odometry to TF Publisher Node

This node subscribes to odometry messages and publishes the corresponding
TF transform from odom frame to robot base frame.

This is needed because Gazebo's odometry publisher may not publish TF
through the ROS-Gazebo bridge properly.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTfNode(Node):
    """Node that converts odometry messages to TF transforms."""

    def __init__(self):
        super().__init__('odom_to_tf_node')
        
        # Parameters (use_sim_time is automatically declared by ROS 2)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'ballvac')
        
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Odom to TF node started')
        self.get_logger().info(f'  Odom topic: {odom_topic}')
        self.get_logger().info(f'  Publishing TF: {self.odom_frame} -> {self.robot_frame}')

    def odom_callback(self, msg: Odometry):
        """Convert odometry message to TF transform."""
        t = TransformStamped()
        
        # Use the timestamp from odometry message
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.robot_frame
        
        # Copy position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Copy orientation
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
