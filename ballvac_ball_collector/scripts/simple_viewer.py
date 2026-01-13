#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import argparse
import threading

class SimpleViewer(Node):
    def __init__(self, robot_name="ballvac1"):
        super().__init__('simple_viewer')
        self.bridge = CvBridge()
        
        prefix = f"/{robot_name}" if robot_name else ""
        
        self.debug_topic = f"{prefix}/ball_perception/debug_image"
        self.mask_topic = f"{prefix}/ball_perception/debug_mask"

        # Subscribe to both topics
        self.create_subscription(Image, self.debug_topic, self.debug_callback, 10)
        self.create_subscription(Image, self.mask_topic, self.mask_callback, 10)

        self.get_logger().info(f"SimpleViewer started for {robot_name}")
        self.get_logger().info(f"Subscribed to:\n  - {self.debug_topic}\n  - {self.mask_topic}")
        self.get_logger().info("Waiting for images...")

        # Store latest images
        self.latest_debug_img = None
        self.latest_mask_img = None

    def debug_callback(self, msg):
        try:
            self.latest_debug_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting debug image: {e}")

    def mask_callback(self, msg):
        try:
            self.latest_mask_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f"Error converting mask: {e}")

def main():
    parser = argparse.ArgumentParser(description='Simple Viewer for BallVac')
    parser.add_argument('robot_name', nargs='?', default='ballvac1', help='Name of the robot (default: ballvac1)')
    args = parser.parse_args()

    rclpy.init()
    node = SimpleViewer(robot_name=args.robot_name)
    
    try:
        while rclpy.ok():
            # Process callbacks (non-blocking)
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Display images if available
            if node.latest_debug_img is not None:
                cv2.imshow("Detections (Colored)", node.latest_debug_img)
            
            if node.latest_mask_img is not None:
                cv2.imshow("HSV Pipeline Mask", node.latest_mask_img)
            
            # Process GUI events - CRITICAL for moving windows
            key = cv2.waitKey(10) & 0xFF
            if key == 27 or key == ord('q'):  # ESC or q to quit
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
