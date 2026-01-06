#!/usr/bin/env python3
"""
Simple test script to verify Ackermann steering via geometry_msgs/Twist.

Usage:
    ros2 run ballvac_ball_collector test_steering.py

Expected behavior:
    1. Robot drives forward for 2s
    2. Robot turns left (positive angular.z) for 3s
    3. Robot turns right (negative angular.z) for 3s
    4. Stop

If wheels visibly steer left/right, the command interface is working.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class SteeringTester(Node):
    def __init__(self):
        super().__init__('steering_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('=== STEERING TEST STARTING ===')
        self.get_logger().info('Watch the front wheels for steering movement!')
        
    def send_cmd(self, linear_x: float, angular_z: float, duration: float, description: str):
        """Send velocity command for specified duration."""
        self.get_logger().info(f'{description}: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        start_time = time.time()
        rate = self.create_rate(20)  # 20 Hz
        while time.time() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()
            
    def run_test(self):
        """Run steering test sequence."""
        # Wait for simulation to be ready
        time.sleep(1.0)
        
        # Test 1: Forward
        self.send_cmd(0.3, 0.0, 2.0, "TEST 1: Forward (straight)")
        
        # Test 2: Turn left (positive angular.z)
        self.send_cmd(0.3, 0.8, 3.0, "TEST 2: Forward + Turn LEFT (angular.z > 0)")
        
        # Test 3: Turn right (negative angular.z)
        self.send_cmd(0.3, -0.8, 3.0, "TEST 3: Forward + Turn RIGHT (angular.z < 0)")
        
        # Test 4: Sharp left turn (stationary turning)
        self.send_cmd(0.1, 1.5, 2.0, "TEST 4: Slow + Sharp LEFT turn")
        
        # Stop
        self.send_cmd(0.0, 0.0, 0.5, "STOP")
        
        self.get_logger().info('=== STEERING TEST COMPLETE ===')
        self.get_logger().info('If front wheels visibly steered left/right, steering works!')
        self.get_logger().info('If wheels only pointed straight, check Ackermann plugin config.')


def main():
    rclpy.init()
    node = SteeringTester()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
