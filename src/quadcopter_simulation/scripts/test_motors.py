#!/usr/bin/env python3
"""
Simple test to verify drone motor command functionality
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class DroneMotorTest(Node):
    def __init__(self):
        super().__init__('drone_motor_test')
        
        # Create publishers for individual motor speeds
        self.motor_pubs = []
        for i in range(4):
            pub = self.create_publisher(Float64, f'/motor_speed_{i}', 10)
            self.motor_pubs.append(pub)
        
        # Wait for connections
        time.sleep(2)
        
        self.get_logger().info("Motor test node initialized")
        
    def test_motors(self):
        """Send test commands to motors"""
        self.get_logger().info("Sending test motor commands...")
        
        # Send modest motor speeds (typical hover is around 1500)
        test_speed = 1200.0
        
        for i, pub in enumerate(self.motor_pubs):
            msg = Float64()
            msg.data = test_speed
            pub.publish(msg)
            self.get_logger().info(f"Motor {i}: {test_speed}")
        
        self.get_logger().info("Test commands sent!")

def main():
    rclpy.init()
    test_node = DroneMotorTest()
    
    try:
        # Send test commands
        test_node.test_motors()
        
        # Keep running for a moment
        rclpy.spin_once(test_node, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()