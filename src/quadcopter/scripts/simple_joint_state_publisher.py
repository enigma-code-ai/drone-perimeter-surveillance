#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class SimpleJointStatePublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        
        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create timer to publish joint states at 10Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Joint names for the quadcopter propellers
        self.joint_names = ['propeller_1', 'propeller_2', 'propeller_3', 'propeller_4']
        
        self.get_logger().info('Simple joint state publisher started')

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Set joint names and positions (all at 0.0 for now)
        msg.name = self.joint_names
        msg.position = [0.0, 0.0, 0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()