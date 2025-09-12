#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import sys
import termios
import tty
import threading

class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/flight_mode', 10)
        self.arm_pub = self.create_publisher(Bool, '/arm', 10)
        
        # Movement parameters
        self.linear_speed = 1.0  # m/s
        self.vertical_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # State
        self.armed = False
        self.flight_mode = "MANUAL"
        
        self.get_logger().info('Drone Teleop Started')
        self.print_instructions()
        
        # Start keyboard input thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def print_instructions(self):
        print("\n" + "="*50)
        print("DRONE TELEOPERATION CONTROL")
        print("="*50)
        print("\nControl Keys:")
        print("  W/S     : Forward/Backward")
        print("  A/D     : Left/Right") 
        print("  Q/E     : Rotate Left/Right")
        print("  R/F     : Up/Down")
        print("\nFlight Commands:")
        print("  T       : Takeoff (auto hover at 1m)")
        print("  L       : Land")
        print("  Space   : ARM/DISARM")
        print("  H       : Hold position")
        print("\nSpeed Control:")
        print("  1/2     : Decrease/Increase linear speed")
        print("  3/4     : Decrease/Increase angular speed")
        print("\n  ESC     : Emergency Stop")
        print("  Ctrl+C  : Quit")
        print("="*50 + "\n")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()
                
                if key == 'w' or key == 'W':
                    twist.linear.x = self.linear_speed
                    self.get_logger().info('Moving forward')
                elif key == 's' or key == 'S':
                    twist.linear.x = -self.linear_speed
                    self.get_logger().info('Moving backward')
                elif key == 'a' or key == 'A':
                    twist.linear.y = self.linear_speed
                    self.get_logger().info('Moving left')
                elif key == 'd' or key == 'D':
                    twist.linear.y = -self.linear_speed
                    self.get_logger().info('Moving right')
                elif key == 'q' or key == 'Q':
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('Rotating left')
                elif key == 'e' or key == 'E':
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('Rotating right')
                elif key == 'r' or key == 'R':
                    twist.linear.z = self.vertical_speed
                    self.get_logger().info('Moving up')
                elif key == 'f' or key == 'F':
                    twist.linear.z = -self.vertical_speed
                    self.get_logger().info('Moving down')
                elif key == 't' or key == 'T':
                    self.takeoff()
                elif key == 'l' or key == 'L':
                    self.land()
                elif key == ' ':
                    self.toggle_arm()
                elif key == 'h' or key == 'H':
                    self.hold_position()
                elif key == '1':
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.1f} m/s')
                elif key == '2':
                    self.linear_speed = min(5.0, self.linear_speed + 0.1)
                    self.get_logger().info(f'Linear speed: {self.linear_speed:.1f} m/s')
                elif key == '3':
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    self.get_logger().info(f'Angular speed: {self.angular_speed:.1f} rad/s')
                elif key == '4':
                    self.angular_speed = min(3.0, self.angular_speed + 0.1)
                    self.get_logger().info(f'Angular speed: {self.angular_speed:.1f} rad/s')
                elif key == '\x1b':  # ESC
                    self.emergency_stop()
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    # Stop if no valid key
                    twist = Twist()
                
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

    def takeoff(self):
        if self.armed:
            twist = Twist()
            twist.linear.z = 1.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('TAKEOFF commanded')
        else:
            self.get_logger().warn('Cannot takeoff - drone not armed')

    def land(self):
        twist = Twist()
        twist.linear.z = -0.5
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('LANDING commanded')

    def toggle_arm(self):
        self.armed = not self.armed
        arm_msg = Bool()
        arm_msg.data = self.armed
        self.arm_pub.publish(arm_msg)
        status = "ARMED" if self.armed else "DISARMED"
        self.get_logger().info(f'Drone {status}')

    def hold_position(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('HOLD POSITION commanded')

    def emergency_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.armed = False
        arm_msg = Bool()
        arm_msg.data = False
        self.arm_pub.publish(arm_msg)
        self.get_logger().warn('EMERGENCY STOP - Drone disarmed')

def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()