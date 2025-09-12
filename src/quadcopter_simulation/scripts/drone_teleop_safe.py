#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import sys
import select
import os

# Try to import termios, but handle Windows/non-terminal environments
try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

class DroneTeleopSafe(Node):
    def __init__(self):
        super().__init__('drone_teleop_safe')
        
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
        
        # Control mode
        self.interactive_mode = self.check_interactive_mode()
        
        self.get_logger().info('Drone Teleop Started (Safe Mode)')
        
        if self.interactive_mode:
            self.print_instructions()
            # Timer for keyboard polling
            self.timer = self.create_timer(0.1, self.keyboard_check)
            if HAS_TERMIOS and sys.stdin.isatty():
                self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.get_logger().info('Non-interactive mode detected. Publishing test commands...')
            self.demo_timer = self.create_timer(2.0, self.demo_flight)
            self.demo_state = 0

    def check_interactive_mode(self):
        """Check if we're running in an interactive terminal"""
        return sys.stdin.isatty() and HAS_TERMIOS

    def print_instructions(self):
        print("\n" + "="*50)
        print("DRONE TELEOPERATION CONTROL (SAFE MODE)")
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
        """Get keyboard input in a non-blocking way"""
        if not self.interactive_mode:
            return None
            
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = None
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except:
            return None

    def keyboard_check(self):
        """Check for keyboard input and process commands"""
        if not self.interactive_mode:
            return
            
        key = self.get_key()
        if key is None:
            return
            
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
            return
        elif key == 'l' or key == 'L':
            self.land()
            return
        elif key == ' ':
            self.toggle_arm()
            return
        elif key == 'h' or key == 'H':
            self.hold_position()
            return
        elif key == '1':
            self.linear_speed = max(0.1, self.linear_speed - 0.1)
            self.get_logger().info(f'Linear speed: {self.linear_speed:.1f} m/s')
            return
        elif key == '2':
            self.linear_speed = min(5.0, self.linear_speed + 0.1)
            self.get_logger().info(f'Linear speed: {self.linear_speed:.1f} m/s')
            return
        elif key == '\x1b':  # ESC
            self.emergency_stop()
            return
        elif key == '\x03':  # Ctrl+C
            self.cleanup()
            rclpy.shutdown()
            return
        
        self.cmd_vel_pub.publish(twist)

    def demo_flight(self):
        """Demo flight pattern for non-interactive mode"""
        twist = Twist()
        
        if self.demo_state == 0:
            self.get_logger().info('Demo: Arming drone')
            self.toggle_arm()
        elif self.demo_state == 1:
            self.get_logger().info('Demo: Taking off')
            twist.linear.z = 1.0
        elif self.demo_state == 2:
            self.get_logger().info('Demo: Hovering')
            twist.linear.z = 0.0
        elif self.demo_state == 3:
            self.get_logger().info('Demo: Moving forward')
            twist.linear.x = 0.5
        elif self.demo_state == 4:
            self.get_logger().info('Demo: Rotating')
            twist.angular.z = 0.5
        elif self.demo_state == 5:
            self.get_logger().info('Demo: Landing')
            twist.linear.z = -0.5
        else:
            self.get_logger().info('Demo: Complete')
            twist = Twist()
            
        self.cmd_vel_pub.publish(twist)
        self.demo_state = (self.demo_state + 1) % 7

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

    def cleanup(self):
        """Clean up on exit"""
        if self.interactive_mode and HAS_TERMIOS and hasattr(self, 'settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        # Send stop command
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleopSafe()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()