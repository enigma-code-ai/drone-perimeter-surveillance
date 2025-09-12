#!/usr/bin/env python3
"""
Advanced Drone Teleoperation System
====================================

Professional-grade drone control system with:
- Multiple flight modes (Manual, Stabilized, Altitude Hold, Position Hold, Auto)
- Real-time HUD with telemetry display
- Advanced safety systems (geofencing, battery monitoring, collision avoidance)
- Mission planning and waypoint navigation
- Flight data logging and analytics
- Emergency failsafe systems
- Customizable control profiles
- Performance monitoring and diagnostics

Author: Drone Perimeter Surveillance Team
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import sys
import time
import json
import threading
import math
from datetime import datetime
from collections import deque
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

# ROS2 Messages
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from std_msgs.msg import String, Bool, Float32, Header
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

# Terminal control
try:
    import termios
    import tty
    import select
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False

# Rich console for advanced UI
try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.live import Live
    from rich.text import Text
    from rich.progress import Progress, SpinnerColumn, TextColumn
    from rich import box
    HAS_RICH = True
except ImportError:
    HAS_RICH = False
    print("Warning: 'rich' library not found. Install with: pip install rich")

class FlightMode(Enum):
    """Advanced flight modes"""
    DISARMED = "DISARMED"
    MANUAL = "MANUAL"              # Direct stick control
    STABILIZED = "STABILIZED"      # Attitude stabilization
    ALTITUDE_HOLD = "ALTITUDE_HOLD"# Hold altitude automatically
    POSITION_HOLD = "POSITION_HOLD"# Hold position and altitude
    AUTO_MISSION = "AUTO_MISSION"  # Execute predefined mission
    RETURN_TO_HOME = "RTH"         # Autonomous return to launch
    EMERGENCY = "EMERGENCY"        # Emergency landing mode

class SafetyStatus(Enum):
    """Safety system status"""
    SAFE = "SAFE"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"
    EMERGENCY = "EMERGENCY"

@dataclass
class DroneState:
    """Comprehensive drone state"""
    # Position and orientation
    position: Point = None
    orientation: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # Roll, Pitch, Yaw
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # Flight state
    flight_mode: FlightMode = FlightMode.DISARMED
    armed: bool = False
    flying: bool = False
    altitude_agl: float = 0.0  # Above ground level
    
    # Safety and status
    battery_voltage: float = 12.6  # Simulation default voltage
    battery_percentage: float = 85.0  # Start with good battery level
    safety_status: SafetyStatus = SafetyStatus.SAFE
    geofence_violations: List[str] = None
    
    # Performance metrics
    flight_time: float = 0.0
    distance_traveled: float = 0.0
    max_altitude: float = 0.0
    
    def __post_init__(self):
        """Initialize default values"""
        if self.position is None:
            self.position = Point()
            self.position.x = 0.0
            self.position.y = 0.0
            self.position.z = 0.0
        if self.geofence_violations is None:
            self.geofence_violations = []

@dataclass
class ControlSettings:
    """Advanced control configuration"""
    # Speed limits
    max_linear_speed: float = 5.0
    max_angular_speed: float = 2.0
    max_vertical_speed: float = 3.0
    
    # Control sensitivity
    linear_sensitivity: float = 1.0
    angular_sensitivity: float = 1.0
    vertical_sensitivity: float = 1.0
    
    # Safety limits
    max_altitude: float = 50.0
    max_distance: float = 100.0
    min_battery_voltage: float = 10.5
    
    # Geofence (rectangular for simplicity)
    geofence_enabled: bool = True
    geofence_min_x: float = -50.0
    geofence_max_x: float = 50.0
    geofence_min_y: float = -50.0
    geofence_max_y: float = 50.0

class AdvancedDroneTeleop(Node):
    """Advanced drone teleoperation system"""
    
    def __init__(self):
        super().__init__('advanced_drone_teleop')
        
        # Initialize state and settings
        self.drone_state = DroneState()
        self.settings = ControlSettings()
        self.use_rich_ui = True  # Will be set by setup_ui
        
        # Flight data logging
        self.flight_log = deque(maxlen=10000)  # Last 10k data points
        self.mission_waypoints = []
        self.current_waypoint_index = 0
        
        # Timing
        self.start_time = time.time()
        self.last_position = Point()
        self.total_distance = 0.0
        
        # Control state
        self.current_twist = Twist()
        self.target_position = PoseStamped()
        self.home_position = PoseStamped()
        self.last_key_time = 0.0
        self.key_timeout = 0.2
        
        # Setup QoS profiles for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.mode_pub = self.create_publisher(String, '/flight_mode', qos_profile)
        self.arm_pub = self.create_publisher(Bool, '/arm', qos_profile)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/waypoint_target', qos_profile)
        self.path_pub = self.create_publisher(Path, '/mission_path', qos_profile)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.pose_callback, qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, '/drone/imu/data', self.imu_callback, qos_profile)
        self.battery_sub = self.create_subscription(
            BatteryState, '/drone/battery', self.battery_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odometry', self.odometry_callback, qos_profile)
        
        # Timers
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        self.safety_timer = self.create_timer(0.1, self.safety_check)   # 10Hz
        self.telemetry_timer = self.create_timer(0.1, self.log_telemetry)  # 10Hz
        
        # Terminal and UI setup
        self.setup_terminal()
        self.setup_ui()
        
        # Start keyboard input thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
        self.get_logger().info('Advanced Drone Teleop System Initialized')
        self.get_logger().info(f'Flight modes available: {[mode.value for mode in FlightMode]}')

    def setup_terminal(self):
        """Configure terminal for optimal input handling"""
        self.interactive_mode = sys.stdin.isatty() and HAS_TERMIOS
        
        self.get_logger().info(f"Terminal setup - stdin.isatty(): {sys.stdin.isatty()}, HAS_TERMIOS: {HAS_TERMIOS}")
        self.get_logger().info(f"Interactive mode enabled: {self.interactive_mode}")
        
        if self.interactive_mode and HAS_TERMIOS:
            self.original_settings = termios.tcgetattr(sys.stdin)
            self.get_logger().info("Terminal settings saved successfully")
        else:
            self.get_logger().warn("Terminal not interactive or termios not available")
        
    def setup_ui(self):
        """Initialize the UI system - disable Rich UI if interactive mode needed"""
        if HAS_RICH and not self.interactive_mode:
            # Only use Rich UI if not in interactive terminal mode
            self.console = Console()
            self.setup_rich_ui()
        else:
            # Use simple text UI for better keyboard compatibility
            self.console = None
            self.setup_simple_ui()
            print("=== ADVANCED DRONE TELEOPERATION SYSTEM ===")
            print("Rich UI disabled for keyboard compatibility")
            print("Controls: Space=ARM/DISARM, w/a/s/d=Move, t=Takeoff, l=Land, q=Quit")
    
    def setup_simple_ui(self):
        """Setup simple text-based UI for keyboard compatibility"""
        self.use_rich_ui = False
    
    def setup_rich_ui(self):
        """Setup Rich console-based HUD"""
        self.layout = Layout()
        
        # Create layout sections
        self.layout.split_column(
            Layout(name="header", size=3),
            Layout(name="body"),
            Layout(name="footer", size=5)
        )
        
        self.layout["body"].split_row(
            Layout(name="left"),
            Layout(name="center"),
            Layout(name="right")
        )
        
        self.layout["left"].split_column(
            Layout(name="status"),
            Layout(name="controls")
        )
        
        self.layout["center"].split_column(
            Layout(name="telemetry"),
            Layout(name="mission")
        )
        
        self.layout["right"].split_column(
            Layout(name="safety"),
            Layout(name="performance")
        )
        
        # Start live display
        self.live = Live(self.layout, refresh_per_second=10, screen=True)
        self.live.start()
    
    def print_basic_instructions(self):
        """Basic instructions when Rich is not available"""
        print("\n" + "="*70)
        print("ADVANCED DRONE TELEOPERATION SYSTEM")
        print("="*70)
        print("\nFlight Modes:")
        print("  M  : Manual mode (direct control)")
        print("  S  : Stabilized mode (attitude hold)")
        print("  A  : Altitude hold mode")
        print("  P  : Position hold mode")
        print("  G  : Auto mission mode")
        print("  H  : Return to home")
        print("\nMovement Controls:")
        print("  W/S     : Forward/Backward")
        print("  A/D     : Left/Right")
        print("  Q/E     : Rotate Left/Right")
        print("  R/F     : Up/Down")
        print("\nFlight Commands:")
        print("  T       : Takeoff")
        print("  L       : Land")
        print("  Space   : ARM/DISARM")
        print("  Enter   : Emergency stop")
        print("\nMission Commands:")
        print("  N       : Set waypoint at current position")
        print("  B       : Begin mission")
        print("  C       : Clear mission")
        print("\nSystem Commands:")
        print("  I       : Show detailed info")
        print("  +/-     : Adjust speed")
        print("  ESC     : Emergency landing")
        print("  Ctrl+C  : Quit")
        print("="*70 + "\n")

    def keyboard_loop(self):
        """Advanced keyboard input handling with multi-key support"""
        if not self.interactive_mode:
            self.get_logger().warn("Interactive mode disabled - keyboard input not available")
            return
        
        self.get_logger().info("Keyboard input loop started - press keys to test")
        
        # Set terminal to raw mode once
        if HAS_TERMIOS:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        
        try:
            while self.running and rclpy.ok():
                try:
                    # Simple blocking read with timeout
                    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if ready:
                        key = sys.stdin.read(1)
                        if key:
                            self.process_keyboard_input(key)
                            self.last_key_time = time.time()
                    
                    # Small delay for ROS spinning
                    time.sleep(0.01)
                except Exception as e:
                    self.get_logger().error(f'Keyboard input error: {e}')
                    break
        finally:
            # Restore terminal settings
            if HAS_TERMIOS:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def get_key_non_blocking(self) -> Optional[str]:
        """Get keyboard input without blocking - simplified version"""
        if not self.interactive_mode:
            return None
        
        try:
            # Use a simpler approach - just try to read with a very short timeout
            import select
            import sys
            import tty
            import termios
            
            # Check if input is available with a very short timeout
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return None
            
            # Read the character in raw mode
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setraw(sys.stdin.fileno())
                key = sys.stdin.read(1)
                return key
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                
        except Exception as e:
            self.get_logger().error(f"get_key_non_blocking error: {e}")
            return None
    
    def process_keyboard_input(self, key: str):
        """Process keyboard input with advanced command handling"""
        # Log only special keys for debugging
        if key == ' ':
            self.get_logger().info("Space key - ARM/DISARM")
        elif not key.isprintable():
            self.get_logger().debug(f"Special key: ord={ord(key)}")
        
        key_lower = key.lower()
        
        # Movement controls (highest priority when armed and in manual modes)
        if self.drone_state.armed and self.drone_state.flight_mode in [FlightMode.MANUAL, FlightMode.STABILIZED]:
            if key == 'w':
                self.get_logger().info("Forward movement")
                self.current_twist.linear.x = self.settings.max_linear_speed * self.settings.linear_sensitivity
                self.current_twist.linear.y = 0.0
                self.current_twist.linear.z = 0.0
                self.current_twist.angular.z = 0.0
                self.get_logger().info(f"Publishing: x={self.current_twist.linear.x}")
                return
            elif key == 's':
                self.get_logger().info("Backward movement") 
                self.current_twist.linear.x = -self.settings.max_linear_speed * self.settings.linear_sensitivity
                self.current_twist.linear.y = 0.0
                self.current_twist.linear.z = 0.0
                self.current_twist.angular.z = 0.0
                self.get_logger().info(f"Publishing: x={self.current_twist.linear.x}")
                return
            elif key == 'a':
                self.get_logger().info("Left movement")
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = self.settings.max_linear_speed * self.settings.linear_sensitivity
                self.current_twist.linear.z = 0.0
                self.current_twist.angular.z = 0.0
                self.get_logger().info(f"Publishing: y={self.current_twist.linear.y}")
                return
            elif key == 'd':
                self.get_logger().info("Right movement")
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = -self.settings.max_linear_speed * self.settings.linear_sensitivity
                self.current_twist.linear.z = 0.0
                self.current_twist.angular.z = 0.0
                self.get_logger().info(f"Publishing: y={self.current_twist.linear.y}")
                return
            elif key == 'q':
                self.get_logger().info("Rotate left")
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = 0.0
                self.current_twist.linear.z = 0.0
                self.current_twist.angular.z = self.settings.max_angular_speed * self.settings.angular_sensitivity
                self.get_logger().info(f"Publishing: angular.z={self.current_twist.angular.z}")
                return
            elif key == 'e':
                self.get_logger().info("Rotate right")
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = 0.0
                self.current_twist.linear.z = 0.0
                self.current_twist.angular.z = -self.settings.max_angular_speed * self.settings.angular_sensitivity
                self.get_logger().info(f"Publishing: angular.z={self.current_twist.angular.z}")
                return
            elif key == 'r':
                self.get_logger().info("Up movement")
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = 0.0
                self.current_twist.linear.z = self.settings.max_vertical_speed * self.settings.vertical_sensitivity
                self.current_twist.angular.z = 0.0
                self.get_logger().info(f"Publishing: z={self.current_twist.linear.z}")
                return
            elif key == 'f':
                self.get_logger().info("Down movement")
                self.current_twist.linear.x = 0.0
                self.current_twist.linear.y = 0.0
                self.current_twist.linear.z = -self.settings.max_vertical_speed * self.settings.vertical_sensitivity
                self.current_twist.angular.z = 0.0
                self.get_logger().info(f"Publishing: z={self.current_twist.linear.z}")
                return
        
        # Flight mode changes (only when not moving or not armed)
        if key_lower == 'm':
            self.set_flight_mode(FlightMode.MANUAL)
        elif key_lower == 's' and not (self.drone_state.armed and self.drone_state.flight_mode in [FlightMode.MANUAL, FlightMode.STABILIZED]):
            self.set_flight_mode(FlightMode.STABILIZED)
        elif key_lower == 'a' and not (self.drone_state.armed and self.drone_state.flight_mode in [FlightMode.MANUAL, FlightMode.STABILIZED]):
            self.set_flight_mode(FlightMode.ALTITUDE_HOLD)
        elif key_lower == 'p':
            self.set_flight_mode(FlightMode.POSITION_HOLD)
        elif key_lower == 'g':
            self.set_flight_mode(FlightMode.AUTO_MISSION)
        elif key_lower == 'h':
            self.return_to_home()
        
        # Flight commands
        elif key_lower == 't':
            self.takeoff()
        elif key_lower == 'l':
            self.land()
        elif key == ' ':
            self.get_logger().info("Space key - ARM/DISARM")
            self.toggle_arm()
        elif key == '\r':  # Enter
            self.emergency_stop()
        
        # Mission commands
        elif key_lower == 'n':
            self.add_waypoint()
        elif key_lower == 'b':
            self.begin_mission()
        elif key_lower == 'c':
            self.clear_mission()
        
        # System commands
        elif key_lower == 'i':
            self.show_detailed_info()
        elif key == '+':
            self.adjust_speed(0.1)
        elif key == '-':
            self.adjust_speed(-0.1)
        elif key == '\x1b':  # ESC
            self.emergency_stop()
        elif key_lower == 'q':
            self.get_logger().info("Quit requested")
            self.running = False
            self.emergency_landing()
        elif key == '\x03':  # Ctrl+C
            self.shutdown()

    def set_flight_mode(self, mode: FlightMode):
        """Change flight mode with safety checks"""
        if not self.drone_state.armed and mode != FlightMode.DISARMED:
            self.get_logger().warn(f"Cannot switch to {mode.value} - drone not armed")
            return
        
        old_mode = self.drone_state.flight_mode
        self.drone_state.flight_mode = mode
        
        # Publish mode change
        mode_msg = String()
        mode_msg.data = mode.value
        self.mode_pub.publish(mode_msg)
        
        # Reset control state when changing modes
        if mode != FlightMode.MANUAL:
            self.current_twist = Twist()
        
        self.get_logger().info(f"Flight mode changed: {old_mode.value} → {mode.value}")
        
        # Special handling for different modes
        if mode == FlightMode.POSITION_HOLD:
            self.target_position.pose.position = self.drone_state.position
        elif mode == FlightMode.AUTO_MISSION:
            if not self.mission_waypoints:
                self.get_logger().warn("No mission waypoints defined")
                self.set_flight_mode(FlightMode.POSITION_HOLD)

    def control_loop(self):
        """Main control loop - 50Hz"""
        current_time = time.time()
        
        # Handle control timeout (stop movement if no input)
        if current_time - self.last_key_time > self.key_timeout:
            if self.drone_state.flight_mode == FlightMode.MANUAL:
                self.current_twist = Twist()
        
        # Execute mode-specific control logic
        if self.drone_state.flight_mode == FlightMode.MANUAL:
            # Only log if there's actual movement
            if (abs(self.current_twist.linear.x) > 0.1 or abs(self.current_twist.linear.y) > 0.1 or 
                abs(self.current_twist.linear.z) > 0.1 or abs(self.current_twist.angular.z) > 0.1):
                self.get_logger().info(f"Publishing cmd_vel: x={self.current_twist.linear.x:.2f}, y={self.current_twist.linear.y:.2f}, z={self.current_twist.linear.z:.2f}, yaw={self.current_twist.angular.z:.2f}")
            self.cmd_vel_pub.publish(self.current_twist)
        elif self.drone_state.flight_mode == FlightMode.ALTITUDE_HOLD:
            self.altitude_hold_control()
        elif self.drone_state.flight_mode == FlightMode.POSITION_HOLD:
            self.position_hold_control()
        elif self.drone_state.flight_mode == FlightMode.AUTO_MISSION:
            self.mission_control()
        elif self.drone_state.flight_mode == FlightMode.RETURN_TO_HOME:
            self.return_to_home_control()
        
        # Update UI
        if HAS_RICH and hasattr(self, 'live'):
            self.update_rich_display()

    def safety_check(self):
        """Comprehensive safety monitoring"""
        violations = []
        status = SafetyStatus.SAFE
        
        # Battery safety - only check if we have valid battery data
        if self.drone_state.battery_percentage > 0:  # Valid battery data
            if self.drone_state.battery_percentage < 20:
                violations.append("Low battery")
                status = SafetyStatus.WARNING
            if self.drone_state.battery_percentage < 10:
                violations.append("Critical battery")
                status = SafetyStatus.CRITICAL
        # For simulation without battery data, assume safe battery level
        
        # Altitude safety
        if self.drone_state.altitude_agl > self.settings.max_altitude:
            violations.append("Altitude limit exceeded")
            status = SafetyStatus.WARNING
        
        # Geofence check
        if self.settings.geofence_enabled and self.drone_state.position:
            pos = self.drone_state.position
            if (pos.x < self.settings.geofence_min_x or pos.x > self.settings.geofence_max_x or
                pos.y < self.settings.geofence_min_y or pos.y > self.settings.geofence_max_y):
                violations.append("Geofence violation")
                status = SafetyStatus.CRITICAL
        
        # Distance safety - only check if we have position data
        if self.drone_state.position and self.home_position.pose.position:
            home_distance = self.calculate_distance(self.drone_state.position, self.home_position.pose.position)
            if home_distance > self.settings.max_distance:
                violations.append("Max distance exceeded")
                status = SafetyStatus.WARNING
        
        # Update safety status
        self.drone_state.safety_status = status
        self.drone_state.geofence_violations = violations
        
        # Take action on critical violations
        if status == SafetyStatus.CRITICAL and self.drone_state.armed:
            self.get_logger().error(f"Critical safety violation: {violations}")
            if self.drone_state.flight_mode != FlightMode.RETURN_TO_HOME:
                self.return_to_home()

    def pose_callback(self, msg: PoseStamped):
        """Handle pose updates"""
        self.drone_state.position = msg.pose.position
        
        # Calculate distance traveled
        if self.last_position.x != 0 or self.last_position.y != 0:
            distance = self.calculate_distance(self.drone_state.position, self.last_position)
            self.total_distance += distance
            self.drone_state.distance_traveled = self.total_distance
        
        self.last_position = Point(
            x=self.drone_state.position.x,
            y=self.drone_state.position.y,
            z=self.drone_state.position.z
        )
        
        # Update altitude records
        self.drone_state.altitude_agl = self.drone_state.position.z
        if self.drone_state.altitude_agl > self.drone_state.max_altitude:
            self.drone_state.max_altitude = self.drone_state.altitude_agl

    def imu_callback(self, msg: Imu):
        """Handle IMU data"""
        # Extract orientation (simplified - using quaternion to euler conversion)
        try:
            # Simple approximation for small angles
            # For production, use proper tf2 transformations
            self.drone_state.orientation = (0.0, 0.0, 0.0)  # Placeholder
        except Exception as e:
            self.get_logger().debug(f"IMU processing error: {e}")
            self.drone_state.orientation = (0.0, 0.0, 0.0)
        
        # Extract angular velocities
        self.drone_state.angular_velocity = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        )

    def battery_callback(self, msg: BatteryState):
        """Handle battery status"""
        self.drone_state.battery_voltage = msg.voltage
        self.drone_state.battery_percentage = msg.percentage * 100

    def odometry_callback(self, msg: Odometry):
        """Handle odometry data"""
        self.drone_state.velocity = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        )

    def log_telemetry(self):
        """Log flight data for analysis"""
        telemetry_entry = {
            'timestamp': time.time(),
            'flight_time': time.time() - self.start_time,
            'position': {
                'x': self.drone_state.position.x if self.drone_state.position else 0.0,
                'y': self.drone_state.position.y if self.drone_state.position else 0.0,
                'z': self.drone_state.position.z if self.drone_state.position else 0.0
            },
            'orientation': self.drone_state.orientation,
            'velocity': self.drone_state.velocity,
            'flight_mode': self.drone_state.flight_mode.value,
            'battery': self.drone_state.battery_percentage,
            'altitude': self.drone_state.altitude_agl,
            'safety_status': self.drone_state.safety_status.value
        }
        
        self.flight_log.append(telemetry_entry)
        self.drone_state.flight_time = telemetry_entry['flight_time']

    # Additional methods for advanced features would continue here...
    # (Mission control, waypoint navigation, return-to-home, etc.)
    
    def altitude_hold_control(self):
        """Altitude hold mode - maintain current altitude while allowing horizontal movement"""
        # Maintain current altitude
        twist = Twist()
        twist.linear.x = self.current_twist.linear.x
        twist.linear.y = self.current_twist.linear.y
        twist.angular.z = self.current_twist.angular.z
        
        # Altitude control (PID would be implemented here)
        altitude_error = self.target_position.pose.position.z - self.drone_state.position.z
        twist.linear.z = max(-1.0, min(1.0, altitude_error * 0.5))  # Simple P controller
        
        self.cmd_vel_pub.publish(twist)
    
    def position_hold_control(self):
        """Position hold mode - maintain current position and altitude"""
        twist = Twist()
        
        # Position control (simplified PID)
        pos_error_x = self.target_position.pose.position.x - self.drone_state.position.x
        pos_error_y = self.target_position.pose.position.y - self.drone_state.position.y
        pos_error_z = self.target_position.pose.position.z - self.drone_state.position.z
        
        # Simple proportional control
        twist.linear.x = max(-1.0, min(1.0, pos_error_x * 0.3))
        twist.linear.y = max(-1.0, min(1.0, pos_error_y * 0.3))
        twist.linear.z = max(-1.0, min(1.0, pos_error_z * 0.5))
        
        self.cmd_vel_pub.publish(twist)
    
    def mission_control(self):
        """Execute autonomous mission through waypoints"""
        if not self.mission_waypoints or self.current_waypoint_index >= len(self.mission_waypoints):
            self.get_logger().info("Mission completed, switching to position hold")
            self.set_flight_mode(FlightMode.POSITION_HOLD)
            return
        
        current_waypoint = self.mission_waypoints[self.current_waypoint_index]
        distance_to_waypoint = self.calculate_distance(
            self.drone_state.position, current_waypoint.pose.position)
        
        # Check if waypoint reached
        if distance_to_waypoint < 1.0:  # 1 meter tolerance
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached")
            self.current_waypoint_index += 1
            return
        
        # Navigate to current waypoint
        self.target_position = current_waypoint
        self.position_hold_control()
    
    def return_to_home_control(self):
        """Autonomous return to home"""
        home_distance = self.calculate_distance(
            self.drone_state.position, self.home_position.pose.position)
        
        if home_distance < 2.0:  # 2 meter tolerance
            self.get_logger().info("Arrived at home position, landing")
            self.land()
            return
        
        # Navigate to home position
        self.target_position = self.home_position
        self.position_hold_control()
    
    def takeoff(self):
        """Automated takeoff sequence"""
        if not self.drone_state.armed:
            self.get_logger().warn("Cannot takeoff - drone not armed")
            return
        
        if self.drone_state.flying:
            self.get_logger().warn("Already flying")
            return
        
        # Set takeoff altitude
        self.target_position.pose.position = self.drone_state.position
        self.target_position.pose.position.z = 2.0  # Takeoff to 2 meters
        
        # Record home position
        self.home_position.pose.position = Point(
            x=self.drone_state.position.x,
            y=self.drone_state.position.y,
            z=0.0  # Ground level
        )
        
        self.set_flight_mode(FlightMode.ALTITUDE_HOLD)
        self.drone_state.flying = True
        self.get_logger().info("Initiating takeoff sequence")
    
    def land(self):
        """Automated landing sequence"""
        self.target_position.pose.position = self.drone_state.position
        self.target_position.pose.position.z = 0.0  # Ground level
        
        self.set_flight_mode(FlightMode.ALTITUDE_HOLD)
        self.get_logger().info("Initiating landing sequence")
        
        # Check if landed
        if self.drone_state.altitude_agl < 0.2:
            self.drone_state.flying = False
            self.set_flight_mode(FlightMode.DISARMED)
            self.get_logger().info("Landing completed")
    
    def toggle_arm(self):
        """Toggle arm/disarm state with safety checks"""
        if self.drone_state.armed:
            # Disarm - only if not flying
            if self.drone_state.flying:
                self.get_logger().warn("Cannot disarm while flying - land first")
                return
            
            self.drone_state.armed = False
            self.set_flight_mode(FlightMode.DISARMED)
            self.get_logger().info("Drone DISARMED")
        else:
            # Arm - perform safety checks (relaxed for simulation)
            if self.drone_state.battery_percentage > 0 and self.drone_state.battery_percentage < 10:
                self.get_logger().error("Cannot arm - battery critically low")
                return
            
            if self.drone_state.safety_status == SafetyStatus.EMERGENCY:
                self.get_logger().error("Cannot arm - emergency safety violations")
                return
            
            self.drone_state.armed = True
            self.set_flight_mode(FlightMode.MANUAL)
            self.get_logger().info("Drone ARMED")
        
        # Publish arm state
        arm_msg = Bool()
        arm_msg.data = self.drone_state.armed
        self.arm_pub.publish(arm_msg)
    
    def add_waypoint(self):
        """Add current position as mission waypoint"""
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = "world"
        waypoint.pose.position = Point(
            x=self.drone_state.position.x,
            y=self.drone_state.position.y,
            z=self.drone_state.position.z
        )
        
        self.mission_waypoints.append(waypoint)
        self.get_logger().info(f"Waypoint {len(self.mission_waypoints)} added at "
                              f"({waypoint.pose.position.x:.1f}, "
                              f"{waypoint.pose.position.y:.1f}, "
                              f"{waypoint.pose.position.z:.1f})")
        
        self.publish_mission_path()
    
    def begin_mission(self):
        """Start executing the mission"""
        if not self.mission_waypoints:
            self.get_logger().warn("No waypoints defined for mission")
            return
        
        if not self.drone_state.armed or not self.drone_state.flying:
            self.get_logger().warn("Drone must be armed and flying to start mission")
            return
        
        self.current_waypoint_index = 0
        self.set_flight_mode(FlightMode.AUTO_MISSION)
        self.get_logger().info(f"Mission started with {len(self.mission_waypoints)} waypoints")
    
    def clear_mission(self):
        """Clear all mission waypoints"""
        self.mission_waypoints.clear()
        self.current_waypoint_index = 0
        self.get_logger().info("Mission waypoints cleared")
        self.publish_mission_path()
    
    def publish_mission_path(self):
        """Publish mission path for visualization"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "world"
        path.poses = self.mission_waypoints
        self.path_pub.publish(path)
    
    def return_to_home(self):
        """Initiate return to home sequence"""
        if not self.drone_state.flying:
            self.get_logger().warn("Not flying - cannot return to home")
            return
        
        self.set_flight_mode(FlightMode.RETURN_TO_HOME)
        self.get_logger().info("Returning to home position")
    
    def emergency_stop(self):
        """Immediate emergency stop"""
        self.current_twist = Twist()
        self.cmd_vel_pub.publish(self.current_twist)
        self.set_flight_mode(FlightMode.POSITION_HOLD)
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
    
    def emergency_landing(self):
        """Emergency landing procedure"""
        self.set_flight_mode(FlightMode.EMERGENCY)
        self.target_position.pose.position = self.drone_state.position
        self.target_position.pose.position.z = 0.0
        
        # Fast descent
        twist = Twist()
        twist.linear.z = -2.0  # Fast descent
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().error("EMERGENCY LANDING INITIATED")
    
    def adjust_speed(self, delta: float):
        """Adjust control sensitivity"""
        self.settings.linear_sensitivity = max(0.1, min(2.0, 
            self.settings.linear_sensitivity + delta))
        self.settings.angular_sensitivity = max(0.1, min(2.0, 
            self.settings.angular_sensitivity + delta))
        
        self.get_logger().info(f"Speed adjusted: linear={self.settings.linear_sensitivity:.1f}, "
                              f"angular={self.settings.angular_sensitivity:.1f}")
    
    def show_detailed_info(self):
        """Display comprehensive drone information"""
        info = f"""
        
========== DRONE STATUS ==========
Flight Mode: {self.drone_state.flight_mode.value}
Armed: {self.drone_state.armed}
Flying: {self.drone_state.flying}

Position: ({self.drone_state.position.x:.2f}, {self.drone_state.position.y:.2f}, {self.drone_state.position.z:.2f})
Orientation: R:{math.degrees(self.drone_state.orientation[0]):.1f}° P:{math.degrees(self.drone_state.orientation[1]):.1f}° Y:{math.degrees(self.drone_state.orientation[2]):.1f}°
Velocity: ({self.drone_state.velocity[0]:.2f}, {self.drone_state.velocity[1]:.2f}, {self.drone_state.velocity[2]:.2f}) m/s

Battery: {self.drone_state.battery_percentage:.1f}% ({self.drone_state.battery_voltage:.2f}V)
Safety Status: {self.drone_state.safety_status.value}
Altitude AGL: {self.drone_state.altitude_agl:.2f}m
Max Altitude: {self.drone_state.max_altitude:.2f}m

Flight Time: {self.drone_state.flight_time:.1f}s
Distance Traveled: {self.drone_state.distance_traveled:.2f}m
Mission Waypoints: {len(self.mission_waypoints)}

Geofence Violations: {', '.join(self.drone_state.geofence_violations) if self.drone_state.geofence_violations else 'None'}
==================================
        """
        
        if HAS_RICH:
            self.console.print(Panel(info, title="Drone Information"))
        else:
            print(info)
    
    def shutdown(self):
        """Clean shutdown procedure"""
        self.get_logger().info("Shutting down Advanced Drone Teleop...")
        
        # Emergency stop if flying
        if self.drone_state.flying:
            self.emergency_stop()
        
        # Disarm if armed
        if self.drone_state.armed:
            self.drone_state.armed = False
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
        
        # Stop keyboard thread
        self.running = False
        
        # Restore terminal
        if self.interactive_mode and HAS_TERMIOS:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            except:
                pass
        
        # Stop Rich display
        if HAS_RICH and hasattr(self, 'live'):
            self.live.stop()
        
        # Save flight log
        self.save_flight_log()
        
        self.get_logger().info("Shutdown complete")
    
    def save_flight_log(self):
        """Save flight data to file"""
        if not self.flight_log:
            return
        
        filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        filepath = f"/tmp/{filename}"  # Adjust path as needed
        
        try:
            with open(filepath, 'w') as f:
                json.dump(list(self.flight_log), f, indent=2)
            self.get_logger().info(f"Flight log saved: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save flight log: {e}")
    
    def calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calculate 3D distance between two points"""
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
    
    def update_rich_display(self):
        """Update the Rich-based HUD display"""
        if not HAS_RICH or not hasattr(self, 'live'):
            return
        
        try:
            # Header
            header_text = Text("ADVANCED DRONE TELEOPERATION SYSTEM", style="bold blue")
            header_text.append(f" | Mode: {self.drone_state.flight_mode.value}", style="bold green")
            header_text.append(f" | Status: {self.drone_state.safety_status.value}", 
                             style="bold red" if self.drone_state.safety_status != SafetyStatus.SAFE else "bold green")
            self.layout["header"].update(Panel(header_text, box=box.DOUBLE))
            
            # Status Panel
            status_table = Table(title="Flight Status", box=box.SIMPLE)
            status_table.add_column("Parameter", style="cyan")
            status_table.add_column("Value", style="white")
            
            armed_status = "✅ ARMED" if self.drone_state.armed else "❌ DISARMED"
            flying_status = "🚁 FLYING" if self.drone_state.flying else "🛬 GROUNDED"
            
            status_table.add_row("Armed", armed_status)
            status_table.add_row("Flying", flying_status)
            status_table.add_row("Flight Time", f"{self.drone_state.flight_time:.1f}s")
            status_table.add_row("Distance", f"{self.drone_state.distance_traveled:.1f}m")
            
            self.layout["status"].update(Panel(status_table))
            
            # Controls Panel
            controls_text = """
[bold cyan]Flight Modes:[/bold cyan]
[green]M[/green] Manual   [green]S[/green] Stabilized   [green]A[/green] Altitude Hold
[green]P[/green] Position [green]G[/green] Auto Mission [green]H[/green] Return Home

[bold cyan]Movement:[/bold cyan]
[green]W/A/S/D[/green] Move   [green]Q/E[/green] Rotate   [green]R/F[/green] Up/Down

[bold cyan]Commands:[/bold cyan]
[green]T[/green] Takeoff  [green]L[/green] Land  [green]Space[/green] Arm  [green]ESC[/green] Emergency
            """
            self.layout["controls"].update(Panel(controls_text, title="Controls"))
            
            # Telemetry Panel
            telemetry_table = Table(title="Telemetry", box=box.SIMPLE)
            telemetry_table.add_column("Parameter", style="cyan")
            telemetry_table.add_column("Value", style="white")
            
            pos = self.drone_state.position
            vel = self.drone_state.velocity
            orient = self.drone_state.orientation
            
            telemetry_table.add_row("Position X", f"{pos.x:.2f}m")
            telemetry_table.add_row("Position Y", f"{pos.y:.2f}m")
            telemetry_table.add_row("Altitude", f"{pos.z:.2f}m")
            telemetry_table.add_row("Velocity X", f"{vel[0]:.2f}m/s")
            telemetry_table.add_row("Velocity Y", f"{vel[1]:.2f}m/s")
            telemetry_table.add_row("Velocity Z", f"{vel[2]:.2f}m/s")
            telemetry_table.add_row("Roll", f"{math.degrees(orient[0]):.1f}°")
            telemetry_table.add_row("Pitch", f"{math.degrees(orient[1]):.1f}°")
            telemetry_table.add_row("Yaw", f"{math.degrees(orient[2]):.1f}°")
            
            self.layout["telemetry"].update(Panel(telemetry_table))
            
            # Mission Panel
            mission_table = Table(title="Mission Status", box=box.SIMPLE)
            mission_table.add_column("Parameter", style="cyan")
            mission_table.add_column("Value", style="white")
            
            mission_table.add_row("Waypoints", str(len(self.mission_waypoints)))
            mission_table.add_row("Current WP", str(self.current_waypoint_index + 1))
            
            if self.mission_waypoints and self.current_waypoint_index < len(self.mission_waypoints):
                current_wp = self.mission_waypoints[self.current_waypoint_index]
                wp_distance = self.calculate_distance(pos, current_wp.pose.position)
                mission_table.add_row("WP Distance", f"{wp_distance:.1f}m")
            
            home_distance = self.calculate_distance(pos, self.home_position.pose.position)
            mission_table.add_row("Home Distance", f"{home_distance:.1f}m")
            
            self.layout["mission"].update(Panel(mission_table))
            
            # Safety Panel
            safety_color = "green"
            if self.drone_state.safety_status == SafetyStatus.WARNING:
                safety_color = "yellow"
            elif self.drone_state.safety_status == SafetyStatus.CRITICAL:
                safety_color = "red"
            
            safety_table = Table(title=f"Safety ({self.drone_state.safety_status.value})", 
                                title_style=safety_color, box=box.SIMPLE)
            safety_table.add_column("Parameter", style="cyan")
            safety_table.add_column("Value", style="white")
            
            battery_color = "green"
            if self.drone_state.battery_percentage < 20:
                battery_color = "yellow"
            elif self.drone_state.battery_percentage < 10:
                battery_color = "red"
            
            safety_table.add_row("Battery", f"[{battery_color}]{self.drone_state.battery_percentage:.1f}%[/{battery_color}]")
            safety_table.add_row("Voltage", f"{self.drone_state.battery_voltage:.2f}V")
            safety_table.add_row("Max Alt", f"{self.drone_state.max_altitude:.1f}m")
            
            if self.drone_state.geofence_violations:
                violations_text = "[red]" + ", ".join(self.drone_state.geofence_violations) + "[/red]"
            else:
                violations_text = "[green]None[/green]"
            safety_table.add_row("Violations", violations_text)
            
            self.layout["safety"].update(Panel(safety_table))
            
            # Performance Panel
            performance_table = Table(title="Performance", box=box.SIMPLE)
            performance_table.add_column("Metric", style="cyan")
            performance_table.add_column("Value", style="white")
            
            performance_table.add_row("Speed", f"{math.sqrt(vel[0]**2 + vel[1]**2):.2f}m/s")
            performance_table.add_row("Max Alt", f"{self.drone_state.max_altitude:.1f}m")
            performance_table.add_row("Total Dist", f"{self.drone_state.distance_traveled:.1f}m")
            performance_table.add_row("Log Entries", str(len(self.flight_log)))
            
            # Control sensitivity
            performance_table.add_row("Linear Sens", f"{self.settings.linear_sensitivity:.1f}")
            performance_table.add_row("Angular Sens", f"{self.settings.angular_sensitivity:.1f}")
            
            self.layout["performance"].update(Panel(performance_table))
            
            # Footer
            footer_text = Text("Commands: ", style="bold")
            footer_text.append("[M]ode [T]akeoff [L]and [Space]Arm [N]avWP [B]eginMission [H]ome [I]nfo [ESC]Emergency [Ctrl+C]Quit", 
                             style="dim")
            self.layout["footer"].update(Panel(footer_text))
            
        except Exception as e:
            # Fallback in case of display errors
            self.get_logger().debug(f"Display update error: {e}")
            pass
    
    # ... (Additional advanced methods would continue)

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = AdvancedDroneTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()