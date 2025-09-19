#!/usr/bin/env python3
"""
ROS2 Drone Controller - Python Implementation
=============================================

Professional quadcopter flight controller with:
- Cascaded PID control (Position → Attitude → Rate)
- Multi-level control loops for stability
- Real-time motor mixing for X-configuration
- Gravity compensation and safety limits
- ROS2 native implementation in Python

Converted from C++ implementation to Python for improved
maintainability and rapid development.

Author: Drone Perimeter Surveillance Team
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import math
import numpy as np
from dataclasses import dataclass

# ROS2 Message Types
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


@dataclass
class PIDController:
    """Professional PID Controller with windup protection and clamping"""
    
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    max_output: float = float('inf')
    min_output: float = float('-inf')
    max_integral: float = float('inf')
    
    def __post_init__(self):
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID output with anti-windup and derivative kick prevention
        
        Args:
            error: Current error value
            dt: Time step in seconds
            
        Returns:
            Control output clamped to min/max limits
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with windup protection
        self.error_sum += error * dt
        self.error_sum = max(min(self.error_sum, self.max_integral),
                             -self.max_integral)
        i_term = self.ki * self.error_sum
        
        # Derivative term (derivative of error to prevent setpoint kick)
        d_error = (error - self.last_error) / dt if dt > 0 else 0.0
        d_term = self.kd * d_error
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output and implement anti-windup
        if output > self.max_output:
            output = self.max_output
            # Anti-windup: reduce integral if we're saturated
            if self.ki != 0:
                self.error_sum = (self.max_output - p_term - d_term) / self.ki
        elif output < self.min_output:
            output = self.min_output
            # Anti-windup: reduce integral if we're saturated
            if self.ki != 0:
                self.error_sum = (self.min_output - p_term - d_term) / self.ki
        
        self.last_error = error
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.error_sum = 0.0
        self.last_error = 0.0


class DroneController(Node):
    """
    Advanced ROS2 Drone Flight Controller
    
    Implements a professional-grade cascaded control system:
    1. Position Control Loop (50 Hz) → Desired angles
    2. Attitude Control Loop (100 Hz) → Desired rates
    3. Rate Control Loop (100 Hz) → Motor commands
    
    Features:
    - Robust PID controllers with anti-windup
    - Motor mixing for X-configuration quadcopter
    - Safety limits and emergency handling
    - Real-time telemetry and diagnostics
    """
    
    def __init__(self):
        super().__init__('drone_controller_python')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize PID controllers
        self._initialize_pid_controllers()
        
        # Initialize state variables
        self._initialize_state()
        
        # Setup ROS2 interfaces
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()
        
        self.get_logger().info("Python Drone Controller initialized")
        freq_hz = 1000.0/self.control_dt_ms
        self.get_logger().info(f"Control frequency: {freq_hz:.1f} Hz")
    
    def _declare_parameters(self):
        """Declare ROS2 parameters with default values"""
        self.declare_parameter('mass', 1.5)
        self.declare_parameter('arm_length', 0.22)
        self.declare_parameter('thrust_coefficient', 2.95e-05)
        self.declare_parameter('max_thrust', 15.0)
        self.declare_parameter('control_frequency', 100.0)  # Hz
        self.declare_parameter('max_angle', 30.0)  # degrees
        self.declare_parameter('gravity', 9.81)
    
    def _get_parameters(self):
        """Retrieve parameter values"""
        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        arm_param = self.get_parameter('arm_length').get_parameter_value()
        self.arm_length = arm_param.double_value
        thrust_param = self.get_parameter('thrust_coefficient')
        self.thrust_coeff = thrust_param.get_parameter_value().double_value
        thrust_max_param = self.get_parameter('max_thrust')
        self.max_thrust = thrust_max_param.get_parameter_value().double_value
        freq_param = self.get_parameter('control_frequency')
        self.control_freq = freq_param.get_parameter_value().double_value
        angle_param = self.get_parameter('max_angle').get_parameter_value()
        self.max_angle_rad = math.radians(angle_param.double_value)
        grav_param = self.get_parameter('gravity').get_parameter_value()
        self.gravity = grav_param.double_value
        
        # Calculate control timing
        self.control_dt = 1.0 / self.control_freq
        self.control_dt_ms = int(1000.0 / self.control_freq)
    
    def _initialize_pid_controllers(self):
        """Initialize all PID controllers with tuned parameters"""
        
        # Position Controllers (Position → Angle)
        self.pos_x_pid = PIDController(
            kp=1.5, ki=0.0, kd=0.5,
            max_output=10.0, min_output=-10.0,
            max_integral=5.0
        )
        
        self.pos_y_pid = PIDController(
            kp=1.5, ki=0.0, kd=0.5,
            max_output=10.0, min_output=-10.0,
            max_integral=5.0
        )
        
        self.pos_z_pid = PIDController(
            kp=2.0, ki=0.1, kd=1.0,
            max_output=20.0, min_output=-20.0,
            max_integral=10.0
        )
        
        # Attitude Controllers (Angle → Rate)
        self.roll_pid = PIDController(
            kp=7.0, ki=0.0, kd=0.0,
            max_output=50.0, min_output=-50.0
        )
        
        self.pitch_pid = PIDController(
            kp=7.0, ki=0.0, kd=0.0,
            max_output=50.0, min_output=-50.0
        )
        
        self.yaw_pid = PIDController(
            kp=5.0, ki=0.0, kd=0.0,
            max_output=30.0, min_output=-30.0
        )
        
        # Rate Controllers (Rate → Torque)
        self.roll_rate_pid = PIDController(
            kp=0.15, ki=0.05, kd=0.003,
            max_output=1.0, min_output=-1.0,
            max_integral=0.5
        )
        
        self.pitch_rate_pid = PIDController(
            kp=0.15, ki=0.05, kd=0.003,
            max_output=1.0, min_output=-1.0,
            max_integral=0.5
        )
        
        self.yaw_rate_pid = PIDController(
            kp=0.2, ki=0.05, kd=0.0,
            max_output=1.0, min_output=-1.0,
            max_integral=0.5
        )
    
    def _initialize_state(self):
        """Initialize drone state variables"""
        # Current state
        self.current_position = np.array([0.0, 0.0, 0.0])
        # roll, pitch, yaw
        self.current_orientation = np.array([0.0, 0.0, 0.0])
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
        
        # Target state (initialized to hover at 1m altitude)
        self.target_position = np.array([0.0, 0.0, 1.0])
        self.target_yaw = 0.0
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        self.target_yaw_rate = 0.0
        
        # Safety flags
        self.armed = True
        self.emergency_stop = False
        
        # Timing
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 2.0  # seconds
    
    def _setup_publishers(self):
        """Setup ROS2 publishers"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Individual motor speed publishers for Gazebo
        from std_msgs.msg import Float64
        self.motor_0_pub = self.create_publisher(Float64, 'motor_speed_0', qos)
        self.motor_1_pub = self.create_publisher(Float64, 'motor_speed_1', qos)
        self.motor_2_pub = self.create_publisher(Float64, 'motor_speed_2', qos)
        self.motor_3_pub = self.create_publisher(Float64, 'motor_speed_3', qos)
    
    def _setup_subscribers(self):
        """Setup ROS2 subscribers"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/drone/imu/data',
            self.imu_callback,
            qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            qos
        )
    
    def _setup_timers(self):
        """Setup control loop timers"""
        self.control_timer = self.create_timer(
            self.control_dt,
            self.control_loop
        )
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Process velocity commands and convert to position targets
        
        Args:
            msg: Twist message with linear and angular velocities
        """
        self.target_velocity[0] = msg.linear.x
        self.target_velocity[1] = msg.linear.y
        self.target_velocity[2] = msg.linear.z
        self.target_yaw_rate = msg.angular.z
        
        # Update target position based on velocity (velocity control mode)
        self.target_position += self.target_velocity * self.control_dt
        self.target_yaw += self.target_yaw_rate * self.control_dt
        
        # Safety: prevent ground collision
        if self.target_position[2] < 0.1:
            self.target_position[2] = 0.1
        
        # Normalize yaw angle
        self.target_yaw = self._normalize_angle(self.target_yaw)
        
        # Update command timestamp
        self.last_cmd_time = self.get_clock().now()
        
        # Debug logging
        if abs(msg.linear.x) > 0.1 or abs(msg.linear.y) > 0.1:
            self.get_logger().info(
                f"CMD_VEL: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, "
                f"target_pos=[{self.target_position[0]:.2f}, "
                f"{self.target_position[1]:.2f}, {self.target_position[2]:.2f}]"
            )
    
    def imu_callback(self, msg: Imu):
        """
        Process IMU data to extract orientation and angular velocities
        
        Args:
            msg: IMU message with orientation and angular velocity
        """
        # Extract quaternion and convert to Euler angles
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Convert to numpy array
        self.current_orientation = np.array([roll, pitch, yaw])
        
        # Extract angular velocities
        self.current_angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
    
    def pose_callback(self, msg: PoseStamped):
        """
        Process pose data to extract current position
        
        Args:
            msg: PoseStamped message with position and orientation
        """
        self.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
    
    def control_loop(self):
        """
        Main control loop - executes at specified frequency
        
        Implements cascaded control:
        1. Position control → desired accelerations → desired angles
        2. Attitude control → desired angular rates
        3. Rate control → motor torques → motor commands
        """
        if not self.armed or self.emergency_stop:
            self._send_zero_commands()
            return
        
        # Check for command timeout
        current_time = self.get_clock().now()
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > self.cmd_timeout:
            # Command timeout - maintain position
            self.target_velocity.fill(0.0)
            self.target_yaw_rate = 0.0
        
        # === POSITION CONTROL LOOP ===
        position_error = self.target_position - self.current_position
        
        # Compute desired accelerations
        acc_x = self.pos_x_pid.compute(position_error[0], self.control_dt)
        acc_y = self.pos_y_pid.compute(position_error[1], self.control_dt)
        acc_z = self.pos_z_pid.compute(position_error[2], self.control_dt) + self.gravity
        
        # Convert desired accelerations to desired angles (in body frame)
        yaw = self.current_orientation[2]
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        
        # Rotate accelerations to body frame
        acc_x_body = acc_x * cos_yaw + acc_y * sin_yaw
        acc_y_body = -acc_x * sin_yaw + acc_y * cos_yaw
        
        # Convert to desired roll/pitch angles
        target_roll = math.atan2(-acc_y_body, self.gravity)
        target_pitch = math.atan2(acc_x_body, self.gravity)
        
        # Clamp angles for safety
        target_roll = max(min(target_roll, self.max_angle_rad), -self.max_angle_rad)
        target_pitch = max(min(target_pitch, self.max_angle_rad), -self.max_angle_rad)
        
        # === ATTITUDE CONTROL LOOP ===
        roll_error = target_roll - self.current_orientation[0]
        pitch_error = target_pitch - self.current_orientation[1]
        yaw_error = self._normalize_angle(self.target_yaw - self.current_orientation[2])
        
        # Compute desired angular rates
        target_roll_rate = self.roll_pid.compute(roll_error, self.control_dt)
        target_pitch_rate = self.pitch_pid.compute(pitch_error, self.control_dt)
        target_yaw_rate = self.yaw_pid.compute(yaw_error, self.control_dt)
        
        # === RATE CONTROL LOOP ===
        roll_rate_error = target_roll_rate - self.current_angular_velocity[0]
        pitch_rate_error = target_pitch_rate - self.current_angular_velocity[1]
        yaw_rate_error = target_yaw_rate - self.current_angular_velocity[2]
        
        # Compute control torques
        roll_torque = self.roll_rate_pid.compute(roll_rate_error, self.control_dt)
        pitch_torque = self.pitch_rate_pid.compute(pitch_rate_error, self.control_dt)
        yaw_torque = self.yaw_rate_pid.compute(yaw_rate_error, self.control_dt)
        
        # === MOTOR MIXING ===
        total_thrust = self.mass * acc_z
        
        # X-configuration motor mixing
        # Motor layout: 0=Front, 1=Right, 2=Back, 3=Left
        motor_commands = np.array([
            total_thrust/4 - pitch_torque/2 + yaw_torque/4,  # Front
            total_thrust/4 - roll_torque/2 - yaw_torque/4,   # Right  
            total_thrust/4 + pitch_torque/2 + yaw_torque/4,  # Back
            total_thrust/4 + roll_torque/2 - yaw_torque/4    # Left
        ])
        
        # Convert thrust to motor speeds (simplified model)
        motor_speeds = np.sqrt(np.maximum(0.0, motor_commands / self.thrust_coeff))
        
        # Publish motor commands
        self._publish_motor_commands(motor_speeds)
        
        # Debug logging (only occasionally to avoid spam)
        if int(current_time.nanoseconds / 1e9) % 5 == 0:  # Every 5 seconds
            self.get_logger().info(
                f"Position error: [{position_error[0]:.2f}, "
                f"{position_error[1]:.2f}, {position_error[2]:.2f}], "
                f"Motor speeds: [{motor_speeds[0]:.1f}, {motor_speeds[1]:.1f}, "
                f"{motor_speeds[2]:.1f}, {motor_speeds[3]:.1f}]"
            )
    
    def _publish_motor_commands(self, motor_speeds: np.ndarray):
        """
        Publish individual motor speed commands
        
        Args:
            motor_speeds: Array of 4 motor speeds
        """
        # Debug: Log motor speeds being published
        self.get_logger().info(f"Publishing motor speeds: {motor_speeds}")
        
        # Publish to individual motor speed topics
        msg0 = Float64()
        msg0.data = float(motor_speeds[0])
        self.motor_0_pub.publish(msg0)
        
        msg1 = Float64()
        msg1.data = float(motor_speeds[1])
        self.motor_1_pub.publish(msg1)
        
        msg2 = Float64()
        msg2.data = float(motor_speeds[2])
        self.motor_2_pub.publish(msg2)
        
        msg3 = Float64()
        msg3.data = float(motor_speeds[3])
        self.motor_3_pub.publish(msg3)
    
    def _send_zero_commands(self):
        """Send zero commands to all motors (emergency stop)"""
        # Send zero to all motors
        zero_msg = Float64()
        zero_msg.data = 0.0
        
        self.motor_0_pub.publish(zero_msg)
        self.motor_1_pub.publish(zero_msg)
        self.motor_2_pub.publish(zero_msg)
        self.motor_3_pub.publish(zero_msg)
    
    def _normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to [-pi, pi]
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle in [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    """Main entry point for the drone controller"""
    rclpy.init(args=args)
    
    try:
        drone_controller = DroneController()
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in drone controller: {e}")
    finally:
        if 'drone_controller' in locals():
            drone_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
