#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        # Parameters
        self.declare_parameter("waypoint_tolerance", 0.2)
        self.declare_parameter("max_velocity", 2.0)
        self.declare_parameter("lookahead_distance", 1.0)

        self.tolerance = self.get_parameter("waypoint_tolerance").value
        self.max_vel = self.get_parameter("max_velocity").value
        self.lookahead = self.get_parameter("lookahead_distance").value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/drone/pose", self.pose_callback, 10
        )
        self.waypoint_sub = self.create_subscription(
            PoseStamped, "/waypoint", self.add_waypoint, 10
        )

        # State
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.mission_active = False

        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Example mission
        self.load_example_mission()

        self.get_logger().info("Waypoint Follower initialized")

    def load_example_mission(self):
        """Load a square pattern mission"""
        self.waypoints = [
            (0.0, 0.0, 1.0),  # Home
            (2.0, 0.0, 1.0),  # Point 1
            (2.0, 2.0, 1.5),  # Point 2
            (0.0, 2.0, 2.0),  # Point 3
            (0.0, 0.0, 1.5),  # Point 4
            (0.0, 0.0, 0.5),  # Landing approach
        ]
        self.publish_path()
        self.get_logger().info(f"Loaded mission with {len(self.waypoints)} waypoints")

    def add_waypoint(self, msg):
        """Add a waypoint from external command"""
        waypoint = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.waypoints.append(waypoint)
        self.publish_path()
        self.get_logger().info(f"Added waypoint: {waypoint}")

    def pose_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg.pose

    def publish_path(self):
        """Publish planned path for visualization"""
        path = Path()
        path.header.frame_id = "world"
        path.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

    def control_loop(self):
        """Main control loop for waypoint following"""
        if not self.current_pose or not self.waypoints or not self.mission_active:
            return

        # Get current position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z

        # Check if we've reached all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.mission_complete()
            return

        # Get target waypoint
        target = self.waypoints[self.current_waypoint_idx]

        # Calculate error
        error_x = target[0] - x
        error_y = target[1] - y
        error_z = target[2] - z
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        # Check if waypoint reached
        if distance < self.tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}")
            return

        # Pure pursuit controller
        # Calculate velocity commands
        cmd = Twist()

        # Normalize and scale velocities
        if distance > 0:
            vel_scale = min(1.0, distance / self.lookahead) * self.max_vel
            cmd.linear.x = (error_x / distance) * vel_scale
            cmd.linear.y = (error_y / distance) * vel_scale
            cmd.linear.z = (error_z / distance) * vel_scale * 0.5  # Slower vertical

        # Calculate yaw to face direction of travel
        if abs(error_x) > 0.1 or abs(error_y) > 0.1:
            desired_yaw = math.atan2(error_y, error_x)
            # Simple P controller for yaw
            cmd.angular.z = desired_yaw * 0.5

        self.cmd_vel_pub.publish(cmd)

    def start_mission(self):
        """Start autonomous mission"""
        self.mission_active = True
        self.current_waypoint_idx = 0
        self.get_logger().info("Mission started")

    def stop_mission(self):
        """Stop mission and hover"""
        self.mission_active = False
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Mission stopped")

    def mission_complete(self):
        """Called when all waypoints are reached"""
        self.mission_active = False
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Mission complete!")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()

    # Start mission after 5 seconds
    node.create_timer(5.0, lambda: node.start_mission())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
