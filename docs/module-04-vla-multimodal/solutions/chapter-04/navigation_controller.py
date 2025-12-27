#!/usr/bin/env python3
"""
Navigation Controller for Autonomous Humanoid

Handles robot navigation using Nav2 or simple velocity commands,
with obstacle avoidance and path planning integration.
"""

import time
from typing import Optional, Tuple

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

# Optional Nav2 imports
try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False


class NavigationController:
    """Navigation controller with Nav2 integration."""

    def __init__(self, parent_node):
        """Initialize navigation controller.

        Args:
            parent_node: Parent ROS 2 node
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()

        # Velocity publisher for simple navigation
        self.cmd_vel_pub = parent_node.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 action client (if available)
        self.nav_client = None
        if NAV2_AVAILABLE:
            try:
                self.nav_client = ActionClient(
                    parent_node, NavigateToPose, 'navigate_to_pose')
                self.logger.info("Nav2 action client created")
            except Exception as e:
                self.logger.warn(f"Nav2 not available: {e}")

        # Known locations (could be loaded from config)
        self.known_locations = {
            'table': (2.0, 1.0, 0.0),
            'kitchen': (5.0, 2.0, 0.0),
            'living_room': (0.0, 3.0, 0.0),
            'bedroom': (-2.0, 4.0, 0.0),
            'storage': (3.0, 0.0, 0.0),
            'charging_station': (0.0, 0.0, 0.0),
        }

        # Navigation parameters
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s

    def navigate_to(self, target: str, x: float = None, y: float = None) -> bool:
        """Navigate to a target location.

        Args:
            target: Location name or 'coordinates'
            x: X coordinate (if not using named location)
            y: Y coordinate (if not using named location)

        Returns:
            True if navigation succeeded, False otherwise
        """
        # Resolve coordinates
        if target in self.known_locations:
            x, y, _ = self.known_locations[target]
            self.logger.info(f"Navigating to known location '{target}' at ({x}, {y})")
        elif x is not None and y is not None:
            self.logger.info(f"Navigating to coordinates ({x}, {y})")
        else:
            self.logger.warn(f"Unknown location: {target}")
            return False

        # Use Nav2 if available
        if self.nav_client and NAV2_AVAILABLE:
            return self._navigate_with_nav2(x, y)
        else:
            return self._navigate_simple(x, y)

    def _navigate_with_nav2(self, x: float, y: float) -> bool:
        """Navigate using Nav2 action server."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.logger.warn("Nav2 server not available, using simple navigation")
            return self._navigate_simple(x, y)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.logger.info(f"Sending Nav2 goal: ({x}, {y})")

        future = self.nav_client.send_goal_async(goal)
        # For simplicity, we don't wait for completion here
        # In a real system, you'd want to track the goal status

        return True

    def _navigate_simple(self, x: float, y: float) -> bool:
        """Simple navigation using velocity commands (simulation)."""
        self.logger.info("Using simple velocity-based navigation")

        # Simulate movement with velocity commands
        twist = Twist()
        twist.linear.x = self.max_linear_velocity * 0.5  # Move at half speed

        # Publish for a duration proportional to distance
        distance = (x**2 + y**2)**0.5
        duration = min(distance / twist.linear.x, 5.0)  # Cap at 5 seconds for demo

        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        self.logger.info(f"Arrived at destination (simulated)")
        return True

    def stop(self):
        """Stop the robot immediately."""
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        self.logger.info("Robot stopped")

    def rotate(self, angle_degrees: float) -> bool:
        """Rotate the robot by a given angle.

        Args:
            angle_degrees: Rotation angle (positive = left, negative = right)

        Returns:
            True if rotation succeeded
        """
        twist = Twist()
        twist.angular.z = self.max_angular_velocity * (1 if angle_degrees > 0 else -1)

        # Estimate duration based on angle
        duration = abs(angle_degrees) / 90.0  # ~1s per 90 degrees at max speed

        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        return True

    def is_available(self) -> bool:
        """Check if navigation is available."""
        return True  # Simple navigation always available
