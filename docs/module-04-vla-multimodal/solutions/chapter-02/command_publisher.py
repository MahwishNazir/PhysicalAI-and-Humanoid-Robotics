#!/usr/bin/env python3
"""
Command Publisher for Voice-Controlled Robotics

This module parses voice commands and publishes corresponding ROS 2
Twist messages for robot control. It handles natural language variations
and provides configurable command mappings.

Features:
- Natural language command parsing
- Configurable velocity parameters
- Support for directional and rotational commands
- Safety bounds enforcement
- Command history tracking

Usage:
    from command_publisher import CommandPublisher

    publisher = CommandPublisher()
    twist = publisher.parse_and_publish("move forward slowly")
"""

import re
from typing import Optional, Dict, Tuple
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


@dataclass
class VelocityConfig:
    """Configuration for velocity commands."""
    # Linear velocity (m/s)
    linear_slow: float = 0.2
    linear_normal: float = 0.5
    linear_fast: float = 0.8

    # Angular velocity (rad/s)
    angular_slow: float = 0.3
    angular_normal: float = 0.5
    angular_fast: float = 0.8

    # Safety limits
    max_linear: float = 1.0
    max_angular: float = 1.0


class CommandPublisher(Node):
    """ROS 2 node for publishing velocity commands from voice input."""

    def __init__(self, config: Optional[VelocityConfig] = None):
        """Initialize the command publisher.

        Args:
            config: Velocity configuration (uses defaults if None)
        """
        super().__init__('command_publisher')

        self.config = config or VelocityConfig()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_log_pub = self.create_publisher(String, '/command_log', 10)

        # Subscriber for voice commands
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_callback,
            10
        )

        # Command history
        self.command_history = []
        self.max_history = 100

        # Define command patterns with regex
        self._init_command_patterns()

        self.get_logger().info("Command publisher initialized")

    def _init_command_patterns(self):
        """Initialize command recognition patterns."""
        # Direction patterns
        self.direction_patterns = {
            'forward': re.compile(
                r'(move|go|drive)?\s*(forward|ahead|straight)',
                re.IGNORECASE
            ),
            'backward': re.compile(
                r'(move|go|drive)?\s*(backward|back|reverse)',
                re.IGNORECASE
            ),
            'left': re.compile(
                r'(turn|rotate|spin)?\s*(left)',
                re.IGNORECASE
            ),
            'right': re.compile(
                r'(turn|rotate|spin)?\s*(right)',
                re.IGNORECASE
            ),
            'stop': re.compile(
                r'(stop|halt|freeze|pause)',
                re.IGNORECASE
            ),
        }

        # Speed modifiers
        self.speed_patterns = {
            'slow': re.compile(r'(slow|slowly|careful)', re.IGNORECASE),
            'fast': re.compile(r'(fast|quick|rapid)', re.IGNORECASE),
        }

        # Distance/angle patterns (for future use)
        self.distance_pattern = re.compile(r'(\d+\.?\d*)\s*(meter|m|feet|ft)', re.IGNORECASE)
        self.angle_pattern = re.compile(r'(\d+)\s*(degree|deg)', re.IGNORECASE)

    def parse_command(self, text: str) -> Optional[Dict]:
        """Parse natural language command into velocity parameters.

        Args:
            text: Natural language command string

        Returns:
            Dict with 'linear_x', 'angular_z', 'direction', 'speed' or None
        """
        text = text.lower().strip()

        if not text:
            return None

        result = {
            'linear_x': 0.0,
            'angular_z': 0.0,
            'direction': None,
            'speed': 'normal',
            'original_text': text
        }

        # Detect speed modifier
        if self.speed_patterns['slow'].search(text):
            result['speed'] = 'slow'
        elif self.speed_patterns['fast'].search(text):
            result['speed'] = 'fast'

        # Get velocity based on speed
        if result['speed'] == 'slow':
            linear_vel = self.config.linear_slow
            angular_vel = self.config.angular_slow
        elif result['speed'] == 'fast':
            linear_vel = self.config.linear_fast
            angular_vel = self.config.angular_fast
        else:
            linear_vel = self.config.linear_normal
            angular_vel = self.config.angular_normal

        # Detect direction
        if self.direction_patterns['stop'].search(text):
            result['direction'] = 'stop'
            result['linear_x'] = 0.0
            result['angular_z'] = 0.0
        elif self.direction_patterns['forward'].search(text):
            result['direction'] = 'forward'
            result['linear_x'] = linear_vel
        elif self.direction_patterns['backward'].search(text):
            result['direction'] = 'backward'
            result['linear_x'] = -linear_vel
        elif self.direction_patterns['left'].search(text):
            result['direction'] = 'left'
            result['angular_z'] = angular_vel
        elif self.direction_patterns['right'].search(text):
            result['direction'] = 'right'
            result['angular_z'] = -angular_vel
        else:
            # No recognized direction
            return None

        return result

    def apply_safety_bounds(self, linear_x: float, angular_z: float) -> Tuple[float, float]:
        """Apply safety bounds to velocity values.

        Args:
            linear_x: Linear velocity (m/s)
            angular_z: Angular velocity (rad/s)

        Returns:
            Tuple of (bounded_linear_x, bounded_angular_z)
        """
        bounded_linear = max(-self.config.max_linear,
                            min(self.config.max_linear, linear_x))
        bounded_angular = max(-self.config.max_angular,
                             min(self.config.max_angular, angular_z))
        return bounded_linear, bounded_angular

    def create_twist_message(self, linear_x: float, angular_z: float) -> Twist:
        """Create a Twist message with safety bounds applied.

        Args:
            linear_x: Linear velocity (m/s)
            angular_z: Angular velocity (rad/s)

        Returns:
            Twist message
        """
        twist = Twist()
        bounded_linear, bounded_angular = self.apply_safety_bounds(linear_x, angular_z)
        twist.linear.x = bounded_linear
        twist.angular.z = bounded_angular
        return twist

    def publish_command(self, command: Dict) -> Twist:
        """Publish velocity command to /cmd_vel.

        Args:
            command: Parsed command dictionary

        Returns:
            Published Twist message
        """
        twist = self.create_twist_message(
            command['linear_x'],
            command['angular_z']
        )

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)

        # Log command
        log_msg = String()
        log_msg.data = (
            f"Command: {command['original_text']} -> "
            f"direction={command['direction']}, "
            f"speed={command['speed']}, "
            f"linear_x={twist.linear.x:.2f}, "
            f"angular_z={twist.angular.z:.2f}"
        )
        self.command_log_pub.publish(log_msg)

        # Track history
        self.command_history.append(command)
        if len(self.command_history) > self.max_history:
            self.command_history.pop(0)

        self.get_logger().info(log_msg.data)

        return twist

    def parse_and_publish(self, text: str) -> Optional[Twist]:
        """Parse command and publish if valid.

        Args:
            text: Natural language command

        Returns:
            Published Twist message, or None if command not recognized
        """
        command = self.parse_command(text)
        if command is None:
            self.get_logger().warn(f"Unrecognized command: '{text}'")
            return None
        return self.publish_command(command)

    def voice_callback(self, msg: String):
        """Callback for /voice_command topic.

        Args:
            msg: String message with voice command text
        """
        self.parse_and_publish(msg.data)

    def get_last_command(self) -> Optional[Dict]:
        """Get the last executed command.

        Returns:
            Last command dict or None if no commands executed
        """
        return self.command_history[-1] if self.command_history else None

    def emergency_stop(self):
        """Immediately stop the robot."""
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn("Emergency stop executed!")


def main(args=None):
    """Main function demonstrating the command publisher."""
    rclpy.init(args=args)

    print("\n" + "=" * 60)
    print("Command Publisher Demo")
    print("=" * 60 + "\n")

    try:
        publisher = CommandPublisher()

        # Test commands
        test_commands = [
            "move forward",
            "go forward slowly",
            "turn left",
            "spin right fast",
            "move backward",
            "go back slowly",
            "stop",
            "halt",
            "invalid command xyz",
        ]

        print("Testing command parsing and publishing:\n")

        for cmd in test_commands:
            print(f"Input: '{cmd}'")
            result = publisher.parse_and_publish(cmd)
            if result:
                print(f"  -> linear_x={result.linear.x:.2f}, "
                      f"angular_z={result.angular.z:.2f}")
            else:
                print("  -> Command not recognized")
            print()

        print("\nDemo complete. Starting continuous mode...")
        print("Subscribe to /voice_command to send commands")
        print("Press Ctrl+C to exit\n")

        rclpy.spin(publisher)

    except KeyboardInterrupt:
        print("\nShutdown requested")
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    main()
