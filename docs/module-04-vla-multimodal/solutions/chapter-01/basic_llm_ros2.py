#!/usr/bin/env python3
"""
Basic LLM-ROS 2 Integration Example

This example demonstrates how to integrate a Large Language Model (LLM) with ROS 2
to enable natural language robot control. The node receives text commands, sends them
to an LLM for interpretation, and publishes corresponding robot movement commands.

Prerequisites:
- ROS 2 Humble installed and sourced
- Python 3.8+
- OpenAI API key set as environment variable

Usage:
    export OPENAI_API_KEY="your-api-key-here"
    python3 basic_llm_ros2.py
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables from .env file if present
load_dotenv()


class LLMRobotNode(Node):
    """ROS 2 node that uses LLM to interpret natural language commands for robot control."""

    def __init__(self):
        super().__init__('llm_robot_node')

        # Initialize OpenAI client
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set!")
            raise ValueError("Missing OpenAI API key")

        self.llm_client = OpenAI(api_key=api_key)
        self.get_logger().info("LLM client initialized successfully")

        # Create ROS 2 publisher for robot velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10  # Queue size
        )
        self.get_logger().info("ROS 2 publisher created for /cmd_vel topic")

        # System prompt for the LLM to understand robot commands
        self.system_prompt = """You are a robot control assistant. Convert natural language commands into robot movement parameters.

Available commands:
- Move forward/backward: Set linear.x (positive = forward, negative = backward, range: -1.0 to 1.0 m/s)
- Turn left/right: Set angular.z (positive = left, negative = right, range: -1.0 to 1.0 rad/s)
- Stop: Set all values to 0.0

Respond ONLY with a JSON object in this exact format:
{"linear_x": <float>, "angular_z": <float>, "explanation": "<brief explanation>"}

Examples:
"Move forward" → {"linear_x": 0.5, "angular_z": 0.0, "explanation": "Moving forward at 0.5 m/s"}
"Turn left" → {"linear_x": 0.0, "angular_z": 0.5, "explanation": "Rotating left at 0.5 rad/s"}
"Stop" → {"linear_x": 0.0, "angular_z": 0.0, "explanation": "Stopping all movement"}
"""

    def process_command(self, natural_language_command):
        """
        Process a natural language command using the LLM and publish the resulting robot command.

        Args:
            natural_language_command (str): Human-readable command (e.g., "Move forward 1 meter")

        Returns:
            bool: True if command was successfully processed and published, False otherwise
        """
        self.get_logger().info(f"Processing command: '{natural_language_command}'")

        try:
            # Step 1: Send command to LLM for interpretation
            response = self.llm_client.chat.completions.create(
                model="gpt-4",  # Use GPT-4 for better reasoning
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": natural_language_command}
                ],
                temperature=0.0,  # Deterministic responses for safety
                max_tokens=100
            )

            # Step 2: Parse LLM response
            llm_response = response.choices[0].message.content.strip()
            self.get_logger().info(f"LLM response: {llm_response}")

            # Parse JSON response
            import json
            try:
                command_params = json.loads(llm_response)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to parse LLM JSON response: {e}")
                return False

            # Step 3: Create ROS 2 Twist message
            twist_msg = Twist()
            twist_msg.linear.x = float(command_params.get("linear_x", 0.0))
            twist_msg.angular.z = float(command_params.get("angular_z", 0.0))

            # Safety bounds checking
            twist_msg.linear.x = max(-1.0, min(1.0, twist_msg.linear.x))
            twist_msg.angular.z = max(-1.0, min(1.0, twist_msg.angular.z))

            # Step 4: Publish command to robot
            self.cmd_publisher.publish(twist_msg)
            self.get_logger().info(
                f"Published command - linear.x: {twist_msg.linear.x}, "
                f"angular.z: {twist_msg.angular.z}"
            )
            self.get_logger().info(f"Explanation: {command_params.get('explanation', 'N/A')}")

            return True

        except Exception as e:
            # Handle API errors (network failures, rate limits, etc.)
            self.get_logger().error(f"Error processing command: {type(e).__name__}: {str(e)}")

            if "rate_limit" in str(e).lower():
                self.get_logger().warn("API rate limit exceeded. Please wait and try again.")
            elif "network" in str(e).lower() or "connection" in str(e).lower():
                self.get_logger().warn("Network error. Check your internet connection.")
            elif "authentication" in str(e).lower() or "api_key" in str(e).lower():
                self.get_logger().error("Authentication failed. Check your OPENAI_API_KEY.")

            return False


def main(args=None):
    """Main function to run the LLM-ROS 2 integration example."""

    # Initialize ROS 2
    rclpy.init(args=args)

    try:
        # Create the LLM robot node
        node = LLMRobotNode()

        # Example commands to test
        test_commands = [
            "Move forward 1 meter",
            "Turn left 90 degrees",
            "Move backward",
            "Stop the robot"
        ]

        print("\n" + "="*60)
        print("LLM-ROS 2 Integration Example")
        print("="*60)
        print("\nTesting natural language robot control...\n")

        # Process each test command
        for i, command in enumerate(test_commands, 1):
            print(f"\n[Test {i}/{len(test_commands)}] Command: \"{command}\"")
            print("-" * 60)
            success = node.process_command(command)
            if success:
                print("✓ Command processed successfully")
            else:
                print("✗ Command processing failed")

            # Small delay between commands
            import time
            time.sleep(1)

        print("\n" + "="*60)
        print("Testing complete!")
        print("="*60 + "\n")

        # Keep the node running to maintain the ROS 2 publisher
        print("Node is running. Press Ctrl+C to exit.")
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    except Exception as e:
        print(f"\nFatal error: {e}")
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    main()
