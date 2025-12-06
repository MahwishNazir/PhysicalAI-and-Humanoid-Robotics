#!/usr/bin/env python3
"""
Hello ROS 2 - Combined Publisher and Subscriber Example
Module 1, Chapter 1: What is ROS 2?

This script demonstrates basic ROS 2 concepts by combining
a publisher and subscriber in a single file for educational purposes.

Prerequisites:
- ROS 2 Humble installed
- rclpy Python package

Usage:
1. Source ROS 2: source /opt/ros/humble/setup.bash
2. Run publisher in terminal 1: python3 01-hello-ros2.py --mode publisher
3. Run subscriber in terminal 2: python3 01-hello-ros2.py --mode subscriber

Author: Physical AI & Humanoid Robotics Book
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse


class HelloWorldPublisher(Node):
    """
    A simple publisher node that sends Hello World messages.

    This node demonstrates:
    - Creating a node
    - Publishing to a topic
    - Using timers for periodic execution
    - Logging information
    """

    def __init__(self):
        super().__init__('hello_world_publisher')

        # Create publisher: topic name is 'hello', message type is String, queue size is 10
        self.publisher_ = self.create_publisher(String, 'hello', 10)

        # Create a timer that calls our callback every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track messages
        self.counter = 0

        self.get_logger().info('Hello World Publisher started!')
        self.get_logger().info('Publishing to topic: /hello')

    def timer_callback(self):
        """Called every second to publish a message."""
        msg = String()
        msg.data = f'Hello World: {self.counter}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log for debugging
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.counter += 1


class HelloWorldSubscriber(Node):
    """
    A simple subscriber node that receives Hello World messages.

    This node demonstrates:
    - Creating a node
    - Subscribing to a topic
    - Handling incoming messages with a callback
    - Logging information
    """

    def __init__(self):
        super().__init__('hello_world_subscriber')

        # Create subscriber: topic name is 'hello', message type is String
        # The listener_callback function will be called when messages arrive
        self.subscription = self.create_subscription(
            String,
            'hello',
            self.listener_callback,
            10)  # Queue size

        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('Hello World Subscriber started!')
        self.get_logger().info('Listening on topic: /hello')

    def listener_callback(self, msg):
        """Called whenever a message is received on the /hello topic."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to run either publisher or subscriber based on command-line argument.
    """
    parser = argparse.ArgumentParser(description='Hello ROS 2 Example')
    parser.add_argument('--mode', type=str, choices=['publisher', 'subscriber'],
                        default='publisher',
                        help='Run as publisher or subscriber')

    # Parse arguments (skip first argument which is the script name)
    import sys
    parsed_args = parser.parse_args(sys.argv[1:])

    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the appropriate node based on mode
    if parsed_args.mode == 'publisher':
        node = HelloWorldPublisher()
    else:
        node = HelloWorldSubscriber()

    try:
        # Keep the node running until interrupted (Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')

    # Clean up
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
