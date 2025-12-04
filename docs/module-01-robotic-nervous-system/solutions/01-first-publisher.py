#!/usr/bin/env python3
"""
Simple ROS 2 Publisher Node
This node publishes "Hello World" messages to the 'hello' topic.
"""

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from std_msgs.msg import String  # Standard string message type


class HelloWorldPublisher(Node):
    """A node that publishes Hello World messages."""

    def __init__(self):
        # Initialize the node with name 'hello_world_publisher'
        super().__init__('hello_world_publisher')

        # Create a publisher for String messages on the 'hello' topic
        # Queue size of 10 means we'll buffer up to 10 messages if subscribers are slow
        self.publisher_ = self.create_publisher(String, 'hello', 10)

        # Create a timer that calls our callback function every 1.0 seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track how many messages we've sent
        self.counter = 0

        self.get_logger().info('Hello World Publisher has started!')

    def timer_callback(self):
        """This function is called every second by the timer."""
        # Create a new String message
        msg = String()
        msg.data = f'Hello World: {self.counter}'

        # Publish the message to the 'hello' topic
        self.publisher_.publish(msg)

        # Log the message for debugging
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.counter += 1


def main(args=None):
    """Main function to run the node."""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our node
    node = HelloWorldPublisher()

    try:
        # Keep the node running until interrupted (Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
