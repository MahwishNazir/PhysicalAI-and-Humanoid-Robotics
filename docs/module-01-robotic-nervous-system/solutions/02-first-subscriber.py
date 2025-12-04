#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber Node
This node listens for messages on the 'hello' topic and prints them.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    """A node that subscribes to Hello World messages."""

    def __init__(self):
        # Initialize the node with name 'hello_world_subscriber'
        super().__init__('hello_world_subscriber')

        # Create a subscriber for String messages on the 'hello' topic
        # Our callback function will be called whenever a message arrives
        self.subscription = self.create_subscription(
            String,                    # Message type
            'hello',                   # Topic name
            self.listener_callback,    # Callback function
            10)                        # Queue size

        self.get_logger().info('Hello World Subscriber is listening...')

    def listener_callback(self, msg):
        """This function is called whenever a message is received."""
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    node = HelloWorldSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
