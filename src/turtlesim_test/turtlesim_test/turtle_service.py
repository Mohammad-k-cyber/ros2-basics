#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ClearBackground(Node):
    def __init__(self):
        super().__init__('clear_background_client')
        # Create service client for clearing background
        self.client = self.create_client(Empty, '/clear')
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /clear service...')
        # Create empty request and call service
        self.request = Empty.Request()
        self.clear_background()

    def clear_background(self):
        # Call the service asynchronously
        future = self.client.call_async(self.request)
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Background cleared!')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = ClearBackground()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()