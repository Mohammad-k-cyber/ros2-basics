#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParamClient(Node):
    def __init__(self):
        super().__init__('param_client')
        # Declare parameters with default values
        self.declare_parameter('bg_r', 0)
        self.declare_parameter('bg_g', 0)
        self.declare_parameter('bg_b', 0)

        # Set background color parameters (light blue)
        self.set_parameters([
            rclpy.parameter.Parameter('background_r', rclpy.Parameter.Type.INTEGER, 200),
            rclpy.parameter.Parameter('background_g', rclpy.Parameter.Type.INTEGER, 200),
            rclpy.parameter.Parameter('background_b', rclpy.Parameter.Type.INTEGER, 255),
        ])
        self.get_logger().info("Background color changed!")

def main(args=None):
    rclpy.init(args=args)
    node = ParamClient()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()