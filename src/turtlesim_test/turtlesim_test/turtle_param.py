#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamClient(Node):
    def __init__(self):
        super().__init__('param_client')
        
        # Declare parameters with default values
        self.declare_parameter('background_r', 0)
        self.declare_parameter('background_g', 0)
        self.declare_parameter('background_b', 0)

        # Set background color parameters (light blue)
        self.set_parameters([
            Parameter('background_r', Parameter.Type.INTEGER, 200),
            Parameter('background_g', Parameter.Type.INTEGER, 200),
            Parameter('background_b', Parameter.Type.INTEGER, 255),
        ])
        self.get_logger().info("Background color changed to light blue!")

def main(args=None):
    rclpy.init(args=args)
    node = ParamClient()
    # Spin once to allow parameter changes to take effect
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
