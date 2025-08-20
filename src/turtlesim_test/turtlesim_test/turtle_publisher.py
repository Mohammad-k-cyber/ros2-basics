#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher')
        # Create publisher for turtle velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Set timer to publish every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.move_turtle)
        self.get_logger().info("Publisher started: Moving Turtle!")

    def move_turtle(self):
        # Create movement message
        msg = Twist()
        msg.linear.x = 1.0    # Move forward
        msg.angular.z = 0.5   # Turn left (creates circular motion)
        # Send command to turtle
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePublisher()
    rclpy.spin(node)  # Keep node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()