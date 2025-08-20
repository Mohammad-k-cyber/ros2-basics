#!/usr/bin/env python3
import sys, termios, tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleKeyboard(Node):
    def __init__(self):
        super().__init__('turtle_keyboard')
        # Create publisher for turtle commands
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Declare speed parameter
        self.declare_parameter('speed', 1.0)
        self.speed = self.get_parameter('speed').value

    def get_key(self):
        # Get single keypress without Enter
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        self.get_logger().info("Use WASD to move, Q to quit")
        while True:
            key = self.get_key()
            msg = Twist()
            # Set velocity based on key pressed
            if key == 'w':
                msg.linear.x = self.speed      # Forward
            elif key == 's':
                msg.linear.x = -self.speed     # Backward
            elif key == 'a':
                msg.angular.z = self.speed     # Left
            elif key == 'd':
                msg.angular.z = -self.speed    # Right
            elif key == 'q':
                break                          # Quit
            # Send command to turtle
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()