#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.action import RotateAbsolute

class TurtleActionClient(Node):
    def __init__(self):
        super().__init__('turtle_action_client')
        # Create action client for turtle rotation
        self._action_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
        self.action_done = False  # Flag to track completion
        self.send_goal(1.57)  # Rotate 90 degrees (π/2 radians)

    def send_goal(self, angle):
        # Create goal message
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = angle
        
        # Wait for action server and send goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle goal acceptance/rejection
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.action_done = True  # Mark as done
            return
        self.get_logger().info('Goal accepted!')
        # Get result when action completes
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Handle action completion
        result = future.result().result
        self.get_logger().info(f'Rotation done, final pose: {result.delta}')
        self.action_done = True  # Mark as done

def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()
    
    # Spin until action is complete
    while rclpy.ok() and not action_client.action_done:
        rclpy.spin_once(action_client, timeout_sec=0.1)
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()