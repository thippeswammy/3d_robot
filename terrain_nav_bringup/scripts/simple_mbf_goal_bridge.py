#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from mbf_msgs.action import MoveBase

class SimpleMBFGoalBridge(Node):
    def __init__(self):
        super().__init__('simple_mbf_goal_bridge')
        
        # Subscribe to RViz goal topic
        # RViz 2 "2D Goal Pose" tool typically publishes to /goal_pose
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Subscribe to legacy topic just in case
        self.legacy_goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        self._action_client = ActionClient(self, MoveBase, '/move_base_flex/move_base')
        
        self.get_logger().info('Simple MBF Goal Bridge started. Waiting for goals on /goal_pose or /move_base_simple/goal...')

    def goal_callback(self, msg):
        self.get_logger().info('Received goal from RViz. Sending to MBF...')
        
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose = msg
        
        # Wait for server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MBF action server not available!')
            return

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MBF')
            return

        self.get_logger().info('Goal accepted by MBF. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with outcome: {result.outcome}')
        self.get_logger().info(f'Message: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMBFGoalBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
