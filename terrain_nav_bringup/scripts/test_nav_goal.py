#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class NavGoalPublisher(Node):
    def __init__(self):
        super().__init__('nav_goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        time.sleep(2) # Wait for connections

    def send_goal(self, x, y, z=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0 # Minimal orientation
        
        self.get_logger().info(f'Publishing goal: x={x}, y={y}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalPublisher()
    
    # Example goal: move 2 meters forward
    node.send_goal(2.0, 0.0)
    
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
