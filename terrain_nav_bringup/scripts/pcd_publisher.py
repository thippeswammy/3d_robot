#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import sys
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.declare_parameter('pcd_filename', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic_name', 'map_cloud')
        self.declare_parameter('publish_rate', 1.0) # Hz

        pcd_filename = self.get_parameter('pcd_filename').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(PointCloud2, topic_name, 10)
        
        if not pcd_filename:
            self.get_logger().error("No pcd_filename parameter provided")
            return

        self.get_logger().info(f"Loading PCD from: {pcd_filename}")
        self.pcd_msg = self.load_pcd(pcd_filename)
        
        if self.pcd_msg:
             self.timer = self.create_timer(1.0/rate, self.timer_callback)
             self.get_logger().info(f"Publishing to {topic_name} at {rate} Hz")
        else:
             self.get_logger().error("Failed to create PointCloud2 message")

    def load_pcd(self, filepath):
        try:
            pcd = o3d.io.read_point_cloud(filepath)
            if not pcd.has_points():
                 self.get_logger().error("PCD file is empty or invalid")
                 return None
            
            points = np.asarray(pcd.points)
            self.get_logger().info(f"Loaded {len(points)} points")
            
            header = Header()
            header.frame_id = self.frame_id
            
            # Create PointCloud2 msg
            # open3d points are float64, ROS usually wants float32
            msg = point_cloud2.create_cloud_xyz32(header, points)
            return msg
            
        except Exception as e:
            self.get_logger().error(f"Error loading PCD: {str(e)}")
            return None

    def timer_callback(self):
        if self.pcd_msg:
            self.pcd_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.pcd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
