#!/usr/bin/env python3

import numpy as np
# Patch np.float for compatibility with transforms3d/old libraries
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SimpleLocalizationNode(Node):
    def __init__(self):
        super().__init__('simple_localization_node')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to Gazebo Ground Truth Pose
        # The bridge typically publishes to /model/<name>/pose of type tf2_msgs/TFMessage or geometry_msgs/PoseArray
        # But commonly we bridge /model/my_bot/odometry which is Odometry.
        # Let's use the same Odometry topic from the bridge, but treat it as absolute truth for map->base
        # Subscribe to Gazebo Ground Truth Pose (Bridged as PoseStamped)
        from tf2_msgs.msg import TFMessage
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/tf_gt',
            self.tf_gt_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.get_logger().info("Simple Localization Node (TF-GT) Started. Waiting for /tf_gt...")

    def tf_gt_callback(self, msgs):
        # find the transform from world/uneven_terrain to base_footprint (or link)
        # Gazebo PosePublisher with static_publisher:true usually publishes world->model
        for transform in msgs.transforms:
            # Check for the robot's model pose
            if transform.child_frame_id == 'my_bot':
                t_map_base = self.transform_to_matrix(transform.transform)
                self.process_localization(t_map_base, transform.header.stamp)
                break

    def process_localization(self, t_map_base, stamp):
        # 2. Get T_odom_base from TF
        try:
            # Look up the latest available transform
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            t_odom_base = self.transform_to_matrix(trans.transform)
            
            # 3. Compute T_map_odom
            t_odom_base_inv = tf_transformations.inverse_matrix(t_odom_base)
            t_map_odom = tf_transformations.concatenate_matrices(t_map_base, t_odom_base_inv)
            
            # 4. publish
            self.publish_transform(t_map_odom, stamp)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not lookup odom->base: {e}", throttle_duration_sec=5.0)
            pass

    def pose_to_matrix(self, pose):
        t = [pose.position.x, pose.position.y, pose.position.z]
        r = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return tf_transformations.compose_matrix(translate=t, angles=tf_transformations.euler_from_quaternion(r))

    def transform_to_matrix(self, transform):
        t = [transform.translation.x, transform.translation.y, transform.translation.z]
        r = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        return tf_transformations.compose_matrix(translate=t, angles=tf_transformations.euler_from_quaternion(r))

    def publish_transform(self, matrix, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        try:
            scale, shear, angles, trans, persp = tf_transformations.decompose_matrix(matrix)
        except Exception as e:
            self.get_logger().error(f"Failed to decompose matrix: {e}")
            return
            
        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])
        
        q = tf_transformations.quaternion_from_euler(*angles)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
