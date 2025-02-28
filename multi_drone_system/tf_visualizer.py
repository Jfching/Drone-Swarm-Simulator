#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import math

class TFVisualizer(Node):
    def __init__(self):
        super().__init__('tf_visualizer')
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 10)
        
        # Create a timer
        self.create_timer(0.05, self.publish_tf)  # 20Hz update
        
        # Number of drones
        self.num_drones = 5
        self.get_logger().info('TF Visualizer started')
        
    def publish_tf(self):
        # Get current time
        now = self.get_clock().now()
        time_sec = now.nanoseconds / 1e9
        
        # Create marker array
        markers = MarkerArray()
        
        # Publish transforms for each drone
        for i in range(self.num_drones):
            # Calculate position
            angle = 2.0 * math.pi * i / self.num_drones + time_sec * 0.5
            radius = 3.0 + math.sin(time_sec * 0.2) * 1.0
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 1.0 + math.sin(time_sec + i) * 0.5
            
            # Create transform
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f'drone_{i}'
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # Publish transform
            self.tf_broadcaster.sendTransform(t)
            
            # Create marker for this drone
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = now.to_msg()
            marker.ns = "drones"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            
            marker.color.r = 1.0 if i % 3 == 0 else 0.0
            marker.color.g = 1.0 if i % 3 == 1 else 0.0
            marker.color.b = 1.0 if i % 3 == 2 else 0.0
            marker.color.a = 1.0
            
            markers.markers.append(marker)
            
            # Log position periodically
            if i == 0 and time_sec % 1.0 < 0.05:
                self.get_logger().info(f'Drone 0 position: ({x:.2f}, {y:.2f}, {z:.2f})')
        
        # Publish all markers
        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = TFVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()