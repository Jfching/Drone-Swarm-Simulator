#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import math
import numpy as np

class TFVisualizer(Node):
    def __init__(self):
        super().__init__('tf_visualizer')
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 10)
        
        # Path markers for visualization
        self.path_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        # Store drone paths
        self.drone_paths = [[] for _ in range(5)]
        self.max_path_length = 100  # Maximum number of points in each path
        
        # Create a timer
        self.create_timer(0.05, self.publish_tf)  # 20Hz update
        
        # Number of drones
        self.num_drones = 5
        self.get_logger().info('TF Visualizer started')
        
    def publish_tf(self):
        # Get current time
        now = self.get_clock().now()
        time_sec = now.nanoseconds / 1e9
        
        # Create marker array for drones
        markers = MarkerArray()
        
        # Create marker array for paths
        path_markers = MarkerArray()
        
        # Publish transforms for each drone
        for i in range(self.num_drones):
            # Calculate position
            angle = 2.0 * math.pi * i / self.num_drones + time_sec * 0.5
            radius = 3.0 + math.sin(time_sec * 0.2) * 1.0
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 1.0 + math.sin(time_sec + i) * 0.5
            
            # Calculate direction (for drone front orientation)
            dx = -radius * math.sin(angle) * 0.5  # Derivative of position gives velocity
            dy = radius * math.cos(angle) * 0.5
            yaw = math.atan2(dy, dx)
            
            # Create transform
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f'drone_{i}'
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            
            # Set orientation based on direction of movement
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sy
            t.transform.rotation.w = cy
            
            # Publish transform
            self.tf_broadcaster.sendTransform(t)
            
            # Store path point
            self.drone_paths[i].append((x, y, z))
            if len(self.drone_paths[i]) > self.max_path_length:
                self.drone_paths[i].pop(0)
            
            # Create drone body marker
            body = Marker()
            body.header.frame_id = "world"
            body.header.stamp = now.to_msg()
            body.ns = "drones"
            body.id = i*10
            body.type = Marker.CUBE
            body.action = Marker.ADD
            
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = z
            body.pose.orientation.x = 0.0
            body.pose.orientation.y = 0.0
            body.pose.orientation.z = sy
            body.pose.orientation.w = cy
            
            body.scale.x = 0.4  # Length
            body.scale.y = 0.4  # Width
            body.scale.z = 0.1  # Height
            
            # Different color for each drone
            body.color.r = 0.2 if i % 3 == 0 else 0.1
            body.color.g = 0.2 if i % 3 == 1 else 0.1
            body.color.b = 0.2 if i % 3 == 2 else 0.1
            body.color.a = 1.0
            
            markers.markers.append(body)
            
            # Create 4 rotors
            rotor_offsets = [
                (0.2, 0.2, 0.05),  # Front-right
                (-0.2, 0.2, 0.05), # Front-left
                (-0.2, -0.2, 0.05),# Back-left
                (0.2, -0.2, 0.05)  # Back-right
            ]
            
            for j, (dx, dy, dz) in enumerate(rotor_offsets):
                # Transform offsets by drone's orientation
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                rx = cos_yaw * dx - sin_yaw * dy
                ry = sin_yaw * dx + cos_yaw * dy
                
                rotor = Marker()
                rotor.header.frame_id = "world"
                rotor.header.stamp = now.to_msg()
                rotor.ns = "rotors"
                rotor.id = i*10 + j + 1
                rotor.type = Marker.CYLINDER
                rotor.action = Marker.ADD
                
                rotor.pose.position.x = x + rx
                rotor.pose.position.y = y + ry
                rotor.pose.position.z = z + dz
                
                # Set orientation based on drone's orientation
                rotor.pose.orientation.x = 0.0
                rotor.pose.orientation.y = 0.0
                rotor.pose.orientation.z = sy
                rotor.pose.orientation.w = cy
                
                rotor.scale.x = 0.1  # Diameter
                rotor.scale.y = 0.1  # Diameter
                rotor.scale.z = 0.02 # Height
                
                # Make rotors colorful
                if j == 0:  # Front-right: red
                    rotor.color.r = 1.0
                    rotor.color.g = 0.0
                    rotor.color.b = 0.0
                elif j == 1:  # Front-left: green
                    rotor.color.r = 0.0
                    rotor.color.g = 1.0
                    rotor.color.b = 0.0
                elif j == 2:  # Back-left: blue
                    rotor.color.r = 0.0
                    rotor.color.g = 0.0
                    rotor.color.b = 1.0
                else:  # Back-right: yellow
                    rotor.color.r = 1.0
                    rotor.color.g = 1.0
                    rotor.color.b = 0.0
                    
                rotor.color.a = 1.0
                
                markers.markers.append(rotor)
            
            # Create path marker
            path = Marker()
            path.header.frame_id = "world"
            path.header.stamp = now.to_msg()
            path.ns = "paths"
            path.id = i
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            
            path.scale.x = 0.03  # Line width
            
            # Set path color based on drone ID
            path.color.r = 1.0 if i % 3 == 0 else 0.0
            path.color.g = 1.0 if i % 3 == 1 else 0.0
            path.color.b = 1.0 if i % 3 == 2 else 1.0
            path.color.a = 0.8
            
            # Add path points
            for px, py, pz in self.drone_paths[i]:
                p = Point()
                p.x = px
                p.y = py
                p.z = pz
                path.points.append(p)
            
            path_markers.markers.append(path)
            
            # Log position periodically
            if i == 0 and time_sec % 1.0 < 0.05:
                self.get_logger().info(f'Drone 0 position: ({x:.2f}, {y:.2f}, {z:.2f})')
        
        # Publish all markers
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)

def main(args=None):
    rclpy.init(args=args)
    node = TFVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()