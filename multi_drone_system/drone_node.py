#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class DroneController(Node):
    def __init__(self, drone_id=0):
        super().__init__(f'drone_{drone_id}')
        self.drone_id = drone_id
        
        # Declare parameters
        self.declare_parameter('drone_id', drone_id)
        drone_id = self.get_parameter('drone_id').value
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/drone_{drone_id}/cmd_vel',
            10
        )
        
        # Create a timer for demonstration movement
        self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f'Drone {drone_id} controller started')
        
    def timer_callback(self):
        """Send simple movement commands for testing"""
        cmd = Twist()
        cmd.linear.z = 0.1  # Small upward movement
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Publishing test velocity command')

def main(args=None):
    rclpy.init(args=args)
    
    # Get drone ID from parameters
    drone_controller = DroneController()
    
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()