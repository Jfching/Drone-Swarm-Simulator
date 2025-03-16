#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import tf2_ros
import math
import numpy as np
import random
from enum import Enum

# Define formation types enum
class FormationType(Enum):
    NONE = 0
    CIRCLE = 1
    LINE = 2
    GRID = 3
    V_SHAPE = 4
    CUSTOM = 5

# Define swarm controller class
class SwarmController:   
    def __init__(self, num_drones=5):
        #Initialize swarm controller
        self.num_drones = num_drones
        
        #Boids algorithm parameters
        self.separation_weight = 1.5
        self.alignment_weight = 1.0
        self.cohesion_weight = 1.0
        self.obstacle_avoidance_weight = 2.0
        self.target_weight = 0.8
        
        #Distances
        self.separation_distance = 1.0
        self.neighbor_distance = 3.0
        self.obstacle_avoid_distance = 2.0
        
        #Maximum speeds
        self.max_speed = 1.0
        self.max_force = 0.5
        
        # Formation parameters
        self.current_formation = FormationType.NONE
        self.formation_scale = 3.0
        self.custom_positions = []
        self.formation_center = [0.0, 0.0, 1.5]  # Default center point
        
    def set_formation(self, formation_type, scale=None, center=None, custom_positions=None):
        """
        Set the desired formation for the swarm.
        
        Args:
            formation_type: FormationType enum value
            scale: Size of the formation (if None, uses current scale)
            center: [x, y, z] center of formation (if None, uses current center)
            custom_positions: List of [x, y, z] positions for CUSTOM formation
        """
        self.current_formation = formation_type
        
        if scale is not None:
            self.formation_scale = scale
            
        if center is not None:
            self.formation_center = center
            
        if custom_positions is not None and formation_type == FormationType.CUSTOM:
            self.custom_positions = custom_positions
            
        # Make sure we have enough positions for custom formation
        if formation_type == FormationType.CUSTOM and len(self.custom_positions) < self.num_drones:
            # Fill with defaults if needed
            while len(self.custom_positions) < self.num_drones:
                idx = len(self.custom_positions)
                angle = 2.0 * math.pi * idx / self.num_drones
                x = math.cos(angle) * self.formation_scale
                y = math.sin(angle) * self.formation_scale
                z = self.formation_center[2]
                self.custom_positions.append([x, y, z])
                
    def get_formation_target(self, drone_id):
        """
        Get the target position for a drone in the current formation.
        
        Args:
            drone_id: ID of the drone (0 to num_drones-1)
            
        Returns:
            [x, y, z] target position
        """
        cx, cy, cz = self.formation_center
        
        if self.current_formation == FormationType.CIRCLE:
            angle = 2.0 * math.pi * drone_id / self.num_drones
            x = cx + math.cos(angle) * self.formation_scale
            y = cy + math.sin(angle) * self.formation_scale
            z = cz
            return [x, y, z]
            
        elif self.current_formation == FormationType.LINE:
            # Line along the x-axis
            offset = self.formation_scale * (drone_id - (self.num_drones - 1) / 2) / max(self.num_drones - 1, 1)
            return [cx + offset, cy, cz]
            
        elif self.current_formation == FormationType.GRID:
            # Calculate a square grid
            side_length = math.ceil(math.sqrt(self.num_drones))
            row = drone_id // side_length
            col = drone_id % side_length
            
            # Center the grid
            offset_x = (col - (side_length - 1) / 2) * self.formation_scale / side_length
            offset_y = (row - (side_length - 1) / 2) * self.formation_scale / side_length
            
            return [cx + offset_x, cy + offset_y, cz]
            
        elif self.current_formation == FormationType.V_SHAPE:
            # V shape formation
            if self.num_drones == 1:
                return [cx, cy, cz]
                
            half = (self.num_drones - 1) // 2
            if drone_id == 0:  # Leader at the point of the V
                return [cx + self.formation_scale, cy, cz]
            elif drone_id <= half:  # Right side of V
                idx = drone_id
                offset_x = self.formation_scale - idx * self.formation_scale / half
                offset_y = idx * self.formation_scale / half
                return [cx + offset_x, cy + offset_y, cz]
            else:  # Left side of V
                idx = drone_id - half
                offset_x = self.formation_scale - idx * self.formation_scale / half
                offset_y = -idx * self.formation_scale / half
                return [cx + offset_x, cy + offset_y, cz]
                
        elif self.current_formation == FormationType.CUSTOM:
            if drone_id < len(self.custom_positions):
                pos = self.custom_positions[drone_id]
                return [cx + pos[0], cy + pos[1], cz + pos[2]]
                
        # Default behavior - scatter in a circle
        angle = 2.0 * math.pi * drone_id / self.num_drones
        x = cx + math.cos(angle) * self.formation_scale
        y = cy + math.sin(angle) * self.formation_scale
        z = cz
        return [x, y, z]
    
    def apply_boids_rules(self, drone_id, drone_positions, drone_velocities, obstacles=None, target=None):
        """
        Apply the Boids flocking algorithm rules to calculate the desired movement.
        
        Args:
            drone_id: ID of the drone to calculate forces for
            drone_positions: List of [x, y, z] positions for all drones
            drone_velocities: List of [vx, vy, vz] velocities for all drones
            obstacles: List of [x, y, z, radius] obstacles to avoid
            target: Optional [x, y, z] target position
            
        Returns:
            [dx, dy, dz] - The desired velocity vector
        """
        if obstacles is None:
            obstacles = []
            
        pos = np.array(drone_positions[drone_id])
        vel = np.array(drone_velocities[drone_id])
        
        # Initialize forces
        separation = np.zeros(3)
        alignment = np.zeros(3)
        cohesion = np.zeros(3)
        obstacle_avoidance = np.zeros(3)
        target_force = np.zeros(3)
        
        # Count neighbors
        neighbors = 0
        center_of_mass = np.zeros(3)
        
        # Apply rules for each neighbor
        for i, (other_pos, other_vel) in enumerate(zip(drone_positions, drone_velocities)):
            if i == drone_id:
                continue
                
            other_pos = np.array(other_pos)
            other_vel = np.array(other_vel)
            
            # Vector from this drone to neighbor
            offset = other_pos - pos
            distance = np.linalg.norm(offset)
            
            # Separation - avoid crowding neighbors
            if distance < self.separation_distance and distance > 0:
                # Calculate steering force away from neighbor
                separation -= offset / distance
                
            # Only consider drones within neighbor distance for alignment and cohesion
            if distance < self.neighbor_distance:
                # Alignment - steer towards average heading of neighbors
                alignment += other_vel
                
                # Cohesion - steer towards center of mass of neighbors
                center_of_mass += other_pos
                
                neighbors += 1
                
        # Complete the calculations if we have neighbors
        if neighbors > 0:
            # Alignment - average velocity of neighbors
            alignment /= neighbors
            # Limit the magnitude of alignment force
            if np.linalg.norm(alignment) > 0:
                alignment = self._limit_magnitude(alignment, self.max_force)
            
            # Cohesion - steer towards center of mass
            center_of_mass /= neighbors
            cohesion = center_of_mass - pos
            # Limit the magnitude of cohesion force
            if np.linalg.norm(cohesion) > 0:
                cohesion = self._limit_magnitude(cohesion, self.max_force)
        
        # Obstacle avoidance
        for obstacle in obstacles:
            obs_pos = np.array(obstacle[:3])
            obs_radius = obstacle[3]
            
            # Vector from obstacle to drone
            offset = pos - obs_pos
            distance = np.linalg.norm(offset)
            
            # Check if we need to avoid
            if distance < obs_radius + self.obstacle_avoid_distance:
                # Strength of avoidance is inversely proportional to distance
                # (closer = stronger avoidance)
                avoid_strength = 1.0 - (distance / (obs_radius + self.obstacle_avoid_distance))
                
                # Direction away from obstacle
                avoid_dir = offset / distance if distance > 0 else np.array([1, 0, 0])
                
                # Add to avoidance force
                obstacle_avoidance += avoid_dir * avoid_strength * self.max_force
        
        # Target seeking
        if target is not None:
            target = np.array(target)
            target_vec = target - pos
            
            # Only apply if we're not at the target
            if np.linalg.norm(target_vec) > 0.1:
                target_force = self._limit_magnitude(target_vec, self.max_force)
        
        # Apply weights to each force
        separation *= self.separation_weight
        alignment *= self.alignment_weight
        cohesion *= self.cohesion_weight
        obstacle_avoidance *= self.obstacle_avoidance_weight
        target_force *= self.target_weight
        
        # Combine all forces
        acceleration = separation + alignment + cohesion + obstacle_avoidance + target_force
        
        # Update velocity with acceleration
        new_vel = vel + acceleration
        
        # Limit speed
        if np.linalg.norm(new_vel) > self.max_speed:
            new_vel = self._limit_magnitude(new_vel, self.max_speed)
            
        return new_vel.tolist()
    
    def _limit_magnitude(self, vector, max_magnitude):
        """
        Limit the magnitude of a vector.
        
        Args:
            vector: NumPy array representing the vector
            max_magnitude: Maximum allowed magnitude
            
        Returns:
            Vector with limited magnitude
        """
        magnitude = np.linalg.norm(vector)
        if magnitude > max_magnitude and magnitude > 0:
            return vector * (max_magnitude / magnitude)
        return vector

class TFVisualizer(Node):
    def __init__(self):
        super().__init__('tf_visualizer')
        
        # Debug flag
        self.debug = False
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a marker publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 10)
        
        # Path markers for visualization
        self.path_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        # Command subscriber
        self.cmd_sub = self.create_subscription(
            String, 
            '/swarm/command', 
            self.command_callback, 
            10
        )
        
        # Number of drones
        self.num_drones = 5
        
        # Initialize the swarm controller
        self.swarm_controller = SwarmController(self.num_drones)
        self.swarm_controller.max_speed = 0.5  # Slow down for smoother movement
        
        # Set the default formation
        self.swarm_controller.set_formation(FormationType.CIRCLE, scale=3.0)
        
        # Store drone positions, velocities, and paths
        # Initialize with spread-out positions
        self.drone_positions = []
        for i in range(self.num_drones):
            angle = 2.0 * math.pi * i / self.num_drones
            radius = 1.0  # Start closer to center
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 1.0
            self.drone_positions.append([x, y, z])
            
        # Initialize with small outward velocities to encourage movement
        self.drone_velocities = []
        for i in range(self.num_drones):
            angle = 2.0 * math.pi * i / self.num_drones
            vx = 0.1 * math.cos(angle)  # Small initial velocity
            vy = 0.1 * math.sin(angle)
            vz = 0.0
            self.drone_velocities.append([vx, vy, vz])
            
        self.drone_paths = [[] for _ in range(self.num_drones)]
        self.max_path_length = 100  # Maximum number of points in each path
        
        # Initialize random obstacles
        self.initialize_random_obstacles()
        
        # Initialize unreachable targets tracker
        self.unreachable_targets = [False] * self.num_drones
        self.closest_safe_positions = [None] * self.num_drones
        self.at_closest_safe_position = [False] * self.num_drones
        
        # Behavior state
        self.behavior_mode = "formation"  # "formation" or "flocking"
        
        # Create a timer
        timer_period = 0.05  # 20Hz update
        self.update_timer = self.create_timer(timer_period, self.update_and_publish)
        
        # Physics timestep
        self.dt = timer_period
        
        # Last update time
        self.last_update_time = self.get_clock().now()
        
        self.get_logger().info('TF Visualizer started')
    
    def initialize_random_obstacles(self):
        """Generate random obstacles each time the simulation starts."""
        # Random number of obstacles between 3-7
        num_obstacles = random.randint(3, 7)
        
        # Create empty obstacle list
        self.obstacles = []
        
        # Define the area where obstacles can appear
        min_x, max_x = -5.0, 5.0
        min_y, max_y = -5.0, 5.0
        min_z, max_z = 0.5, 2.5
        
        # Min distance between obstacles
        min_obstacle_distance = 1.5
        
        # Generate obstacles
        for _ in range(num_obstacles):
            # Try to find a valid position that's not too close to other obstacles
            valid_position = False
            attempts = 0
            
            while not valid_position and attempts < 20:
                # Generate random position
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)
                z = random.uniform(min_z, max_z)
                
                # Random radius between 0.5 and 1.5
                radius = random.uniform(0.5, 1.5)
                
                # Check if this position is far enough from other obstacles
                valid_position = True
                for existing_obs in self.obstacles:
                    ex, ey, ez, er = existing_obs[:4]
                    dist = ((x - ex)**2 + (y - ey)**2 + (z - ez)**2)**0.5
                    if dist < (radius + er + min_obstacle_distance):
                        valid_position = False
                        break
                
                attempts += 1
            
            # If we found a valid position, add the obstacle
            if valid_position:
                # Random color variations - reddish obstacles
                r = random.uniform(0.7, 1.0)
                g = random.uniform(0.1, 0.3)
                b = random.uniform(0.1, 0.3)
                
                # Store obstacle as [x, y, z, radius, r, g, b]
                self.obstacles.append([x, y, z, radius, r, g, b])
        
        self.get_logger().info(f'Created {len(self.obstacles)} random obstacles')
        
    def command_callback(self, msg):
        """Handle commands sent to the swarm."""
        cmd = msg.data.strip().lower()
        
        self.get_logger().info(f'Received command: "{cmd}"')
        
        if cmd == "formation_circle":
            self.behavior_mode = "formation"
            self.swarm_controller.set_formation(FormationType.CIRCLE)
            # Reset closest safe positions when changing formation
            self.closest_safe_positions = [None] * self.num_drones
            self.at_closest_safe_position = [False] * self.num_drones
            self.get_logger().info('Switching to circle formation')
            
        elif cmd == "formation_line":
            self.behavior_mode = "formation"
            self.swarm_controller.set_formation(FormationType.LINE)
            # Reset closest safe positions when changing formation
            self.closest_safe_positions = [None] * self.num_drones
            self.at_closest_safe_position = [False] * self.num_drones
            self.get_logger().info('Switching to line formation')
            
        elif cmd == "formation_grid":
            self.behavior_mode = "formation"
            self.swarm_controller.set_formation(FormationType.GRID)
            # Reset closest safe positions when changing formation
            self.closest_safe_positions = [None] * self.num_drones
            self.at_closest_safe_position = [False] * self.num_drones
            self.get_logger().info('Switching to grid formation')
            
        elif cmd == "formation_v":
            self.behavior_mode = "formation"
            self.swarm_controller.set_formation(FormationType.V_SHAPE)
            # Reset closest safe positions when changing formation
            self.closest_safe_positions = [None] * self.num_drones
            self.at_closest_safe_position = [False] * self.num_drones
            self.get_logger().info('Switching to V formation')
            
        elif cmd == "flocking":
            self.behavior_mode = "flocking"
            self.get_logger().info('Switching to flocking behavior')
            
        elif cmd == "new_obstacles":
            self.initialize_random_obstacles()
            # Reset closest safe positions when changing obstacles
            self.closest_safe_positions = [None] * self.num_drones
            self.at_closest_safe_position = [False] * self.num_drones
            self.get_logger().info('Generated new random obstacles')
            
        elif cmd.startswith("scale_"):
            try:
                scale = float(cmd.split("_")[1])
                self.swarm_controller.formation_scale = scale
                # Reset closest safe positions when changing scale
                self.closest_safe_positions = [None] * self.num_drones
                self.at_closest_safe_position = [False] * self.num_drones
                self.get_logger().info(f'Formation scale set to {scale}')
            except (IndexError, ValueError):
                self.get_logger().error('Invalid scale command')
                
        elif cmd.startswith("center_"):
            try:
                parts = cmd.split("_")
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3]) if len(parts) > 3 else 1.5
                self.swarm_controller.formation_center = [x, y, z]
                # Reset closest safe positions when changing center
                self.closest_safe_positions = [None] * self.num_drones
                self.at_closest_safe_position = [False] * self.num_drones
                self.get_logger().info(f'Formation center set to [{x}, {y}, {z}]')
            except (IndexError, ValueError):
                self.get_logger().error('Invalid center command')
        
    def update_and_publish(self):
        """Update drone positions and publish visualizations."""
        # Calculate elapsed time since last update
        now = self.get_clock().now()
        elapsed = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        
        # Use fixed timestep if elapsed time is too large or small
        if elapsed > 0.1 or elapsed < 0.001:
            elapsed = self.dt
            
        # Create marker arrays
        markers = MarkerArray()
        path_markers = MarkerArray()
        
        # Debug output
        if self.debug:
            # Only log every 20 cycles
            if not hasattr(self, 'debug_counter'):
                self.debug_counter = 0
            self.debug_counter += 1
            
            if self.debug_counter % 20 == 0:
                self.get_logger().info(f'Update cycle - behavior mode: {self.behavior_mode}')
                for i in range(min(2, self.num_drones)):  # Just show first couple drones
                    pos = self.drone_positions[i]
                    vel = self.drone_velocities[i]
                    reachable = "UNREACHABLE" if self.unreachable_targets[i] else "reachable"
                    at_safe = "AT SAFE POSITION" if self.at_closest_safe_position[i] else ""
                    self.get_logger().info(f'Drone {i}: pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), vel=({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}), target: {reachable} {at_safe}')
        
        # Update drone positions based on the current behavior
        if self.behavior_mode == "formation":
            self.update_formation_positions(elapsed)
        else:  # flocking
            self.update_flocking_positions(elapsed)
            
        # Publish obstacle markers
        for i, obstacle in enumerate(self.obstacles):
            obs_marker = self.create_obstacle_marker(i, obstacle, now)
            markers.markers.append(obs_marker)
            
        # Publish transforms and visualizations for each drone
        for i in range(self.num_drones):
            # Get the current position
            x, y, z = self.drone_positions[i]
            
            # Get velocity for orientation
            vx, vy, vz = self.drone_velocities[i]
            
            # Calculate yaw based on velocity direction
            yaw = math.atan2(vy, vx) if (abs(vx) > 0.01 or abs(vy) > 0.01) else 0.0
            
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
            
            # Different color for each drone, with visual indication for state
            if self.at_closest_safe_position[i]:
                # Bright orange for drones that stopped at closest safe position
                body.color.r = 1.0
                body.color.g = 0.6
                body.color.b = 0.0
            elif self.unreachable_targets[i]:
                # Amber color for drones with unreachable targets
                body.color.r = 1.0
                body.color.g = 0.75
                body.color.b = 0.0
            else:
                # Normal coloring
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
            
            # Create camera frustum
            frustum = Marker()
            frustum.header.frame_id = "world"
            frustum.header.stamp = now.to_msg()
            frustum.ns = "sensors"
            frustum.id = i*10 + 6
            frustum.type = Marker.LINE_LIST
            frustum.action = Marker.ADD
            
            # Camera is at the front of the drone
            # Calculate the camera position at the front of the drone
            cam_x = x + math.cos(yaw) * 0.2
            cam_y = y + math.sin(yaw) * 0.2
            cam_z = z
            
            # Define camera parameters
            fov_h = math.radians(60)  # 60 degree horizontal FOV
            fov_v = math.radians(45)  # 45 degree vertical FOV
            range_dist = 5.0  # 5 meter range
            
            # Calculate frustum corners
            corners = []
            for h_sign in [-1, 1]:
                for v_sign in [-1, 1]:
                    # Direction vectors for corners
                    dx = math.cos(yaw + h_sign * fov_h/2)
                    dy = math.sin(yaw + h_sign * fov_h/2)
                    dz = v_sign * math.tan(fov_v/2)
                    
                    # Normalize direction vector
                    mag = math.sqrt(dx*dx + dy*dy + dz*dz)
                    dx /= mag
                    dy /= mag
                    dz /= mag
                    
                    # Calculate corner position
                    corners.append((
                        cam_x + dx * range_dist,
                        cam_y + dy * range_dist,
                        cam_z + dz * range_dist
                    ))
                    
            # Create frustum lines
            frustum.points = []
            # Origin to corners
            for corner in corners:
                # Line from camera to corner
                p_start = Point()
                p_start.x = cam_x
                p_start.y = cam_y
                p_start.z = cam_z
                frustum.points.append(p_start)
                
                p_end = Point()
                p_end.x = corner[0]
                p_end.y = corner[1]
                p_end.z = corner[2]
                frustum.points.append(p_end)
                
            # Connect corners
            for i_corner in range(4):
                # Connect to next corner (in a loop)
                p_start = Point()
                p_start.x = corners[i_corner][0]
                p_start.y = corners[i_corner][1]
                p_start.z = corners[i_corner][2]
                frustum.points.append(p_start)
                
                next_i = (i_corner + 1) % 4
                p_end = Point()
                p_end.x = corners[next_i][0]
                p_end.y = corners[next_i][1]
                p_end.z = corners[next_i][2]
                frustum.points.append(p_end)
                
            # Set line properties
            frustum.scale.x = 0.02  # Line width
            frustum.color.r = 0.9
            frustum.color.g = 0.9
            frustum.color.b = 0.1
            frustum.color.a = 0.6
            
            markers.markers.append(frustum)
            
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
        
        # Publish all markers
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_markers)

    def update_formation_positions(self, dt):
        """Update drone positions to move toward formation targets while avoiding obstacles."""
        # Initialize unreachable state tracker if needed
        if not hasattr(self, 'closest_safe_positions'):
            self.closest_safe_positions = [None] * self.num_drones
            self.at_closest_safe_position = [False] * self.num_drones
        
        for i in range(self.num_drones):
            # Get target position from formation
            target = self.swarm_controller.get_formation_target(i)
            
            # Current position
            current_pos = self.drone_positions[i]
            
            # First, check if target position is reachable (not inside or too close to an obstacle)
            target_is_reachable = True
            blocking_obstacle = None
            
            # Check all obstacles
            for obstacle in self.obstacles:
                obs_x, obs_y, obs_z = obstacle[:3]
                obs_radius = obstacle[3]
                
                # Distance from target to obstacle center
                dx = target[0] - obs_x
                dy = target[1] - obs_y
                dz = target[2] - obs_z
                distance_to_obstacle = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Safety margin - must be clear of obstacle by this distance
                safety_margin = 0.5
                
                # If target is inside or too close to obstacle, it's unreachable
                if distance_to_obstacle < (obs_radius + safety_margin):
                    target_is_reachable = False
                    blocking_obstacle = obstacle
                    break
            
            # Update unreachable state
            self.unreachable_targets[i] = not target_is_reachable
            
            # If target is unreachable and we haven't calculated a closest safe position yet
            if not target_is_reachable and self.closest_safe_positions[i] is None:
                # Find closest safe position to target
                obs_x, obs_y, obs_z = blocking_obstacle[:3]
                obs_radius = blocking_obstacle[3]
                
                # Vector from obstacle to target
                vector_to_target = [
                    target[0] - obs_x,
                    target[1] - obs_y,
                    target[2] - obs_z
                ]
                
                # Distance from obstacle center to target
                distance = math.sqrt(vector_to_target[0]**2 + vector_to_target[1]**2 + vector_to_target[2]**2)
                
                # Normalize the vector
                if distance > 0:
                    vector_to_target = [
                        vector_to_target[0] / distance,
                        vector_to_target[1] / distance,
                        vector_to_target[2] / distance
                    ]
                
                # Calculate closest safe position (on the surface of the safety margin)
                safety_margin = 0.8  # Slightly larger margin for safety
                safe_distance = obs_radius + safety_margin
                
                closest_safe_pos = [
                    obs_x + vector_to_target[0] * safe_distance,
                    obs_y + vector_to_target[1] * safe_distance,
                    obs_z + vector_to_target[2] * safe_distance
                ]
                
                self.closest_safe_positions[i] = closest_safe_pos
                self.at_closest_safe_position[i] = False
                
                if self.debug:
                    self.get_logger().info(f"Drone {i}: Calculated closest safe position {closest_safe_pos} for unreachable target {target}")
            
            # For reachable targets, clear any stored closest safe position
            if target_is_reachable:
                self.closest_safe_positions[i] = None
                self.at_closest_safe_position[i] = False
                
            # If we have an unreachable target and we're already at the closest safe position, just hover
            if not target_is_reachable and self.at_closest_safe_position[i]:
                # Just hover at current position
                self.drone_velocities[i] = [0.0, 0.0, 0.0]
                continue
                
            # Choose the appropriate target
            actual_target = target
            if not target_is_reachable and self.closest_safe_positions[i] is not None:
                actual_target = self.closest_safe_positions[i]
            
            # Calculate direction to target
            dx = actual_target[0] - current_pos[0]
            dy = actual_target[1] - current_pos[1]
            dz = actual_target[2] - current_pos[2]
            
            # Distance to target
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # For unreachable targets, if we're close enough to closest safe position, mark as arrived and stop
            if not target_is_reachable and distance < 0.1:
                self.at_closest_safe_position[i] = True
                self.drone_velocities[i] = [0.0, 0.0, 0.0]
                continue
            
            # Debug output for first drone
            if i == 0 and self.debug and hasattr(self, 'debug_counter') and self.debug_counter % 20 == 0:
                reachable_status = "reachable" if target_is_reachable else "UNREACHABLE"
                target_type = "original" if target_is_reachable else "closest safe position"
                at_position = "" if target_is_reachable else f", at position: {self.at_closest_safe_position[i]}"
                self.get_logger().info(f'Drone 0 target: ({actual_target[0]:.2f}, {actual_target[1]:.2f}, {actual_target[2]:.2f}), distance: {distance:.2f}, {reachable_status}, using {target_type}{at_position}')
            
            if distance > 0.05:  # Only move if we're not very close
                # Calculate velocity based on distance (with a cap)
                speed = min(distance / 1.0, self.swarm_controller.max_speed)
                
                # Normalize direction
                if distance > 0:
                    dx /= distance
                    dy /= distance
                    dz /= distance
                
                # Base velocity toward target
                base_velocity = [dx * speed, dy * speed, dz * speed]
                
                # Initialize obstacle avoidance vector
                avoid_x, avoid_y, avoid_z = 0.0, 0.0, 0.0
                
                # Check for obstacles and calculate avoidance vector
                for obstacle in self.obstacles:
                    obs_x, obs_y, obs_z = obstacle[:3]
                    obs_radius = obstacle[3]
                    
                    # Vector from obstacle to drone
                    vec_x = current_pos[0] - obs_x
                    vec_y = current_pos[1] - obs_y
                    vec_z = current_pos[2] - obs_z
                    
                    # Distance to obstacle center
                    obs_distance = math.sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z)
                    
                    # Safety margin - stay this far away from obstacle surface
                    safety_margin = 0.5
                    
                    # If we're too close to the obstacle
                    if obs_distance < (obs_radius + safety_margin):
                        # Strength of avoidance (stronger when closer)
                        # Avoid division by zero
                        if obs_distance > 0.1:
                            avoid_strength = 3.0 * (1.0 - (obs_distance / (obs_radius + safety_margin)))
                            
                            # Normalize avoidance direction
                            norm = math.sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z)
                            if norm > 0:
                                vec_x /= norm
                                vec_y /= norm
                                vec_z /= norm
                                
                            # Add to avoidance vector
                            avoid_x += vec_x * avoid_strength
                            avoid_y += vec_y * avoid_strength
                            avoid_z += vec_z * avoid_strength
                
                # Normalize avoidance vector if it's not zero
                avoid_norm = math.sqrt(avoid_x*avoid_x + avoid_y*avoid_y + avoid_z*avoid_z)
                if avoid_norm > 0:
                    avoid_x /= avoid_norm
                    avoid_y /= avoid_norm
                    avoid_z /= avoid_norm
                    
                    # Set the avoidance vector to max speed to ensure high priority
                    avoid_x *= self.swarm_controller.max_speed * 1.5
                    avoid_y *= self.swarm_controller.max_speed * 1.5
                    avoid_z *= self.swarm_controller.max_speed * 1.5
                
                    # Bias heavily toward obstacle avoidance (80% avoidance, 20% formation)
                    self.drone_velocities[i] = [
                        base_velocity[0] * 0.2 + avoid_x * 0.8,
                        base_velocity[1] * 0.2 + avoid_y * 0.8,
                        base_velocity[2] * 0.2 + avoid_z * 0.8
                    ]
                else:
                    # No obstacles nearby, just use the formation velocity
                    self.drone_velocities[i] = base_velocity
            else:
                # We're close enough to target
                if not target_is_reachable:
                    self.at_closest_safe_position[i] = True
                
                # Still check for obstacles
                avoid_x, avoid_y, avoid_z = 0.0, 0.0, 0.0
                
                # Check for obstacles and calculate avoidance vector
                for obstacle in self.obstacles:
                    obs_x, obs_y, obs_z = obstacle[:3]
                    obs_radius = obstacle[3]
                    
                    # Vector from obstacle to drone
                    vec_x = current_pos[0] - obs_x
                    vec_y = current_pos[1] - obs_y
                    vec_z = current_pos[2] - obs_z
                    
                    # Distance to obstacle center
                    obs_distance = math.sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z)
                    
                    # Safety margin - stay this far away from obstacle surface
                    safety_margin = 0.5
                    
                    # If we're too close to the obstacle
                    if obs_distance < (obs_radius + safety_margin):
                        # Strength of avoidance (stronger when closer)
                        # Avoid division by zero
                        if obs_distance > 0.1:
                            avoid_strength = 3.0 * (1.0 - (obs_distance / (obs_radius + safety_margin)))
                            
                            # Normalize avoidance direction
                            norm = math.sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z)
                            if norm > 0:
                                vec_x /= norm
                                vec_y /= norm
                                vec_z /= norm
                                
                            # Add to avoidance vector
                            avoid_x += vec_x * avoid_strength
                            avoid_y += vec_y * avoid_strength
                            avoid_z += vec_z * avoid_strength
                
                # If there are nearby obstacles, move away
                avoid_norm = math.sqrt(avoid_x*avoid_x + avoid_y*avoid_y + avoid_z*avoid_z)
                if avoid_norm > 0:
                    # Normalize and scale to half max speed for gentler avoidance
                    avoid_x = (avoid_x / avoid_norm) * self.swarm_controller.max_speed * 0.5
                    avoid_y = (avoid_y / avoid_norm) * self.swarm_controller.max_speed * 0.5
                    avoid_z = (avoid_z / avoid_norm) * self.swarm_controller.max_speed * 0.5
                    
                    self.drone_velocities[i] = [avoid_x, avoid_y, avoid_z]
                else:
                    # No obstacles nearby, we can stop
                    self.drone_velocities[i] = [0.0, 0.0, 0.0]
            
            # Update position based on velocity and time step
            self.drone_positions[i] = [
                current_pos[0] + self.drone_velocities[i][0] * dt,
                current_pos[1] + self.drone_velocities[i][1] * dt,
                current_pos[2] + self.drone_velocities[i][2] * dt
            ]
    
    def update_flocking_positions(self, dt):
        """Update drone positions using Boids flocking algorithm."""
        # Calculate new velocities for all drones
        new_velocities = []
        
        for i in range(self.num_drones):
            # Get target position - use formation center as a loose target
            target = self.swarm_controller.formation_center
            
            # Apply Boids rules to get new velocity
            new_vel = self.swarm_controller.apply_boids_rules(
                i, 
                self.drone_positions,
                self.drone_velocities,
                self.obstacles,
                target
            )
            
            new_velocities.append(new_vel)
        
        # Update velocities
        self.drone_velocities = new_velocities
        
        # Update positions based on velocities
        for i in range(self.num_drones):
            self.drone_positions[i] = [
                self.drone_positions[i][0] + self.drone_velocities[i][0] * dt,
                self.drone_positions[i][1] + self.drone_velocities[i][1] * dt,
                self.drone_positions[i][2] + self.drone_velocities[i][2] * dt
            ]
    
    def create_obstacle_marker(self, obstacle_id, obstacle, now):
        """Create a marker for an obstacle."""
        x, y, z, radius = obstacle[:4]
        
        # Get color information if available (default to red if not)
        r, g, b = obstacle[4:7] if len(obstacle) >= 7 else (0.8, 0.2, 0.2)
        
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = now.to_msg()
        marker.ns = "obstacles"
        marker.id = obstacle_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.6
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    
    # Set logging level to DEBUG for more detailed output
    node = TFVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()