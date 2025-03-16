#!/usr/bin/env python3

import math
import numpy as np
from enum import Enum

class FormationType(Enum):
    NONE = 0
    CIRCLE = 1
    LINE = 2
    GRID = 3
    V_SHAPE = 4
    CUSTOM = 5

class SwarmController:
    """
    Controller for drone swarm behaviors including flocking and formations.
    """
    
    def __init__(self, num_drones=5):
        """Initialize the swarm controller."""
        self.num_drones = num_drones
        
        # Boids algorithm parameters
        self.separation_weight = 1.5
        self.alignment_weight = 1.0
        self.cohesion_weight = 1.0
        self.obstacle_avoidance_weight = 2.0
        self.target_weight = 0.8
        
        # Distances
        self.separation_distance = 1.0
        self.neighbor_distance = 3.0
        self.obstacle_avoid_distance = 2.0
        
        # Maximum speeds
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