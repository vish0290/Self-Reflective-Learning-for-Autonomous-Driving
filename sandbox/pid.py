#!/usr/bin/env python3
"""
PID Controller for Trajectory Following

This module provides a controller that follows 3D waypoints in CARLA.
Can be used for:
1. Data collection (following ground truth trajectory)
2. VLM inference (following predicted trajectory)

Implements both:
- Pure Pursuit: Geometric path following
- Stanley Controller: Heading + cross-track error
"""

import numpy as np
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Import our trajectory module
from traj_planner import Waypoint3D


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class ControllerConfig:
    """Controller configuration."""
    # Speed control
    target_speed_kmh: float = 30.0
    max_throttle: float = 0.8
    max_brake: float = 0.8
    
    # Steering control
    max_steering: float = 0.7
    
    # Pure Pursuit parameters
    lookahead_distance: float = 5.0  # meters
    min_lookahead: float = 2.0
    max_lookahead: float = 15.0
    lookahead_speed_factor: float = 0.5  # lookahead = base + speed * factor
    
    # Stanley parameters
    stanley_k: float = 0.5  # Cross-track error gain
    stanley_k_soft: float = 1.0  # Softening factor
    
    # PID gains for speed control
    speed_kp: float = 0.5
    speed_ki: float = 0.1
    speed_kd: float = 0.1


# =============================================================================
# VEHICLE CONTROL OUTPUT
# =============================================================================

@dataclass
class VehicleControl:
    """Vehicle control commands."""
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    
    def to_carla_control(self):
        """Convert to CARLA VehicleControl object."""
        import carla
        control = carla.VehicleControl()
        control.throttle = float(np.clip(self.throttle, 0.0, 1.0))
        control.brake = float(np.clip(self.brake, 0.0, 1.0))
        control.steer = float(np.clip(self.steer, -1.0, 1.0))
        return control


# =============================================================================
# PURE PURSUIT CONTROLLER
# =============================================================================

class PurePursuitController:
    """
    Pure Pursuit path following controller.
    
    Finds a lookahead point on the path and steers toward it.
    Simple and robust for smooth paths.
    """
    
    def __init__(self, config: ControllerConfig = None):
        self.config = config or ControllerConfig()
        
        # PID state for speed control
        self._speed_integral = 0.0
        self._speed_prev_error = 0.0
    
    def compute_control(self, waypoints: List[Waypoint3D],
                       vehicle_transform,
                       current_speed_kmh: float,
                       target_speed_kmh: float = None) -> VehicleControl:
        """
        Compute vehicle control to follow waypoints.
        
        Args:
            waypoints: List of 3D waypoints to follow
            vehicle_transform: Current vehicle transform
            current_speed_kmh: Current speed in km/h
            target_speed_kmh: Target speed (default from config)
        
        Returns:
            VehicleControl with throttle, brake, steer
        """
        if not waypoints or len(waypoints) < 2:
            # No path - stop
            return VehicleControl(throttle=0.0, brake=0.5, steer=0.0)
        
        target_speed = target_speed_kmh or self.config.target_speed_kmh
        
        # Get vehicle state
        v_loc = vehicle_transform.location
        v_rot = vehicle_transform.rotation
        vehicle_pos = np.array([v_loc.x, v_loc.y])
        vehicle_yaw = np.radians(v_rot.yaw)
        
        # Compute lookahead distance based on speed
        current_speed_ms = current_speed_kmh / 3.6
        lookahead = self.config.lookahead_distance + \
                   current_speed_ms * self.config.lookahead_speed_factor
        lookahead = np.clip(lookahead, self.config.min_lookahead, 
                           self.config.max_lookahead)
        
        # Find lookahead point
        lookahead_point = self._find_lookahead_point(waypoints, vehicle_pos, lookahead)
        
        if lookahead_point is None:
            # Use last waypoint
            lookahead_point = np.array([waypoints[-1].x, waypoints[-1].y])
        
        # Compute steering using Pure Pursuit geometry
        steer = self._compute_steering(vehicle_pos, vehicle_yaw, 
                                       lookahead_point, lookahead)
        
        # Compute throttle/brake using PID
        throttle, brake = self._compute_speed_control(
            current_speed_kmh, target_speed
        )
        
        return VehicleControl(
            throttle=throttle,
            brake=brake,
            steer=steer
        )
    
    def _find_lookahead_point(self, waypoints: List[Waypoint3D],
                              vehicle_pos: np.ndarray,
                              lookahead: float) -> Optional[np.ndarray]:
        """Find point on path at lookahead distance."""
        
        # Convert waypoints to numpy array
        path = np.array([[wp.x, wp.y] for wp in waypoints])
        
        # Find distances from vehicle to each waypoint
        distances = np.linalg.norm(path - vehicle_pos, axis=1)
        
        # Find first point beyond lookahead distance
        for i in range(len(distances)):
            if distances[i] >= lookahead:
                if i == 0:
                    return path[0]
                
                # Interpolate between waypoints i-1 and i
                d_prev = distances[i - 1]
                d_curr = distances[i]
                
                # Linear interpolation factor
                t = (lookahead - d_prev) / (d_curr - d_prev + 1e-6)
                t = np.clip(t, 0.0, 1.0)
                
                point = path[i - 1] + t * (path[i] - path[i - 1])
                return point
        
        # All points are within lookahead - return last point
        return path[-1]
    
    def _compute_steering(self, vehicle_pos: np.ndarray,
                         vehicle_yaw: float,
                         target_point: np.ndarray,
                         lookahead: float) -> float:
        """
        Compute steering angle using Pure Pursuit formula.
        
        steering = atan(2 * L * sin(alpha) / ld)
        where:
            L = wheelbase (approximated)
            alpha = angle to target point relative to vehicle heading
            ld = lookahead distance
        """
        # Vector to target
        to_target = target_point - vehicle_pos
        
        # Angle to target in world frame
        target_angle = np.arctan2(to_target[1], to_target[0])
        
        # Angle relative to vehicle heading
        alpha = target_angle - vehicle_yaw
        
        # Normalize to [-pi, pi]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        # Pure Pursuit steering formula
        # Assuming wheelbase L ≈ 2.5m for typical car
        wheelbase = 2.5
        ld = max(lookahead, 0.1)
        
        steering = np.arctan2(2.0 * wheelbase * np.sin(alpha), ld)
        
        # Normalize to [-1, 1]
        steering = steering / np.radians(70)  # Assume max steering angle ~70°
        steering = np.clip(steering, -self.config.max_steering, 
                          self.config.max_steering)
        
        return float(-steering)
    
    def _compute_speed_control(self, current_speed: float, 
                               target_speed: float) -> Tuple[float, float]:
        """PID speed control."""
        error = target_speed - current_speed
        
        # PID terms
        self._speed_integral += error * 0.033  # Assuming ~30 FPS
        self._speed_integral = np.clip(self._speed_integral, -10, 10)
        
        derivative = (error - self._speed_prev_error) / 0.033
        self._speed_prev_error = error
        
        # PID output
        output = (self.config.speed_kp * error +
                 self.config.speed_ki * self._speed_integral +
                 self.config.speed_kd * derivative)
        
        # Convert to throttle/brake
        if output > 0:
            throttle = np.clip(output / 50.0, 0.0, self.config.max_throttle)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-output / 50.0, 0.0, self.config.max_brake)
        
        return float(throttle), float(brake)
    
    def reset(self):
        """Reset controller state."""
        self._speed_integral = 0.0
        self._speed_prev_error = 0.0


# =============================================================================
# STANLEY CONTROLLER
# =============================================================================

class StanleyController:
    """
    Stanley path following controller.
    
    Combines heading error and cross-track error for more precise following.
    Better for sharp turns but can be oscillatory.
    """
    
    def __init__(self, config: ControllerConfig = None):
        self.config = config or ControllerConfig()
        self._speed_integral = 0.0
        self._speed_prev_error = 0.0
    
    def compute_control(self, waypoints: List[Waypoint3D],
                       vehicle_transform,
                       current_speed_kmh: float,
                       target_speed_kmh: float = None) -> VehicleControl:
        """Compute control using Stanley method."""
        
        if not waypoints or len(waypoints) < 2:
            return VehicleControl(throttle=0.0, brake=0.5, steer=0.0)
        
        target_speed = target_speed_kmh or self.config.target_speed_kmh
        
        # Get vehicle state
        v_loc = vehicle_transform.location
        v_rot = vehicle_transform.rotation
        vehicle_pos = np.array([v_loc.x, v_loc.y])
        vehicle_yaw = np.radians(v_rot.yaw)
        
        # Find nearest path point and heading
        nearest_idx, cross_track_error = self._find_nearest_point(
            waypoints, vehicle_pos
        )
        
        # Get path heading at nearest point
        path_heading = self._get_path_heading(waypoints, nearest_idx)
        
        # Heading error
        heading_error = path_heading - vehicle_yaw
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Stanley steering formula
        current_speed_ms = max(current_speed_kmh / 3.6, 0.1)
        
        cte_term = np.arctan2(
            self.config.stanley_k * cross_track_error,
            self.config.stanley_k_soft + current_speed_ms
        )
        
        steering = heading_error + cte_term
        steering = steering / np.radians(70)
        steering = np.clip(steering, -self.config.max_steering,
                          self.config.max_steering)
        
        # Speed control (same as Pure Pursuit)
        throttle, brake = self._compute_speed_control(
            current_speed_kmh, target_speed
        )
        
        return VehicleControl(
            throttle=throttle,
            brake=brake,
            steer=float(steering)
        )
    
    def _find_nearest_point(self, waypoints: List[Waypoint3D],
                           vehicle_pos: np.ndarray) -> Tuple[int, float]:
        """Find nearest waypoint and cross-track error."""
        path = np.array([[wp.x, wp.y] for wp in waypoints])
        distances = np.linalg.norm(path - vehicle_pos, axis=1)
        nearest_idx = np.argmin(distances)
        
        # Compute signed cross-track error
        if nearest_idx < len(waypoints) - 1:
            # Vector along path
            p1 = path[nearest_idx]
            p2 = path[nearest_idx + 1]
            path_vec = p2 - p1
            
            # Vector from path point to vehicle
            to_vehicle = vehicle_pos - p1
            
            # Cross product for signed distance
            cross = path_vec[0] * to_vehicle[1] - path_vec[1] * to_vehicle[0]
            path_len = np.linalg.norm(path_vec)
            
            if path_len > 0:
                cross_track_error = cross / path_len
            else:
                cross_track_error = distances[nearest_idx]
        else:
            cross_track_error = distances[nearest_idx]
        
        return nearest_idx, cross_track_error
    
    def _get_path_heading(self, waypoints: List[Waypoint3D],
                         idx: int) -> float:
        """Get path heading at given index."""
        if idx >= len(waypoints) - 1:
            idx = len(waypoints) - 2
        
        p1 = np.array([waypoints[idx].x, waypoints[idx].y])
        p2 = np.array([waypoints[idx + 1].x, waypoints[idx + 1].y])
        
        diff = p2 - p1
        return np.arctan2(diff[1], diff[0])
    
    def _compute_speed_control(self, current_speed: float,
                               target_speed: float) -> Tuple[float, float]:
        """Same PID speed control as Pure Pursuit."""
        error = target_speed - current_speed
        
        self._speed_integral += error * 0.033
        self._speed_integral = np.clip(self._speed_integral, -10, 10)
        
        derivative = (error - self._speed_prev_error) / 0.033
        self._speed_prev_error = error
        
        output = (self.config.speed_kp * error +
                 self.config.speed_ki * self._speed_integral +
                 self.config.speed_kd * derivative)
        
        if output > 0:
            throttle = np.clip(output / 50.0, 0.0, self.config.max_throttle)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-output / 50.0, 0.0, self.config.max_brake)
        
        return float(throttle), float(brake)
    
    def reset(self):
        """Reset controller state."""
        self._speed_integral = 0.0
        self._speed_prev_error = 0.0


# =============================================================================
# SIMPLE WAYPOINT FOLLOWER
# =============================================================================

class SimpleWaypointFollower:
    """
    Simple controller that just steers toward the next waypoint.
    
    Good for testing, less smooth than Pure Pursuit/Stanley.
    """
    
    def __init__(self, target_speed_kmh: float = 30.0):
        self.target_speed = target_speed_kmh
        self._speed_integral = 0.0
    
    def compute_control(self, waypoints: List[Waypoint3D],
                       vehicle_transform,
                       current_speed_kmh: float) -> VehicleControl:
        """Compute control toward first waypoint."""
        
        if not waypoints:
            return VehicleControl(throttle=0.0, brake=0.5, steer=0.0)
        
        # Target is first waypoint
        target = waypoints[0]
        
        # Vehicle state
        v_loc = vehicle_transform.location
        v_rot = vehicle_transform.rotation
        
        # Angle to target
        dx = target.x - v_loc.x
        dy = target.y - v_loc.y
        target_angle = np.degrees(np.arctan2(dy, dx))
        
        # Angle error
        angle_error = target_angle - v_rot.yaw
        
        # Normalize to [-180, 180]
        while angle_error > 180:
            angle_error -= 360
        while angle_error < -180:
            angle_error += 360
        
        # Simple proportional steering
        steer = np.clip(angle_error / 45.0, -1.0, 1.0)
        
        # Simple speed control
        speed_error = self.target_speed - current_speed_kmh
        self._speed_integral += speed_error * 0.033
        self._speed_integral = np.clip(self._speed_integral, -5, 5)
        
        throttle_cmd = 0.3 * speed_error + 0.1 * self._speed_integral
        
        if throttle_cmd > 0:
            throttle = np.clip(throttle_cmd / 20.0, 0.0, 0.8)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-throttle_cmd / 20.0, 0.0, 0.5)
        
        return VehicleControl(
            throttle=float(throttle),
            brake=float(brake),
            steer=float(steer)
        )


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    print("PID Controller Module")
    print("=" * 50)
    print("\nAvailable controllers:")
    print("  1. PurePursuitController - Geometric path following")
    print("  2. StanleyController - Heading + cross-track error")
    print("  3. SimpleWaypointFollower - Basic steering to waypoint")
    
    # Create test waypoints
    test_waypoints = [
        Waypoint3D(x=5.0, y=0.0, z=0.0),
        Waypoint3D(x=10.0, y=-1.0, z=0.0),
        Waypoint3D(x=15.0, y=-3.0, z=0.0),
        Waypoint3D(x=20.0, y=-5.0, z=0.0),
    ]
    
    print(f"\nTest waypoints (curving left):")
    for i, wp in enumerate(test_waypoints):
        print(f"  {i+1}. ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
    
    # Mock vehicle transform
    class MockTransform:
        class Location:
            x, y, z = 0.0, 0.0, 0.5
        class Rotation:
            pitch, yaw, roll = 0.0, 0.0, 0.0
        location = Location()
        rotation = Rotation()
    
    vehicle_transform = MockTransform()
    current_speed = 25.0  # km/h
    
    # Test Pure Pursuit
    print(f"\nPure Pursuit Controller:")
    pp = PurePursuitController()
    control = pp.compute_control(test_waypoints, vehicle_transform, current_speed)
    print(f"  Throttle: {control.throttle:.3f}")
    print(f"  Brake:    {control.brake:.3f}")
    print(f"  Steer:    {control.steer:.3f}")
    
    # Test Stanley
    print(f"\nStanley Controller:")
    stanley = StanleyController()
    control = stanley.compute_control(test_waypoints, vehicle_transform, current_speed)
    print(f"  Throttle: {control.throttle:.3f}")
    print(f"  Brake:    {control.brake:.3f}")
    print(f"  Steer:    {control.steer:.3f}")