#!/usr/bin/env python3
"""
Trajectory Projection Module

This module provides:
1. WaypointGenerator - Get 3D waypoints along the road
2. TrajectoryEncoder - Project 3D waypoints to 2D pixels
3. TrajectoryDecoder - Decode 2D pixels back to 3D world coordinates
4. Camera utilities - Intrinsics and transforms

The key insight: 2D pixels can be decoded back to 3D by assuming 
the points lie on the road plane (z = road_height).
"""

import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class CameraConfig:
    """Camera configuration."""
    width: int = 640
    height: int = 480
    fov: float = 90.0  # degrees
    
    # Position relative to vehicle
    x: float = 2.0   # Forward
    y: float = 0.0   # Lateral (0 = center)
    z: float = 1.8   # Height
    
    # Rotation
    pitch: float = -15.0  # Tilt down (negative = look down)
    yaw: float = 0.0
    roll: float = 0.0
    
    @property
    def fx(self) -> float:
        """Focal length in pixels (x)."""
        return self.width / (2.0 * np.tan(np.radians(self.fov / 2.0)))
    
    @property
    def fy(self) -> float:
        """Focal length in pixels (y)."""
        return self.fx  # Square pixels
    
    @property
    def cx(self) -> float:
        """Principal point x."""
        return self.width / 2.0
    
    @property
    def cy(self) -> float:
        """Principal point y."""
        return self.height / 2.0
    
    @property
    def intrinsic_matrix(self) -> np.ndarray:
        """3x3 camera intrinsic matrix."""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])


@dataclass 
class Waypoint3D:
    """3D waypoint in world coordinates."""
    x: float
    y: float
    z: float
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


@dataclass
class Waypoint2D:
    """2D waypoint in pixel coordinates."""
    u: float  # horizontal (0 = left)
    v: float  # vertical (0 = top)

    def to_array(self) -> np.ndarray:
        return np.array([self.u, self.v])

    def to_list(self) -> List[int]:
        return [int(self.u), int(self.v)]


@dataclass
class DrivableCorridor:
    """Drivable corridor boundaries extracted from the HD map."""
    left: List[Waypoint3D]     # Left lane boundary (world coords)
    right: List[Waypoint3D]    # Right lane boundary (world coords)
    center: List[Waypoint3D]   # Centerline
    adj_left: Optional[List[Waypoint3D]] = None   # Adjacent left lane outer edge
    adj_right: Optional[List[Waypoint3D]] = None   # Adjacent right lane outer edge


# =============================================================================
# ROTATION UTILITIES
# =============================================================================

def rotation_matrix_from_euler(pitch: float, yaw: float, roll: float) -> np.ndarray:
    """
    Create rotation matrix from Euler angles (in degrees).
    
    CARLA uses: X-forward, Y-right, Z-up
    Rotation order: Yaw (Z) -> Pitch (Y) -> Roll (X)
    """
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    roll = np.radians(roll)
    
    # Rotation around Z (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Rotation around Y (pitch)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation around X (roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Combined rotation: Rz * Ry * Rx
    return Rz @ Ry @ Rx


def carla_to_camera_frame() -> np.ndarray:
    """
    Transform from CARLA world frame to camera/OpenCV frame.
    
    CARLA:  X-forward, Y-right, Z-up
    OpenCV: X-right, Y-down, Z-forward
    """
    return np.array([
        [0, 1, 0],   # Camera X = CARLA Y (right)
        [0, 0, -1],  # Camera Y = -CARLA Z (down)
        [1, 0, 0]    # Camera Z = CARLA X (forward)
    ])


# =============================================================================
# WAYPOINT GENERATOR
# =============================================================================

class WaypointGenerator:
    """
    Generate 3D waypoints along the road.

    Uses CARLA map API to get waypoints following road topology.
    Supports random route selection for diverse data collection.
    """

    def __init__(self, carla_map, distances: List[float] = None, random_route: bool = False):
        """
        Args:
            carla_map: CARLA map object
            distances: Distances in meters for waypoints [2, 4, 6, 8, 10]
            random_route: If True, randomly select route at intersections
        """
        self.map = carla_map
        self.distances = distances or [2.0, 4.0, 6.0, 8.0, 10.0]
        self.random_route = random_route
        self._current_route_choice = None  # Persist choice for consistency
        self._last_junction_id = None

    def set_random_route(self, enabled: bool):
        """Enable/disable random route selection."""
        self.random_route = enabled

    def get_waypoints(self, vehicle) -> List[Waypoint3D]:
        """
        Get waypoints at fixed distances ahead of vehicle.

        Returns list of Waypoint3D following the road.
        If random_route is enabled, randomly selects at intersections.
        """
        import random

        # Get current vehicle waypoint
        vehicle_location = vehicle.get_transform().location
        current_wp = self.map.get_waypoint(vehicle_location)

        if current_wp is None:
            return []

        waypoints = []
        tracking_wp = current_wp

        for i, distance in enumerate(self.distances):
            # Compute incremental step (distances are absolute from vehicle)
            step = distance if i == 0 else distance - self.distances[i - 1]
            next_wps = tracking_wp.next(step)

            if next_wps:
                if len(next_wps) > 1 and self.random_route:
                    # Multiple options (intersection) - randomly select
                    # Check if we're at a new junction or no choice made yet
                    junction_id = tracking_wp.junction_id if tracking_wp.is_junction else None
                    if junction_id != self._last_junction_id or self._current_route_choice is None:
                        # New junction or first time, make a new random choice
                        self._current_route_choice = random.randint(0, len(next_wps) - 1)
                        self._last_junction_id = junction_id

                    # Use the route choice, clamped to valid range
                    choice_idx = min(self._current_route_choice, len(next_wps) - 1)
                    wp = next_wps[choice_idx]
                else:
                    # Single option or random disabled - take first
                    wp = next_wps[0]

                waypoints.append(Waypoint3D(
                    x=wp.transform.location.x,
                    y=wp.transform.location.y,
                    z=wp.transform.location.z
                ))
                tracking_wp = wp  # Continue from this waypoint
            else:
                # Road ends, extrapolate from last known waypoint
                if waypoints:
                    waypoints.append(waypoints[-1])

        return waypoints

    def reset_route_choice(self):
        """Reset route choice (call after respawn)."""
        self._current_route_choice = None
        self._last_junction_id = None

    def get_navigation_from_waypoints(self, waypoints: List[Waypoint3D], vehicle) -> Optional[str]:
        """
        Derive navigation direction from waypoints trajectory.

        Compares vehicle heading vs direction to furthest waypoint.
        Returns: "go straight", "turn left", "turn right", or None
        """
        if len(waypoints) < 2:
            return None

        # Get vehicle heading
        vehicle_transform = vehicle.get_transform()
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)

        # Get direction from vehicle to last waypoint
        vehicle_loc = vehicle_transform.location
        last_wp = waypoints[-1]

        dx = last_wp.x - vehicle_loc.x
        dy = last_wp.y - vehicle_loc.y

        # Calculate angle to target waypoint
        target_angle = math.atan2(dy, dx)

        # Angle difference (normalized to -pi to pi)
        angle_diff = target_angle - vehicle_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Threshold for turn detection (about 20 degrees)
        turn_threshold = 0.35  # radians (~20 degrees)

        if angle_diff > turn_threshold:
            return "turn right"
        elif angle_diff < -turn_threshold:
            return "turn left"
        else:
            return "go straight"

    def get_road_waypoints(self, vehicle, max_distance: float = 35.0,
                           num_output: int = 15) -> List[Waypoint3D]:
        """
        Generate curvature-adaptive waypoints following the road.

        Densely samples the road centerline, then selects points based
        on local heading change — more points on curves, fewer on
        straights.  The result captures the actual road shape.

        Args:
            vehicle: CARLA vehicle actor
            max_distance: How far ahead to look (meters)
            num_output: Target number of output waypoints

        Returns:
            List of Waypoint3D following the road geometry
        """
        import random as _random

        vehicle_location = vehicle.get_transform().location
        current_wp = self.map.get_waypoint(vehicle_location)

        if current_wp is None:
            return []

        # --- Step 1: dense 1-meter sampling along the road -----------
        sample_step = 1.0
        dense = []
        wp = current_wp
        traveled = 0.0

        while traveled < max_distance and wp is not None:
            loc = wp.transform.location
            heading = math.radians(wp.transform.rotation.yaw)
            dense.append((loc.x, loc.y, loc.z, heading))

            next_wps = wp.next(sample_step)
            if not next_wps:
                break

            # Handle intersections
            if len(next_wps) > 1 and self.random_route:
                junction_id = wp.junction_id if wp.is_junction else None
                if junction_id != self._last_junction_id or self._current_route_choice is None:
                    self._current_route_choice = _random.randint(0, len(next_wps) - 1)
                    self._last_junction_id = junction_id
                idx = min(self._current_route_choice, len(next_wps) - 1)
                wp = next_wps[idx]
            else:
                wp = next_wps[0]

            traveled += sample_step

        if len(dense) < 3:
            return [Waypoint3D(x=d[0], y=d[1], z=d[2]) for d in dense]

        # --- Step 2: compute importance score per sample point -------
        # Importance = |heading change| from previous point.
        # Straight segments get low scores, curves get high scores.
        scores = [0.0] * len(dense)
        for i in range(1, len(dense)):
            dh = dense[i][3] - dense[i - 1][3]
            # normalize to [-pi, pi]
            dh = (dh + math.pi) % (2 * math.pi) - math.pi
            scores[i] = abs(dh)

        # --- Step 3: select waypoints weighted by importance ---------
        # Always include first and last point.
        # Fill the rest by greedily picking the highest-importance
        # points that are at least min_gap apart.
        min_gap = max(1, len(dense) // (num_output * 2))

        selected = {0, len(dense) - 1}
        remaining = num_output - 2

        # Build candidate list sorted by score (descending)
        candidates = sorted(range(1, len(dense) - 1),
                            key=lambda i: scores[i], reverse=True)

        for idx in candidates:
            if remaining <= 0:
                break
            # Enforce minimum gap from already-selected points
            if all(abs(idx - s) >= min_gap for s in selected):
                selected.add(idx)
                remaining -= 1

        # If we still need points (very straight road), fill evenly
        if remaining > 0:
            even_indices = np.linspace(0, len(dense) - 1,
                                       num_output).astype(int)
            for idx in even_indices:
                if remaining <= 0:
                    break
                if idx not in selected:
                    selected.add(int(idx))
                    remaining -= 1

        # Sort by path order and build output
        selected_sorted = sorted(selected)
        return [Waypoint3D(x=dense[i][0], y=dense[i][1], z=dense[i][2])
                for i in selected_sorted]

    def get_drivable_corridor(self, vehicle, max_distance: float = 40.0,
                              sample_step: float = 1.5,
                              include_adjacent: bool = True) -> DrivableCorridor:
        """
        Generate the drivable corridor (like an HD map / BEV segment).

        Samples the road ahead and computes left/right lane boundaries
        from the CARLA map's lane width and heading at each point.
        Optionally includes adjacent same-direction lanes.

        Args:
            vehicle: CARLA vehicle actor
            max_distance: How far ahead to sample (meters)
            sample_step: Distance between samples (meters)
            include_adjacent: Also return adjacent lane boundaries

        Returns:
            DrivableCorridor with left/right/center boundary point lists
        """
        import random as _random

        vehicle_location = vehicle.get_transform().location
        current_wp = self.map.get_waypoint(vehicle_location)

        if current_wp is None:
            return DrivableCorridor([], [], [])

        # --- Sample centerline + extract lane geometry at each point ---
        center_pts = []
        left_pts = []
        right_pts = []
        adj_left_pts = []
        adj_right_pts = []

        wp = current_wp
        traveled = 0.0

        while traveled < max_distance and wp is not None:
            loc = wp.transform.location
            yaw_rad = math.radians(wp.transform.rotation.yaw)
            half_w = wp.lane_width / 2.0

            # Right-perpendicular direction in CARLA (X-fwd, Y-right)
            # Forward: (cos(yaw), sin(yaw))
            # Right:   (-sin(yaw), cos(yaw))  [from rotation matrix column 2]
            rx = -math.sin(yaw_rad) * half_w
            ry = math.cos(yaw_rad) * half_w

            center_pts.append(Waypoint3D(x=loc.x, y=loc.y, z=loc.z))
            left_pts.append(Waypoint3D(x=loc.x - rx, y=loc.y - ry, z=loc.z))
            right_pts.append(Waypoint3D(x=loc.x + rx, y=loc.y + ry, z=loc.z))

            # Adjacent lanes
            if include_adjacent:
                # Left adjacent lane
                left_lane = wp.get_left_lane()
                if (left_lane is not None
                        and left_lane.lane_type.name == 'Driving'
                        and str(left_lane.lane_id)[0] != '-'
                            if wp.lane_id > 0
                            else str(left_lane.lane_id)[0] == '-'):
                    # Same direction lane exists on the left
                    ll = left_lane.transform.location
                    adj_half = left_lane.lane_width / 2.0
                    ll_yaw = math.radians(left_lane.transform.rotation.yaw)
                    lrx = -math.sin(ll_yaw) * adj_half
                    lry = math.cos(ll_yaw) * adj_half
                    adj_left_pts.append(
                        Waypoint3D(x=ll.x - lrx, y=ll.y - lry, z=ll.z))
                else:
                    # No adjacent lane — use current left boundary
                    adj_left_pts.append(left_pts[-1])

                # Right adjacent lane
                right_lane = wp.get_right_lane()
                if (right_lane is not None
                        and right_lane.lane_type.name == 'Driving'
                        and str(right_lane.lane_id)[0] == '-'
                            if wp.lane_id > 0
                            else str(right_lane.lane_id)[0] != '-'):
                    rl = right_lane.transform.location
                    adj_half = right_lane.lane_width / 2.0
                    rl_yaw = math.radians(right_lane.transform.rotation.yaw)
                    rrx = -math.sin(rl_yaw) * adj_half
                    rry = math.cos(rl_yaw) * adj_half
                    adj_right_pts.append(
                        Waypoint3D(x=rl.x + rrx, y=rl.y + rry, z=rl.z))
                else:
                    adj_right_pts.append(right_pts[-1])

            # Advance along the road
            next_wps = wp.next(sample_step)
            if not next_wps:
                break

            if len(next_wps) > 1 and self.random_route:
                junction_id = wp.junction_id if wp.is_junction else None
                if junction_id != self._last_junction_id or self._current_route_choice is None:
                    self._current_route_choice = _random.randint(0, len(next_wps) - 1)
                    self._last_junction_id = junction_id
                idx = min(self._current_route_choice, len(next_wps) - 1)
                wp = next_wps[idx]
            else:
                wp = next_wps[0]

            traveled += sample_step

        return DrivableCorridor(
            left=left_pts,
            right=right_pts,
            center=center_pts,
            adj_left=adj_left_pts if include_adjacent else None,
            adj_right=adj_right_pts if include_adjacent else None,
        )


# =============================================================================
# TRAJECTORY ENCODER (3D → 2D)
# =============================================================================

class TrajectoryEncoder:
    """
    Project 3D world waypoints to 2D pixel coordinates.
    """
    
    def __init__(self, camera_config: CameraConfig):
        self.config = camera_config
        self._carla_to_cv = carla_to_camera_frame()
    
    def encode(self, waypoints_3d: List[Waypoint3D], 
               vehicle_transform) -> List[Waypoint2D]:
        """
        Project 3D waypoints to 2D pixels.
        
        Args:
            waypoints_3d: List of 3D waypoints in world coordinates
            vehicle_transform: CARLA vehicle transform
        
        Returns:
            List of 2D waypoints in pixel coordinates
        """
        waypoints_2d, _ = self.encode_with_indices(waypoints_3d, vehicle_transform)
        return waypoints_2d
    
    def encode_with_indices(self, waypoints_3d: List[Waypoint3D],
                           vehicle_transform) -> Tuple[List[Waypoint2D], List[int]]:
        """
        Project 3D waypoints to 2D pixels, tracking which indices are kept.
        
        Args:
            waypoints_3d: List of 3D waypoints in world coordinates
            vehicle_transform: CARLA vehicle transform
        
        Returns:
            Tuple of (2D waypoints, list of original indices that were kept)
        """
        if not waypoints_3d:
            return [], []
        
        # Get camera transform in world frame
        cam_location, cam_rotation = self._get_camera_world_transform(vehicle_transform)
        
        # Build camera rotation matrix (world to camera)
        R_world_to_cam = self._build_rotation_matrix(cam_rotation)
        
        waypoints_2d = []
        kept_indices = []
        
        for idx, wp_3d in enumerate(waypoints_3d):
            # Transform world point to camera frame
            p_world = wp_3d.to_array()
            p_rel = p_world - cam_location
            p_cam = R_world_to_cam @ p_rel
            
            # Check if point is in front of camera
            if p_cam[2] <= 0.1:  # Behind or too close
                continue
            
            # Project to pixel coordinates
            u = self.config.fx * (p_cam[0] / p_cam[2]) + self.config.cx
            v = self.config.fy * (p_cam[1] / p_cam[2]) + self.config.cy
            
            # Check if within image bounds
            if 0 <= u < self.config.width and 0 <= v < self.config.height:
                waypoints_2d.append(Waypoint2D(u=u, v=v))
                kept_indices.append(idx)
        
        return waypoints_2d, kept_indices
    
    def _get_camera_world_transform(self, vehicle_transform):
        """Get camera position and rotation matrix in world frame."""
        v_loc = vehicle_transform.location
        v_rot = vehicle_transform.rotation

        # Camera offset in vehicle frame
        cam_offset = np.array([self.config.x, self.config.y, self.config.z])

        # Rotate camera offset by vehicle rotation
        R_vehicle = rotation_matrix_from_euler(v_rot.pitch, v_rot.yaw, v_rot.roll)
        cam_offset_world = R_vehicle @ cam_offset

        # Camera location in world
        cam_location = np.array([v_loc.x, v_loc.y, v_loc.z]) + cam_offset_world

        # Camera rotation: compose via matrix multiplication (not Euler addition)
        R_camera_offset = rotation_matrix_from_euler(
            self.config.pitch, self.config.yaw, self.config.roll
        )
        R_combined = R_vehicle @ R_camera_offset

        return cam_location, R_combined

    def _build_rotation_matrix(self, R_combined: np.ndarray) -> np.ndarray:
        """Build world-to-camera rotation matrix."""
        return self._carla_to_cv @ R_combined.T


# =============================================================================
# TRAJECTORY DECODER (2D → 3D)
# =============================================================================

class TrajectoryDecoder:
    """
    Decode 2D pixel waypoints back to 3D world coordinates.
    
    Assumes all points lie on the road plane (flat road).
    """
    
    def __init__(self, camera_config: CameraConfig):
        self.config = camera_config
        self._carla_to_cv = carla_to_camera_frame()
        self._cv_to_carla = self._carla_to_cv.T  # Inverse
    
    def decode(self, waypoints_2d: List[Waypoint2D],
               vehicle_transform,
               road_height: float = None,
               debug: bool = False) -> List[Waypoint3D]:
        """
        Decode 2D pixel waypoints to 3D world coordinates.

        Args:
            waypoints_2d: List of 2D waypoints in pixel coordinates
            vehicle_transform: CARLA vehicle transform
            road_height: Z-coordinate of road plane (default: vehicle height)
            debug: Print debug info

        Returns:
            List of 3D waypoints in world coordinates
        """
        if not waypoints_2d:
            return []

        # Get camera transform
        cam_location, cam_rotation = self._get_camera_world_transform(vehicle_transform)

        # Road height defaults to slightly below vehicle
        if road_height is None:
            road_height = vehicle_transform.location.z - 0.5

        # Build rotation matrix (camera to world)
        R_cam_to_world = self._build_rotation_matrix(cam_rotation)

        if debug:
            print(f"[DECODER] cam_loc={cam_location}, cam_rot={cam_rotation}")
            print(f"[DECODER] road_height={road_height}")
            print(f"[DECODER] cx={self.config.cx}, cy={self.config.cy}, fx={self.config.fx}")

        waypoints_3d = []

        for i, wp_2d in enumerate(waypoints_2d):
            # Convert pixel to camera ray direction
            # u increases rightward, v increases downward
            # In camera frame: x=right, y=down, z=forward
            ray_cam = np.array([
                (wp_2d.u - self.config.cx) / self.config.fx,
                (wp_2d.v - self.config.cy) / self.config.fy,
                1.0
            ])

            if debug and i < 3:
                print(f"[DECODER] wp{i}: pixel=({wp_2d.u:.0f}, {wp_2d.v:.0f}), ray_cam={ray_cam}")

            # Normalize ray direction
            ray_cam = ray_cam / np.linalg.norm(ray_cam)

            # Transform ray to world frame
            ray_world = R_cam_to_world @ ray_cam

            if debug and i < 3:
                print(f"[DECODER] wp{i}: ray_world={ray_world}")

            # Ray-plane intersection
            # Plane: z = road_height
            # Ray: P = cam_location + t * ray_world
            # Solve: cam_location.z + t * ray_world.z = road_height

            if abs(ray_world[2]) < 1e-6:
                # Ray parallel to road plane
                continue

            t = (road_height - cam_location[2]) / ray_world[2]

            if t < 0:
                # Intersection behind camera
                continue

            # Calculate intersection point
            p_world = cam_location + t * ray_world

            if debug and i < 3:
                v_loc = vehicle_transform.location
                print(f"[DECODER] wp{i}: t={t:.2f}, p_world=({p_world[0]:.1f}, {p_world[1]:.1f}), rel=({p_world[0]-v_loc.x:.1f}, {p_world[1]-v_loc.y:.1f})")

            waypoints_3d.append(Waypoint3D(
                x=p_world[0],
                y=p_world[1],
                z=road_height
            ))

        return waypoints_3d
    
    def _get_camera_world_transform(self, vehicle_transform):
        """Get camera position and rotation matrix in world frame."""
        v_loc = vehicle_transform.location
        v_rot = vehicle_transform.rotation

        cam_offset = np.array([self.config.x, self.config.y, self.config.z])
        R_vehicle = rotation_matrix_from_euler(v_rot.pitch, v_rot.yaw, v_rot.roll)
        cam_offset_world = R_vehicle @ cam_offset

        cam_location = np.array([v_loc.x, v_loc.y, v_loc.z]) + cam_offset_world

        # Camera rotation: compose via matrix multiplication (not Euler addition)
        R_camera_offset = rotation_matrix_from_euler(
            self.config.pitch, self.config.yaw, self.config.roll
        )
        R_combined = R_vehicle @ R_camera_offset

        return cam_location, R_combined

    def _build_rotation_matrix(self, R_combined: np.ndarray) -> np.ndarray:
        """Build camera-to-world rotation matrix (inverse of encoder)."""
        return R_combined @ self._carla_to_cv.T


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def waypoints_to_pixel_list(waypoints_2d: List[Waypoint2D]) -> List[List[int]]:
    """Convert waypoints to list format for JSON/training data."""
    return [wp.to_list() for wp in waypoints_2d]


def pixel_list_to_waypoints(pixel_list: List[List[int]]) -> List[Waypoint2D]:
    """Convert list format back to Waypoint2D objects."""
    return [Waypoint2D(u=p[0], v=p[1]) for p in pixel_list]


# =============================================================================
# TEST / EXAMPLE
# =============================================================================

if __name__ == '__main__':
    # Test the math without CARLA
    print("Testing Trajectory Projection Math")
    print("=" * 50)
    
    # Create camera config (uses defaults: 640x480)
    cam_config = CameraConfig(
        fov=90,
        x=2.0, y=0.0, z=1.8,
        pitch=-15.0, yaw=0.0, roll=0.0
    )
    
    print(f"\nCamera Config:")
    print(f"  Resolution: {cam_config.width}x{cam_config.height}")
    print(f"  FOV: {cam_config.fov}°")
    print(f"  Focal length: fx={cam_config.fx:.1f}, fy={cam_config.fy:.1f}")
    print(f"  Principal point: cx={cam_config.cx:.1f}, cy={cam_config.cy:.1f}")
    
    # Simulated vehicle at origin, facing forward (positive X)
    class MockTransform:
        class Location:
            x, y, z = 0.0, 0.0, 0.5
        class Rotation:
            pitch, yaw, roll = 0.0, 0.0, 0.0
        location = Location()
        rotation = Rotation()
    
    vehicle_transform = MockTransform()
    
    # Create encoder and decoder
    encoder = TrajectoryEncoder(cam_config)
    decoder = TrajectoryDecoder(cam_config)
    
    # Test waypoints (straight ahead at road level z=0)
    # Using farther distances since close ones may be off-screen
    test_waypoints_3d = [
        Waypoint3D(x=10.0, y=0.0, z=0.0),  # 10m ahead
        Waypoint3D(x=15.0, y=0.0, z=0.0),  # 15m ahead
        Waypoint3D(x=20.0, y=0.0, z=0.0),  # 20m ahead
        Waypoint3D(x=25.0, y=0.0, z=0.0),  # 25m ahead
        Waypoint3D(x=30.0, y=0.0, z=0.0),  # 30m ahead
    ]
    
    print(f"\nOriginal 3D Waypoints:")
    for i, wp in enumerate(test_waypoints_3d):
        print(f"  {i+1}. ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
    
    # Encode to 2D WITH index tracking
    waypoints_2d, kept_indices = encoder.encode_with_indices(test_waypoints_3d, vehicle_transform)
    
    print(f"\nEncoded 2D Waypoints (pixels):")
    print(f"  (Kept {len(waypoints_2d)} of {len(test_waypoints_3d)} waypoints - indices: {kept_indices})")
    for i, wp in enumerate(waypoints_2d):
        orig_idx = kept_indices[i]
        print(f"  {i+1}. ({wp.u:.1f}, {wp.v:.1f}) <- original #{orig_idx+1}")
    
    # Decode back to 3D
    waypoints_3d_decoded = decoder.decode(waypoints_2d, vehicle_transform, road_height=0.0)
    
    print(f"\nDecoded 3D Waypoints:")
    for i, wp in enumerate(waypoints_3d_decoded):
        print(f"  {i+1}. ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
    
    # Check round-trip accuracy - NOW COMPARING CORRECT INDICES!
    print(f"\nRound-trip Accuracy (comparing matching waypoints):")
    for i, (decoded, orig_idx) in enumerate(zip(waypoints_3d_decoded, kept_indices)):
        orig = test_waypoints_3d[orig_idx]
        error = np.sqrt((orig.x - decoded.x)**2 + (orig.y - decoded.y)**2)
        status = "✓" if error < 0.5 else "✗"
        print(f"  {i+1}. Original #{orig_idx+1} ({orig.x:.1f}m) vs Decoded ({decoded.x:.1f}m) -> Error: {error:.3f}m {status}")
    
    # Test with curved trajectory (turning left) - using farther distances
    print(f"\n" + "=" * 50)
    print("Testing Curved Trajectory (Turning Left)")
    
    curved_waypoints_3d = [
        Waypoint3D(x=12.0, y=0.0, z=0.0),
        Waypoint3D(x=16.0, y=-2.0, z=0.0),   # Slight left
        Waypoint3D(x=20.0, y=-5.0, z=0.0),   # More left
        Waypoint3D(x=24.0, y=-9.0, z=0.0),   # Sharp left
    ]
    
    print(f"\nOriginal 3D (curved):")
    for i, wp in enumerate(curved_waypoints_3d):
        print(f"  {i+1}. ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
    
    waypoints_2d_curved, kept_curved = encoder.encode_with_indices(curved_waypoints_3d, vehicle_transform)
    
    print(f"\nEncoded 2D (should shift left = lower u values):")
    for i, wp in enumerate(waypoints_2d_curved):
        print(f"  {i+1}. ({wp.u:.1f}, {wp.v:.1f})")
    
    # Decode and check curved
    decoded_curved = decoder.decode(waypoints_2d_curved, vehicle_transform, road_height=0.0)
    print(f"\nDecoded 3D (curved):")
    for i, wp in enumerate(decoded_curved):
        print(f"  {i+1}. ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")
    
    print(f"\nCurved Round-trip Accuracy:")
    for i, (decoded, orig_idx) in enumerate(zip(decoded_curved, kept_curved)):
        orig = curved_waypoints_3d[orig_idx]
        error = np.sqrt((orig.x - decoded.x)**2 + (orig.y - decoded.y)**2)
        status = "✓" if error < 0.5 else "✗"
        print(f"  {i+1}. Error: {error:.3f}m {status}")