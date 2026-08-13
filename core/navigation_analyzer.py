#!/usr/bin/env python3
"""
Navigation Analyzer Module

Generates navigation tokens from 3D waypoint trajectories.

Supported tokens:
- lane_keeping
- turn_left
- turn_right
- u_turn
- merge_left
- merge_right
- intersection_approach
"""

import numpy as np
from typing import List, Tuple, Optional


def calculate_trajectory_direction(waypoints_3d: List[List[float]]) -> str:
    """
    Analyze trajectory direction from 3D waypoints.
    
    Args:
        waypoints_3d: List of [x, y, z] waypoints
        
    Returns:
        Navigation token string
    """
    if not waypoints_3d or len(waypoints_3d) < 2:
        return 'lane_keeping'
    
    wp0 = np.array(waypoints_3d[0][:2])  # [x, y]
    wp_mid = np.array(waypoints_3d[len(waypoints_3d)//2][:2])
    wp_last = np.array(waypoints_3d[-1][:2])
    
    vec1 = wp_mid - wp0
    vec2 = wp_last - wp_mid
    
    angle1 = np.arctan2(vec1[1], vec1[0])
    angle2 = np.arctan2(vec2[1], vec2[0])
    
    angle_change = np.degrees(angle2 - angle1)
    
    angle_change = (angle_change + 180) % 360 - 180
    
    if angle_change > 150 or angle_change < -150:
        return 'u_turn'
    elif angle_change > 30:
        return 'turn_left'
    elif angle_change < -30:
        return 'turn_right'
    else:
        return 'lane_keeping'


def estimate_curvature(waypoints_3d: List[List[float]], window_size: int = 3) -> float:
    """
    Estimate trajectory curvature using sliding window.
    
    Args:
        waypoints_3d: List of [x, y, z] waypoints
        window_size: Size of window for curvature calculation
        
    Returns:
        Curvature value (higher = more curved)
    """
    if len(waypoints_3d) < window_size + 1:
        return 0.0
    
    curvatures = []
    for i in range(len(waypoints_3d) - window_size):
        p1 = np.array(waypoints_3d[i][:2])
        p2 = np.array(waypoints_3d[i + window_size // 2][:2])
        p3 = np.array(waypoints_3d[i + window_size][:2])
        
        vec1 = p2 - p1
        vec2 = p3 - p2
        
        angle1 = np.arctan2(vec1[1], vec1[0])
        angle2 = np.arctan2(vec2[1], vec2[0])
        
        angle_change = abs(np.degrees(angle2 - angle1))
        curvatures.append(angle_change)
    
    return np.mean(curvatures) if curvatures else 0.0


def detect_intersection(waypoints_3d: List[List[float]], road_width: float = 3.5) -> Optional[str]:
    """
    Detect if approaching intersection based on trajectory shape.
    
    This is a simplified heuristic-based detection.
    In a real implementation, this would use CARLA map API.
    
    Args:
        waypoints_3d: List of [x, y, z] waypoints
        road_width: Estimated road width in meters
        
    Returns:
        'intersection_approach' if detected, None otherwise
    """
    if len(waypoints_3d) < 5:
        return None
    
    # Check for lateral spread that suggests intersection
    y_coords = [wp[1] for wp in waypoints_3d]
    y_spread = max(y_coords) - min(y_coords)
    
    # If Y spread exceeds typical road width, might be intersection
    if y_spread > road_width * 2:
        return 'intersection_approach'
    
    return None


def generate_navigation_token(
    waypoints_3d: List[List[float]],
    use_map_api: bool = False,
    carla_map: Optional[object] = None
) -> str:
    """
    Generate comprehensive navigation token from 3D waypoints.
    
    Args:
        waypoints_3d: List of [x, y, z] waypoints
        use_map_api: If True, use CARLA map API for more accurate detection
        carla_map: CARLA map object (required if use_map_api=True)
        
    Returns:
        Navigation token string
    """
    if not waypoints_3d:
        return 'lane_keeping'
    
    primary_direction = calculate_trajectory_direction(waypoints_3d)
    
    if use_map_api and carla_map is not None:
        try:
            wp0 = carla_map.get_waypoint(
                carla.Location(x=waypoints_3d[0][0], y=waypoints_3d[0][1], z=waypoints_3d[0][2])
            )
            
            wp_mid = carla_map.get_waypoint(
                carla.Location(x=waypoints_3d[len(waypoints_3d)//2][0], 
                             y=waypoints_3d[len(waypoints_3d)//2][1], 
                             z=waypoints_3d[len(waypoints_3d)//2][2])
            )
            
            wp_next = wp_mid.next(2.0)
            if wp_next:
                options = wp_next.next(5.0)
                if len(options) > 1:
                    return 'intersection_approach'
        except Exception:
            pass
    
    if primary_direction != 'lane_keeping':
        return primary_direction

    return primary_direction


# =============================================================================
# TEST / EXAMPLE
# =============================================================================

if __name__ == '__main__':
    print("Testing Navigation Analyzer")
    print("=" * 50)
    
    test_trajectories = {
        'lane_keeping': [
            [100.0, 0.0, 0.0],
            [105.0, 0.0, 0.0],
            [110.0, 0.0, 0.0],
            [115.0, 0.0, 0.0],
            [120.0, 0.0, 0.0],
        ],
        'turn_left': [
            [100.0, 0.0, 0.0],
            [103.0, -2.0, 0.0],
            [106.0, -5.0, 0.0],
            [109.0, -8.0, 0.0],
            [112.0, -12.0, 0.0],
        ],
        'turn_right': [
            [100.0, 0.0, 0.0],
            [103.0, 2.0, 0.0],
            [106.0, 5.0, 0.0],
            [109.0, 8.0, 0.0],
            [112.0, 12.0, 0.0],
        ],
        'u_turn': [
            [100.0, 0.0, 0.0],
            [95.0, -3.0, 0.0],
            [90.0, -5.0, 0.0],
            [95.0, 2.0, 0.0],
            [100.0, 10.0, 0.0],
            [105.0, 3.0, 0.0],
            [110.0, 0.0, 0.0],
            [100.0, -3.0, 0.0],
        ],
        'intersection_approach': [
            [100.0, -3.0, 0.0],
            [103.0, 0.0, 0.0],
            [106.0, 3.0, 0.0],
            [109.0, 6.0, 0.0],
            [112.0, 0.0, 0.0],
            [112.0, 3.0, 0.0],
        ],
    }
    
    for name, trajectory in test_trajectories.items():
        print(f"\n{name.upper()}:")
        nav_token = generate_navigation_token(trajectory)
        print(f"  Token: {nav_token}")
        
        direction = calculate_trajectory_direction(trajectory)
        print(f"  Direction: {direction}")
        curvature = estimate_curvature(trajectory)
        print(f"  Curvature: {curvature:.2f}")
