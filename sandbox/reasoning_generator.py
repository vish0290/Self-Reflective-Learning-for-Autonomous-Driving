#!/usr/bin/env python3
"""
Reasoning Generator Module

Generates natural language reasoning for trajectory decisions based on navigation token,
vehicle state, and waypoints.
"""

import numpy as np
from typing import List, Dict


REASONING_TEMPLATES = {
    'lane_keeping': [
        "Maintaining lane on a straight road. Current speed of {speed:.1f} km/h is appropriate for the current road conditions. Continuing straight to follow lane center.",
        "On a straight road segment. Vehicle speed is {speed:.1f} km/h. Keeping vehicle centered in the lane with smooth steering.",
        "Driving on a straight path. Speed of {speed:.1f} km/h is within safe limits. Maintaining lane position with minimal steering input.",
    ],
    
    'turn_left': [
        "Approaching a left turn. The road curves to the left starting {dist}m ahead. Current speed of {speed:.1f} km/h is appropriate for this curve. I'll steer left smoothly to follow the lane center.",
        "Left turn detected ahead. Vehicle moving at {speed:.1f} km/h. Will initiate gradual left steering to navigate the curve safely.",
        "Curve to left approaching. Speed {speed:.1f} km/h. Applying left steering input to maintain proper lane position through the turn.",
    ],
    
    'turn_right': [
        "Approaching a right turn. The road curves to the right starting {dist}m ahead. Current speed of {speed:.1f} km/h is appropriate for this curve. I'll steer right smoothly to follow the lane center.",
        "Right turn detected ahead. Vehicle moving at {speed:.1f} km/h. Will initiate gradual right steering to navigate the curve safely.",
        "Curve to right approaching. Speed {speed:.1f} km/h. Applying right steering input to maintain proper lane position through the turn.",
    ],
    
    'u_turn': [
        "Executing U-turn maneuver. Current speed is {speed:.1f} km/h. Will turn approximately 180 degrees to reverse direction.",
        "U-turn required. Vehicle speed {speed:.1f} km/h. Performing sharp turn to face opposite direction safely.",
        "Making U-turn. Speed {speed:.1f} km/h. Will execute left steering followed by right steering to complete 180-degree turn.",
    ],
    
    'merge_left': [
        "Approaching merge to left lane. Road curvature suggests merging opportunity {dist}m ahead. Speed {speed:.1f} km/h. Will accelerate slightly and steer left to merge smoothly.",
        "Left merge detected. Vehicle at {speed:.1f} km/h. Checking blind spot and accelerating to merge into adjacent lane.",
    ],
    
    'merge_right': [
        "Approaching merge to right lane. Road curvature suggests merging opportunity {dist}m ahead. Speed {speed:.1f} km/h. Will accelerate slightly and steer right to merge smoothly.",
        "Right merge detected. Vehicle at {speed:.1f} km/h. Checking blind spot and accelerating to merge into adjacent lane.",
    ],
    
    'intersection_approach': [
        "Approaching an intersection. Checking for clear path ahead. Current speed {speed:.1f} km/h is safe. Will proceed straight through intersection.",
        "Intersection ahead. Vehicle speed {speed:.1f} km/h. Monitoring for traffic signals and pedestrian activity. Continuing straight if path is clear.",
    ],
}


def get_waypoint_distances(waypoints_3d: List[List[float]]) -> List[float]:
    """
    Calculate cumulative distances from first waypoint.
    
    Args:
        waypoints_3d: List of [x, y, z] waypoints
        
    Returns:
        List of distances in meters
    """
    if not waypoints_3d:
        return []
    
    distances = [0.0]
    wp0 = np.array(waypoints_3d[0][:2])
    
    for i in range(1, len(waypoints_3d)):
        wp = np.array(waypoints_3d[i][:2])
        dist = np.linalg.norm(wp - wp0)
        distances.append(dist)
    
    return distances


def get_curvature_description(waypoints_3d: List[List[float]]) -> str:
    """
    Get text description of trajectory curvature.
    
    Args:
        waypoints_3d: List of [x, y, z] waypoints
        
    Returns:
        Curvature description string
    """
    if len(waypoints_3d) < 3:
        return "gentle"
    
    wp0 = np.array(waypoints_3d[0][:2])
    wp_mid = np.array(waypoints_3d[len(waypoints_3d)//2][:2])
    wp_last = np.array(waypoints_3d[-1][:2])
    
    vec1 = wp_mid - wp0
    vec2 = wp_last - wp_mid
    
    angle1 = np.arctan2(vec1[1], vec1[0])
    angle2 = np.arctan2(vec2[1], vec2[0])
    
    angle_change = abs(np.degrees(angle2 - angle1))
    
    if angle_change < 10:
        return "gentle curve"
    elif angle_change < 25:
        return "moderate curve"
    elif angle_change < 45:
        return "sharp curve"
    else:
        return "very sharp curve"


def generate_reasoning(
    navigation_token: str,
    vehicle_state: Dict,
    waypoints_3d: List[List[float]],
    use_templates: bool = True
) -> str:
    """
    Generate natural language reasoning for trajectory decision.
    
    Args:
        navigation_token: Navigation classification (lane_keeping, turn_left, etc.)
        vehicle_state: Dictionary containing ego vehicle state
        waypoints_3d: List of [x, y, z] waypoints
        use_templates: If True, use predefined templates. If False, generate dynamically.
        
    Returns:
        Reasoning text string
    """
    speed_kmh = vehicle_state.get('speed_kmh', 0.0)
    heading = vehicle_state.get('rotation', [0, 0, 0, 0])[1] if vehicle_state.get('rotation') else 0.0
    
    distances = get_waypoint_distances(waypoints_3d)
    first_dist = distances[0] if distances else 0
    
    curvature_desc = get_curvature_description(waypoints_3d)
    
    if use_templates and navigation_token in REASONING_TEMPLATES:
        import random
        template = random.choice(REASONING_TEMPLATES[navigation_token])
        reasoning = template.format(
            speed=speed_kmh,
            dist=first_dist
        )
    else:
        if navigation_token == 'lane_keeping':
            reasoning = f"Maintaining lane on a straight road. Current speed of {speed_kmh:.1f} km/h is appropriate for the current road conditions. {curvature_desc} detected. Continuing straight to follow lane center."
        elif navigation_token == 'turn_left':
            reasoning = f"Approaching a left turn. The road curves to the left starting {first_dist:.0f}m ahead. Current speed of {speed_kmh:.1f} km/h is appropriate for this curve. I'll steer left smoothly to follow the lane center. Vehicle heading {heading:.0f} degrees."
        elif navigation_token == 'turn_right':
            reasoning = f"Approaching a right turn. The road curves to the right starting {first_dist:.0f}m ahead. Current speed of {speed_kmh:.1f} km/h is appropriate for this curve. I'll steer right smoothly to follow the lane center. Vehicle heading {heading:.0f} degrees."
        elif navigation_token == 'u_turn':
            reasoning = f"Executing U-turn maneuver. Current speed is {speed_kmh:.1f} km/h. Will turn approximately 180 degrees to reverse direction. {curvature_desc}."
        elif navigation_token == 'merge_left':
            reasoning = f"Approaching merge to left lane. Road curvature suggests merging opportunity {first_dist:.0f}m ahead. Speed {speed_kmh:.1f} km/h. Will accelerate slightly and steer left to merge smoothly. Vehicle heading {heading:.0f} degrees."
        elif navigation_token == 'merge_right':
            reasoning = f"Approaching merge to right lane. Road curvature suggests merging opportunity {first_dist:.0f}m ahead. Speed {speed_kmh:.1f} km/h. Will accelerate slightly and steer right to merge smoothly. Vehicle heading {heading:.0f} degrees."
        elif navigation_token == 'intersection_approach':
            reasoning = f"Approaching an intersection. Checking for clear path ahead. Current speed {speed_kmh:.1f} km/h is safe. Will proceed straight through intersection. Vehicle heading {heading:.0f} degrees."
        else:
            reasoning = f"Following trajectory in {navigation_token} mode. Speed {speed_kmh:.1f} km/h, heading {heading:.0f} degrees. {curvature_desc} detected."
    
    return reasoning


# =============================================================================
# TEST / EXAMPLE
# =============================================================================

if __name__ == '__main__':
    print("Testing Reasoning Generator")
    print("=" * 50)
    
    test_vehicle_state = {
        'speed_kmh': 35.2,
        'rotation': [0.0, 90.0, 0.0],
        'location': [100.0, 0.0, 0.5],
    }
    
    test_waypoints = {
        'lane_keeping': [[100, 0, 0], [105, 0, 0], [110, 0, 0]],
        'turn_left': [[100, 0, 0], [103, -2, 0], [106, -5, 0], [109, -8, 0]],
        'turn_right': [[100, 0, 0], [103, 2, 0], [106, 5, 0], [109, 8, 0]],
        'u_turn': [[100, 0, 0], [95, -3, 0], [90, -5, 0], [95, 2, 0], [100, 10, 0]],
    }
    
    for nav_token, waypoints in test_waypoints.items():
        print(f"\n{nav_token.upper()}:")
        reasoning = generate_reasoning(nav_token, test_vehicle_state, waypoints)
        print(f"  {reasoning}")
    
    print("\n\nTesting with dynamic generation (use_templates=False):")
    reasoning = generate_reasoning('turn_left', test_vehicle_state, test_waypoints['turn_left'], use_templates=False)
    print(f"  {reasoning}")
