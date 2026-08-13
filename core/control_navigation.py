#!/usr/bin/env python3
"""
Control-Based Navigation Command Generator

Generates navigation commands from steering angles and vehicle state.
Used for both training data collection and VLM inference.

Unlike trajectory-based navigation (navigation_analyzer.py), this analyzes
the CONTROL OUTPUT (steering angle) to determine maneuvers.
"""

import math
from typing import Optional, List
import numpy as np


# =============================================================================
# STEERING-BASED NAVIGATION
# =============================================================================

class SteeringNavigationAnalyzer:
    """
    Generate navigation commands from steering angle and vehicle state.

    This provides more accurate navigation context because it reflects
    the actual control being applied, not just waypoint geometry.
    """

    # Steering angle thresholds (in degrees)
    SLIGHT_TURN_THRESHOLD = 5.0      # |angle| > 5° = slight turn
    TURN_THRESHOLD = 15.0             # |angle| > 15° = full turn
    SHARP_TURN_THRESHOLD = 30.0       # |angle| > 30° = sharp turn

    # Speed thresholds for maneuver classification
    SLOW_SPEED_THRESHOLD = 10.0       # km/h - below this is "slow" turn

    @classmethod
    def from_steer_value(cls, steer: float, max_steer_angle: float = 70.0,
                        speed_kmh: float = None) -> str:
        """
        Generate navigation command from CARLA steering value (-1.0 to 1.0).

        Args:
            steer: CARLA steering value (-1.0 = full left, 1.0 = full right)
            max_steer_angle: Maximum steering angle in degrees (default: 70°)
            speed_kmh: Current vehicle speed (optional, for context)

        Returns:
            Navigation command string
        """
        # Convert to degrees
        angle_deg = steer * max_steer_angle
        return cls.from_steer_angle(angle_deg, speed_kmh)

    @classmethod
    def from_steer_angle(cls, angle_deg: float, speed_kmh: Optional[float] = None) -> str:
        """
        Generate navigation command from steering angle in degrees.

        Args:
            angle_deg: Steering angle in degrees (negative = left, positive = right)
            speed_kmh: Current vehicle speed (optional, for context)

        Returns:
            Navigation command string

        CARLA Convention:
            Negative steering angle = LEFT turn
            Positive steering angle = RIGHT turn
        """
        abs_angle = abs(angle_deg)

        # Determine if this is a slow-speed maneuver
        is_slow = speed_kmh is not None and speed_kmh < cls.SLOW_SPEED_THRESHOLD

        # Lane keeping (minimal steering)
        if abs_angle < cls.SLIGHT_TURN_THRESHOLD:
            return "follow_lane"

        # Determine turn severity
        if abs_angle >= cls.SHARP_TURN_THRESHOLD:
            turn_type = "sharp_turn"
        elif abs_angle >= cls.TURN_THRESHOLD:
            turn_type = "turn"
        else:
            turn_type = "slight_turn"

        # Determine direction
        if angle_deg < 0:
            direction = "left"
        else:
            direction = "right"

        # Build command
        if turn_type == "sharp_turn":
            return f"sharp_turn_{direction}"
        elif turn_type == "turn":
            return f"turn_{direction}"
        else:  # slight_turn
            return f"slight_{direction}"

    @classmethod
    def from_waypoints_and_steering(cls,
                                   waypoints_3d: List[List[float]],
                                   steer_angle_deg: float,
                                   vehicle_yaw_deg: Optional[float] = None,
                                   speed_kmh: Optional[float] = None) -> str:
        """
        Enhanced navigation using both waypoints and steering angle.

        This combines trajectory analysis with actual control output for
        more accurate navigation commands.

        Args:
            waypoints_3d: List of [x, y, z] waypoints ahead
            steer_angle_deg: Current steering angle in degrees
            vehicle_yaw_deg: Current vehicle yaw (optional)
            speed_kmh: Current speed (optional)

        Returns:
            Navigation command string
        """
        # Primary signal: steering angle
        steering_nav = cls.from_steer_angle(steer_angle_deg, speed_kmh)

        # If we have waypoints, we can add context
        if waypoints_3d and len(waypoints_3d) >= 3 and vehicle_yaw_deg is not None:
            # Calculate trajectory heading
            wp_start = np.array(waypoints_3d[0][:2])
            wp_end = np.array(waypoints_3d[-1][:2])

            # Trajectory direction
            traj_vec = wp_end - wp_start
            traj_heading = np.degrees(np.arctan2(traj_vec[1], traj_vec[0]))

            # Heading error (how much we need to turn)
            heading_error = cls._normalize_angle(traj_heading - vehicle_yaw_deg)

            # If heading error is large but steering is small, we might be
            # approaching a turn (preparing to turn)
            if abs(heading_error) > 30 and abs(steer_angle_deg) < cls.SLIGHT_TURN_THRESHOLD:
                if heading_error < 0:
                    return "prepare_turn_left"
                else:
                    return "prepare_turn_right"

        return steering_nav

    @staticmethod
    def _normalize_angle(angle_deg: float) -> float:
        """Normalize angle to [-180, 180]."""
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        return angle_deg


# =============================================================================
# DISCRETE CONTROL NAVIGATION (for compatibility with record_dataset.py)
# =============================================================================

def generate_nav_from_discrete_steer(steer_angle_deg: int, speed_kmh: Optional[float] = None) -> str:
    """
    Generate navigation command from discrete steering angle.

    Compatible with DiscreteControlEncoder.train_nav() from record_dataset.py
    but provides more granular classification.

    Args:
        steer_angle_deg: Discrete steering angle in degrees (int)
        speed_kmh: Current speed (optional)

    Returns:
        Navigation command string
    """
    return SteeringNavigationAnalyzer.from_steer_angle(float(steer_angle_deg), speed_kmh)


# =============================================================================
# UNIFIED NAVIGATION COMMAND MAPPING
# =============================================================================

# Human-readable navigation prompts for VLM
NAV_COMMAND_PROMPTS = {
    # Lane keeping
    "follow_lane": "Follow the lane",

    # Slight turns
    "slight_left": "Slight left adjustment",
    "slight_right": "Slight right adjustment",

    # Regular turns
    "turn_left": "Turn left",
    "turn_right": "Turn right",

    # Sharp turns
    "sharp_turn_left": "Sharp left turn",
    "sharp_turn_right": "Sharp right turn",

    # Preparation
    "prepare_turn_left": "Prepare to turn left",
    "prepare_turn_right": "Prepare to turn right",

    # Route-based (from trajectory analysis)
    "u_turn": "Make a U-turn",
    "merge_left": "Merge left",
    "merge_right": "Merge right",
    "intersection_approach": "Approach intersection",
    "lane_keeping": "Keep lane",
}


def get_nav_prompt(nav_command: str) -> str:
    """
    Get human-readable navigation prompt for VLM.

    Args:
        nav_command: Navigation command key

    Returns:
        Human-readable prompt text
    """
    return NAV_COMMAND_PROMPTS.get(nav_command, "Follow the lane")


# =============================================================================
# EXAMPLE / TEST
# =============================================================================

if __name__ == '__main__':
    print("Testing Steering-Based Navigation Analyzer")
    print("=" * 70)

    analyzer = SteeringNavigationAnalyzer()

    # Test cases: (steer_angle, speed, expected_nav)
    test_cases = [
        (0.0, 30.0, "Lane keeping"),
        (3.0, 30.0, "Small adjustment right"),
        (-3.0, 30.0, "Small adjustment left"),
        (10.0, 30.0, "Slight turn right"),
        (-10.0, 30.0, "Slight turn left"),
        (20.0, 30.0, "Turn right"),
        (-20.0, 30.0, "Turn left"),
        (35.0, 30.0, "Sharp turn right"),
        (-35.0, 30.0, "Sharp turn left"),
        (20.0, 5.0, "Slow turn right"),
        (-20.0, 5.0, "Slow turn left"),
    ]

    print("\nSteering Angle → Navigation Command:")
    print(f"{'Angle':<10} | {'Speed':<10} | {'Command':<30}")
    print("-" * 60)

    for angle, speed, description in test_cases:
        nav = analyzer.from_steer_angle(angle, speed)
        prompt = get_nav_prompt(nav)
        print(f"{angle:>6.1f}° | {speed:>6.1f} km/h | {nav:<20} → {prompt}")

    print("\n" + "=" * 70)
    print("Navigation Command Mapping:")
    print("=" * 70)
    for cmd, prompt in sorted(NAV_COMMAND_PROMPTS.items()):
        print(f"  {cmd:<25} → {prompt}")
