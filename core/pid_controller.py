#!/usr/bin/env python3
"""
PID Controller Module for Autonomous Vehicle Control

Provides unified PID-based vehicle control for waypoint following.
Combines lateral (steering) and longitudinal (speed) control.

This module consolidates the VehiclePIDController that was previously
duplicated across vlm_pure_drive.py, vlm_auto_drive.py, and record_dataset.py.

Usage:
    from pid_controller import VehiclePIDController

    controller = VehiclePIDController(target_speed_kmh=30.0)
    control = controller.compute_control(waypoints_3d, vehicle_transform, current_speed_kmh)
    vehicle.apply_control(control)
"""

import math
from typing import List, Optional, Protocol

import numpy as np


class Transform(Protocol):
    """Protocol for transform objects (CARLA or mock)."""

    class location:
        x: float
        y: float
        z: float

    class rotation:
        pitch: float
        yaw: float
        roll: float


class VehicleControl(Protocol):
    """Protocol for vehicle control objects."""

    throttle: float
    brake: float
    steer: float
    hand_brake: bool
    manual_gear_shift: bool


class Waypoint3D(Protocol):
    """Protocol for 3D waypoint objects."""

    x: float
    y: float
    z: float


class VehiclePIDController:
    """
    PID controller for waypoint following.

    Combines lateral (steering) and longitudinal (speed) control.
    Uses pure pursuit algorithm for steering with PID for speed control.
    """

    def __init__(
        self,
        target_speed_kmh: float = 30.0,
        lateral_kp: float = 1.0,
        lateral_ki: float = 0.0,
        lateral_kd: float = 0.1,
        longitudinal_kp: float = 0.5,
        longitudinal_ki: float = 0.05,
        longitudinal_kd: float = 0.1,
        lookahead_distance: float = 8.0,
        max_throttle: float = 0.7,
        max_brake: float = 0.5,
        max_steering_rate: float = 0.30,
    ):
        """
        Initialize PID controller.

        Args:
            target_speed_kmh: Target driving speed in km/h
            lateral_kp: Lateral proportional gain
            lateral_ki: Lateral integral gain
            lateral_kd: Lateral derivative gain
            longitudinal_kp: Longitudinal proportional gain
            longitudinal_ki: Longitudinal integral gain
            longitudinal_kd: Longitudinal derivative gain
            lookahead_distance: Distance to lookahead waypoint in meters
            max_throttle: Maximum throttle value (0-1)
            max_brake: Maximum brake value (0-1)
            max_steering_rate: Maximum steering change per step
        """
        self.target_speed_kmh = target_speed_kmh
        self.lookahead_distance = lookahead_distance
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.max_steering_rate = max_steering_rate

        # Lateral PID (steering)
        self.lat_kp = lateral_kp
        self.lat_ki = lateral_ki
        self.lat_kd = lateral_kd
        self.lat_error_integral = 0.0
        self.lat_prev_error = 0.0

        # Longitudinal PID (speed)
        self.lon_kp = longitudinal_kp
        self.lon_ki = longitudinal_ki
        self.lon_kd = longitudinal_kd
        self.lon_error_integral = 0.0
        self.lon_prev_error = 0.0

        # Steering smoothing
        self.prev_steer = 0.0

    def compute_control(
        self,
        waypoints_3d: List[Waypoint3D],
        vehicle_transform,
        current_speed_kmh: float,
    ):
        """
        Compute vehicle control from waypoints.

        Args:
            waypoints_3d: List of 3D waypoints to follow
            vehicle_transform: Current vehicle transform (CARLA or mock)
            current_speed_kmh: Current speed in km/h

        Returns:
            VehicleControl command (CARLA compatible)
        """
        # Import carla here to allow module to work without CARLA installed
        try:
            import carla

            control = carla.VehicleControl()
        except ImportError:
            # Create a simple control object for testing
            control = _MockVehicleControl()

        if not waypoints_3d:
            # No waypoints - stop the vehicle
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            return control

        # === LATERAL CONTROL (STEERING) ===
        steer = self._compute_steering(waypoints_3d, vehicle_transform,
                                       current_speed_kmh)
        control.steer = float(steer)

        # === LONGITUDINAL CONTROL (SPEED) ===
        throttle, brake = self._compute_speed_control(current_speed_kmh)
        control.throttle = throttle
        control.brake = brake

        control.hand_brake = False
        control.manual_gear_shift = False

        return control

    def _find_lookahead_point(
        self,
        waypoints_3d: List[Waypoint3D],
        vehicle_x: float,
        vehicle_y: float,
        lookahead: float,
    ) -> tuple:
        """
        Find the lookahead point by interpolating along the waypoint path.

        Returns (target_x, target_y, actual_lookahead_dist).
        """
        # Build cumulative path distances from vehicle
        # First: distance from vehicle to each waypoint along the path
        # path_d[0] = dist(vehicle, wp[0])
        # path_d[i] = path_d[0] + sum of segment lengths from wp[0]..wp[i]
        n = len(waypoints_3d)
        path_d = [0.0] * n
        path_d[0] = math.sqrt(
            (waypoints_3d[0].x - vehicle_x) ** 2
            + (waypoints_3d[0].y - vehicle_y) ** 2
        )
        for i in range(1, n):
            seg = math.sqrt(
                (waypoints_3d[i].x - waypoints_3d[i - 1].x) ** 2
                + (waypoints_3d[i].y - waypoints_3d[i - 1].y) ** 2
            )
            path_d[i] = path_d[i - 1] + seg

        # Interpolate along the path to find exact lookahead point
        for i in range(n - 1):
            if path_d[i] <= lookahead <= path_d[i + 1]:
                seg_len = path_d[i + 1] - path_d[i]
                t = (lookahead - path_d[i]) / seg_len if seg_len > 1e-6 else 0.0
                tx = waypoints_3d[i].x + t * (waypoints_3d[i + 1].x - waypoints_3d[i].x)
                ty = waypoints_3d[i].y + t * (waypoints_3d[i + 1].y - waypoints_3d[i].y)
                return tx, ty, lookahead

        # Lookahead exceeds path length — extrapolate from last two waypoints
        if n >= 2:
            dx = waypoints_3d[-1].x - waypoints_3d[-2].x
            dy = waypoints_3d[-1].y - waypoints_3d[-2].y
            seg = math.sqrt(dx * dx + dy * dy)
            if seg > 1e-6:
                overshoot = lookahead - path_d[-1]
                tx = waypoints_3d[-1].x + (dx / seg) * overshoot
                ty = waypoints_3d[-1].y + (dy / seg) * overshoot
                return tx, ty, lookahead

        # Fallback: use the furthest waypoint
        return waypoints_3d[-1].x, waypoints_3d[-1].y, path_d[-1]

    def _compute_steering(
        self,
        waypoints_3d: List[Waypoint3D],
        vehicle_transform,
        current_speed_kmh: float = 0.0,
    ) -> float:
        """Compute steering using pure pursuit with path interpolation."""
        vehicle_x = vehicle_transform.location.x
        vehicle_y = vehicle_transform.location.y

        # Speed-adaptive lookahead: longer at higher speeds, clamped to [5m, max]
        speed_ms = max(current_speed_kmh, 5.0) / 3.6
        lookahead = np.clip(speed_ms * 1.0, 5.0, self.lookahead_distance)

        # Find interpolated lookahead point on the path
        target_x, target_y, actual_ld = self._find_lookahead_point(
            waypoints_3d, vehicle_x, vehicle_y, lookahead
        )

        # Compute heading error
        dx = target_x - vehicle_x
        dy = target_y - vehicle_y

        target_angle = math.atan2(dy, dx)
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)

        angle_diff = target_angle - vehicle_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # PID for steering (with integral windup clamp)
        self.lat_error_integral = np.clip(
            self.lat_error_integral + angle_diff, -1.0, 1.0
        )
        lat_error_derivative = angle_diff - self.lat_prev_error

        steer_output = (
            self.lat_kp * angle_diff
            + self.lat_ki * self.lat_error_integral
            + self.lat_kd * lat_error_derivative
        )

        # Normalize to [-1, 1]
        steer = np.clip(steer_output / math.radians(70), -1.0, 1.0)

        # Speed-adaptive rate limit: responsive at low speed, smoother at high
        # Low speed (<15 km/h) -> 0.25/frame, high speed (>40 km/h) -> 0.08/frame
        rate = np.clip(
            0.30 - (current_speed_kmh / 60.0) * 0.22,
            0.08,
            self.max_steering_rate,
        )
        if steer > self.prev_steer + rate:
            steer = self.prev_steer + rate
        elif steer < self.prev_steer - rate:
            steer = self.prev_steer - rate

        self.prev_steer = steer
        self.lat_prev_error = angle_diff

        return steer

    def _compute_speed_control(self, current_speed_kmh: float) -> tuple:
        """Compute throttle and brake using PID speed control."""
        speed_error = self.target_speed_kmh - current_speed_kmh
        self.lon_error_integral += speed_error
        speed_derivative = speed_error - self.lon_prev_error

        speed_control = (
            self.lon_kp * speed_error
            + self.lon_ki * self.lon_error_integral
            + self.lon_kd * speed_derivative
        )

        if speed_control > 0:
            throttle = float(np.clip(speed_control, 0.0, self.max_throttle))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(np.clip(-speed_control, 0.0, self.max_brake))

        self.lon_prev_error = speed_error

        return throttle, brake

    def reset(self) -> None:
        """Reset controller state."""
        self.lat_error_integral = 0.0
        self.lat_prev_error = 0.0
        self.lon_error_integral = 0.0
        self.lon_prev_error = 0.0
        self.prev_steer = 0.0

    def set_target_speed(self, speed_kmh: float) -> None:
        """Update target speed."""
        self.target_speed_kmh = speed_kmh


class _MockVehicleControl:
    """Mock vehicle control for testing without CARLA."""

    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.hand_brake = False
        self.manual_gear_shift = False


# =============================================================================
# TEST
# =============================================================================

if __name__ == "__main__":
    print("Testing PID Controller Module")
    print("=" * 50)

    # Create controller
    controller = VehiclePIDController(target_speed_kmh=30.0)

    # Mock waypoints
    class MockWaypoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    # Mock transform
    class MockTransform:
        class location:
            x = 0.0
            y = 0.0
            z = 0.0

        class rotation:
            pitch = 0.0
            yaw = 0.0
            roll = 0.0

    waypoints = [
        MockWaypoint(10.0, 0.0, 0.0),
        MockWaypoint(20.0, 0.0, 0.0),
        MockWaypoint(30.0, 0.0, 0.0),
    ]

    transform = MockTransform()
    current_speed = 20.0

    control = controller.compute_control(waypoints, transform, current_speed)

    print(f"Target speed: {controller.target_speed_kmh} km/h")
    print(f"Current speed: {current_speed} km/h")
    print(f"Control output:")
    print(f"  Throttle: {control.throttle:.3f}")
    print(f"  Brake: {control.brake:.3f}")
    print(f"  Steer: {control.steer:.3f}")

    # Test empty waypoints (should stop)
    print("\nTesting with no waypoints (should stop):")
    control = controller.compute_control([], transform, current_speed)
    print(f"  Throttle: {control.throttle:.3f}")
    print(f"  Brake: {control.brake:.3f}")
    print(f"  Steer: {control.steer:.3f}")

    print("\nPID Controller Module test complete!")
