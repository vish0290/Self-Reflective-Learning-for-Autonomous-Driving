#!/usr/bin/env python3
"""
Trajectory Pipeline Module

This module demonstrates the complete trajectory processing pipeline:

    3D Waypoints ──► 3D-to-2D Encoder ──► 2D Pixels ──► 2D-to-3D Decoder ──► Decoded 3D ──► PID Controller

Components:
- TrajectoryPipeline: Orchestrates the full 3D→2D→3D flow
- VLMSimulator: Simulates VLM inference (returns 2D trajectory)
- PipelineValidator: Validates encode/decode accuracy

Usage:
    from trajectory_pipeline import TrajectoryPipeline
    
    pipeline = TrajectoryPipeline(camera_config)
    
    # Data collection mode (get 2D for training)
    waypoints_2d = pipeline.encode(waypoints_3d, vehicle_transform)
    
    # Inference mode (decode 2D to 3D for control)
    waypoints_3d_decoded = pipeline.decode(waypoints_2d, vehicle_transform)
    
    # Full pipeline (3D → 2D → 3D)
    decoded_3d = pipeline.process(waypoints_3d, vehicle_transform)
"""

import numpy as np
import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

from traj_planner import (
    CameraConfig, WaypointGenerator, TrajectoryEncoder, TrajectoryDecoder,
    Waypoint2D, Waypoint3D, waypoints_to_pixel_list, pixel_list_to_waypoints
)
from pid import PurePursuitController, StanleyController, ControllerConfig, VehicleControl


# =============================================================================
# TRAJECTORY PIPELINE
# =============================================================================

class TrajectoryPipeline:
    """
    Complete trajectory processing pipeline.
    
    Handles the full flow:
    1. 3D waypoints from planner
    2. Encode 3D → 2D (for VLM training/inference)
    3. Decode 2D → 3D (for vehicle control)
    4. Control computation
    """
    
    def __init__(self, camera_config: CameraConfig, controller_config: ControllerConfig = None):
        """
        Initialize pipeline components.
        
        Args:
            camera_config: Camera parameters for projection
            controller_config: PID controller parameters
        """
        self.camera_config = camera_config
        self.encoder = TrajectoryEncoder(camera_config)
        self.decoder = TrajectoryDecoder(camera_config)
        self.controller = PurePursuitController(controller_config or ControllerConfig())
        
        # Statistics
        self.stats = {
            'encode_count': 0,
            'decode_count': 0,
            'total_error': 0.0,
            'max_error': 0.0
        }
    
    def encode(self, waypoints_3d: List[Waypoint3D], 
               vehicle_transform) -> List[Waypoint2D]:
        """
        Encode 3D waypoints to 2D pixel coordinates.
        
        Args:
            waypoints_3d: 3D waypoints in world coordinates
            vehicle_transform: Current vehicle transform
        
        Returns:
            2D waypoints in pixel coordinates
        """
        waypoints_2d = self.encoder.encode(waypoints_3d, vehicle_transform)
        self.stats['encode_count'] += 1
        return waypoints_2d
    
    def decode(self, waypoints_2d: List[Waypoint2D], 
               vehicle_transform,
               road_height: float = None) -> List[Waypoint3D]:
        """
        Decode 2D pixel coordinates back to 3D world coordinates.
        
        Args:
            waypoints_2d: 2D waypoints in pixel coordinates
            vehicle_transform: Current vehicle transform
            road_height: Z-coordinate of road plane
        
        Returns:
            3D waypoints in world coordinates
        """
        if road_height is None:
            road_height = vehicle_transform.location.z - 0.5
        
        waypoints_3d = self.decoder.decode(waypoints_2d, vehicle_transform, road_height)
        self.stats['decode_count'] += 1
        return waypoints_3d
    
    def process(self, waypoints_3d: List[Waypoint3D],
                vehicle_transform,
                road_height: float = None) -> Tuple[List[Waypoint2D], List[Waypoint3D]]:
        """
        Full pipeline: 3D → 2D → 3D
        
        Args:
            waypoints_3d: Original 3D waypoints
            vehicle_transform: Current vehicle transform
            road_height: Z-coordinate of road plane
        
        Returns:
            Tuple of (2D waypoints, decoded 3D waypoints)
        """
        # Encode 3D to 2D
        waypoints_2d = self.encode(waypoints_3d, vehicle_transform)
        
        # Decode 2D back to 3D
        waypoints_3d_decoded = self.decode(waypoints_2d, vehicle_transform, road_height)
        
        # Calculate reconstruction error
        if waypoints_3d and waypoints_3d_decoded:
            error = self._calculate_error(waypoints_3d, waypoints_3d_decoded)
            self.stats['total_error'] += error
            self.stats['max_error'] = max(self.stats['max_error'], error)
        
        return waypoints_2d, waypoints_3d_decoded
    
    def compute_control(self, waypoints_3d: List[Waypoint3D],
                        vehicle_transform,
                        current_speed_kmh: float,
                        target_speed_kmh: float = None) -> VehicleControl:
        """
        Compute vehicle control from 3D waypoints.
        
        This can be used with either original or decoded waypoints.
        
        Args:
            waypoints_3d: 3D waypoints to follow
            vehicle_transform: Current vehicle transform
            current_speed_kmh: Current vehicle speed
            target_speed_kmh: Desired speed
        
        Returns:
            VehicleControl with throttle, brake, steer
        """
        return self.controller.compute_control(
            waypoints_3d, vehicle_transform, current_speed_kmh, target_speed_kmh
        )
    
    def process_and_control(self, waypoints_3d_original: List[Waypoint3D],
                           vehicle_transform,
                           current_speed_kmh: float,
                           target_speed_kmh: float = None) -> Tuple[VehicleControl, Dict]:
        """
        Complete pipeline: Generate 3D → Encode 2D → Decode 3D → Control
        
        This is the main method for autonomous driving using the pipeline.
        
        Returns:
            Tuple of (VehicleControl, debug_info dict)
        """
        # Run full pipeline
        waypoints_2d, waypoints_3d_decoded = self.process(
            waypoints_3d_original, vehicle_transform
        )
        
        # Use decoded trajectory for control
        waypoints_for_control = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d_original
        
        # Compute control
        control = self.compute_control(
            waypoints_for_control, vehicle_transform, current_speed_kmh, target_speed_kmh
        )
        
        # Debug info
        debug_info = {
            'original_3d_count': len(waypoints_3d_original),
            'encoded_2d_count': len(waypoints_2d),
            'decoded_3d_count': len(waypoints_3d_decoded),
            'waypoints_2d': waypoints_2d,
            'waypoints_3d_decoded': waypoints_3d_decoded,
            'reconstruction_error': self._calculate_error(waypoints_3d_original, waypoints_3d_decoded)
        }
        
        return control, debug_info
    
    def _calculate_error(self, original: List[Waypoint3D], 
                        decoded: List[Waypoint3D]) -> float:
        """Calculate average reconstruction error."""
        if not original or not decoded:
            return 0.0
        
        min_len = min(len(original), len(decoded))
        total_error = 0.0
        
        for i in range(min_len):
            error = math.sqrt(
                (original[i].x - decoded[i].x)**2 +
                (original[i].y - decoded[i].y)**2
            )
            total_error += error
        
        return total_error / min_len
    
    def get_stats(self) -> Dict:
        """Get pipeline statistics."""
        return {
            **self.stats,
            'avg_error': self.stats['total_error'] / max(self.stats['decode_count'], 1)
        }
    
    def reset_stats(self):
        """Reset statistics."""
        self.stats = {
            'encode_count': 0,
            'decode_count': 0,
            'total_error': 0.0,
            'max_error': 0.0
        }


# =============================================================================
# VLM SIMULATOR (for testing without actual VLM)
# =============================================================================

class VLMSimulator:
    """
    Simulates VLM trajectory prediction.
    
    Used for testing the pipeline without an actual VLM.
    Can add noise to simulate prediction errors.
    """
    
    def __init__(self, noise_level: float = 0.0):
        """
        Args:
            noise_level: Standard deviation of Gaussian noise to add (pixels)
        """
        self.noise_level = noise_level
    
    def predict(self, waypoints_2d: List[Waypoint2D]) -> List[Waypoint2D]:
        """
        Simulate VLM prediction by optionally adding noise.
        
        In real usage, this would be replaced by actual VLM inference.
        
        Args:
            waypoints_2d: Ground truth 2D waypoints
        
        Returns:
            "Predicted" 2D waypoints (with optional noise)
        """
        if self.noise_level == 0.0:
            return waypoints_2d
        
        predicted = []
        for wp in waypoints_2d:
            noise_u = np.random.normal(0, self.noise_level)
            noise_v = np.random.normal(0, self.noise_level)
            predicted.append(Waypoint2D(
                u=wp.u + noise_u,
                v=wp.v + noise_v
            ))
        
        return predicted


# =============================================================================
# PIPELINE VALIDATOR
# =============================================================================

class PipelineValidator:
    """
    Validates the 3D→2D→3D pipeline accuracy.
    
    Useful for:
    - Testing camera calibration
    - Verifying encoder/decoder math
    - Benchmarking reconstruction accuracy
    """
    
    def __init__(self, pipeline: TrajectoryPipeline):
        self.pipeline = pipeline
        self.errors = []
    
    def validate(self, waypoints_3d: List[Waypoint3D],
                 vehicle_transform,
                 road_height: float = None) -> Dict:
        """
        Validate pipeline accuracy for given waypoints.
        
        Returns:
            Dict with validation results
        """
        # Run pipeline
        waypoints_2d, waypoints_3d_decoded = self.pipeline.process(
            waypoints_3d, vehicle_transform, road_height
        )
        
        # Calculate per-waypoint errors
        point_errors = []
        min_len = min(len(waypoints_3d), len(waypoints_3d_decoded))
        
        for i in range(min_len):
            orig = waypoints_3d[i]
            dec = waypoints_3d_decoded[i]
            
            error_xy = math.sqrt((orig.x - dec.x)**2 + (orig.y - dec.y)**2)
            error_z = abs(orig.z - dec.z)
            
            point_errors.append({
                'index': i,
                'original': (orig.x, orig.y, orig.z),
                'decoded': (dec.x, dec.y, dec.z),
                'error_xy': error_xy,
                'error_z': error_z
            })
        
        # Aggregate statistics
        xy_errors = [e['error_xy'] for e in point_errors]
        
        result = {
            'num_original': len(waypoints_3d),
            'num_encoded': len(waypoints_2d),
            'num_decoded': len(waypoints_3d_decoded),
            'point_errors': point_errors,
            'mean_error': np.mean(xy_errors) if xy_errors else 0.0,
            'max_error': np.max(xy_errors) if xy_errors else 0.0,
            'min_error': np.min(xy_errors) if xy_errors else 0.0,
            'std_error': np.std(xy_errors) if xy_errors else 0.0,
            'pass': np.mean(xy_errors) < 1.0 if xy_errors else False  # < 1m is passing
        }
        
        self.errors.extend(xy_errors)
        return result
    
    def get_summary(self) -> Dict:
        """Get summary of all validation runs."""
        if not self.errors:
            return {'status': 'No validations run'}
        
        return {
            'total_points': len(self.errors),
            'mean_error': np.mean(self.errors),
            'max_error': np.max(self.errors),
            'std_error': np.std(self.errors),
            'under_0.5m': sum(1 for e in self.errors if e < 0.5) / len(self.errors) * 100,
            'under_1.0m': sum(1 for e in self.errors if e < 1.0) / len(self.errors) * 100
        }


# =============================================================================
# CONVENIENCE FUNCTIONS
# =============================================================================

def create_pipeline(width: int = 960, height: int = 540, fov: float = 90.0,
                   cam_x: float = 2.0, cam_z: float = 1.8, cam_pitch: float = -15.0,
                   target_speed_kmh: float = 30.0) -> TrajectoryPipeline:
    """
    Create a trajectory pipeline with common settings.
    
    Args:
        width, height: Camera resolution
        fov: Camera field of view
        cam_x, cam_z: Camera position (forward, height)
        cam_pitch: Camera pitch angle
        target_speed_kmh: Target speed for controller
    
    Returns:
        Configured TrajectoryPipeline
    """
    camera_config = CameraConfig(
        width=width,
        height=height,
        fov=fov,
        x=cam_x,
        y=0.0,
        z=cam_z,
        pitch=cam_pitch,
        yaw=0.0,
        roll=0.0
    )
    
    controller_config = ControllerConfig(
        target_speed_kmh=target_speed_kmh
    )
    
    return TrajectoryPipeline(camera_config, controller_config)


# =============================================================================
# TEST
# =============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("TRAJECTORY PIPELINE TEST")
    print("=" * 60)
    
    # Create pipeline
    pipeline = create_pipeline()
    print("\n✓ Pipeline created")
    
    # Create mock vehicle transform
    class MockTransform:
        class Location:
            x, y, z = 0.0, 0.0, 0.5
        class Rotation:
            pitch, yaw, roll = 0.0, 0.0, 0.0
        location = Location()
        rotation = Rotation()
    
    vehicle_transform = MockTransform()
    
    # Create test waypoints (straight road)
    test_waypoints_3d = [
        Waypoint3D(x=5.0, y=0.0, z=0.0),
        Waypoint3D(x=10.0, y=0.0, z=0.0),
        Waypoint3D(x=15.0, y=0.0, z=0.0),
        Waypoint3D(x=20.0, y=0.0, z=0.0),
        Waypoint3D(x=25.0, y=0.0, z=0.0),
    ]
    
    print(f"\n1. Original 3D Waypoints:")
    for i, wp in enumerate(test_waypoints_3d):
        print(f"   [{i+1}] x={wp.x:6.1f}, y={wp.y:6.1f}, z={wp.z:6.1f}")
    
    # Run pipeline
    waypoints_2d, waypoints_3d_decoded = pipeline.process(
        test_waypoints_3d, vehicle_transform, road_height=0.0
    )
    
    print(f"\n2. Encoded 2D Waypoints (pixels):")
    for i, wp in enumerate(waypoints_2d):
        print(f"   [{i+1}] u={wp.u:6.1f}, v={wp.v:6.1f}")
    
    print(f"\n3. Decoded 3D Waypoints:")
    for i, wp in enumerate(waypoints_3d_decoded):
        print(f"   [{i+1}] x={wp.x:6.1f}, y={wp.y:6.1f}, z={wp.z:6.1f}")
    
    # Calculate errors
    print(f"\n4. Reconstruction Errors:")
    for i in range(min(len(test_waypoints_3d), len(waypoints_3d_decoded))):
        orig = test_waypoints_3d[i]
        dec = waypoints_3d_decoded[i]
        error = math.sqrt((orig.x - dec.x)**2 + (orig.y - dec.y)**2)
        status = "✓" if error < 0.5 else "✗"
        print(f"   [{i+1}] error = {error:.4f}m {status}")
    
    # Test with curved trajectory
    print(f"\n" + "=" * 60)
    print("CURVED TRAJECTORY TEST")
    print("=" * 60)
    
    curved_waypoints = [
        Waypoint3D(x=5.0, y=0.0, z=0.0),
        Waypoint3D(x=9.0, y=-1.5, z=0.0),
        Waypoint3D(x=12.0, y=-4.0, z=0.0),
        Waypoint3D(x=14.0, y=-7.0, z=0.0),
    ]
    
    print(f"\nOriginal curved path:")
    for i, wp in enumerate(curved_waypoints):
        print(f"   [{i+1}] x={wp.x:6.1f}, y={wp.y:6.1f}")
    
    waypoints_2d_curved, decoded_curved = pipeline.process(
        curved_waypoints, vehicle_transform, road_height=0.0
    )
    
    print(f"\n2D encoded (should curve left - lower u):")
    for i, wp in enumerate(waypoints_2d_curved):
        print(f"   [{i+1}] u={wp.u:6.1f}, v={wp.v:6.1f}")
    
    # Test control computation
    print(f"\n" + "=" * 60)
    print("CONTROL COMPUTATION TEST")
    print("=" * 60)
    
    control, debug = pipeline.process_and_control(
        curved_waypoints, vehicle_transform, current_speed_kmh=25.0
    )
    
    print(f"\nControl output (following decoded trajectory):")
    print(f"   Throttle: {control.throttle:.3f}")
    print(f"   Brake:    {control.brake:.3f}")
    print(f"   Steer:    {control.steer:.3f}  (negative = turn left)")
    print(f"\n   Reconstruction error: {debug['reconstruction_error']:.4f}m")
    
    # Pipeline stats
    print(f"\n" + "=" * 60)
    print("PIPELINE STATISTICS")
    print("=" * 60)
    stats = pipeline.get_stats()
    print(f"   Encode operations: {stats['encode_count']}")
    print(f"   Decode operations: {stats['decode_count']}")
    print(f"   Average error:     {stats['avg_error']:.4f}m")
    print(f"   Maximum error:     {stats['max_error']:.4f}m")
    
    print(f"\n✓ Pipeline test complete!")
