"""
Core Modules for Autonomous Driving System

This package provides 7 clean, single-responsibility modules:

1. PID Controller (pid_controller.py)
   - VehiclePIDController: Vehicle control for waypoint following

2. Trajectory Planner (traj_planner.py)
   - CameraConfig: Camera configuration
   - TrajectoryEncoder: 3D to 2D projection
   - TrajectoryDecoder: 2D to 3D reconstruction
   - WaypointGenerator: Generate waypoints from CARLA map

3. VLM Inference (vlm_inference.py)
   - VLMTrajectoryPredictor: Synchronous VLM prediction
   - AsyncVLMPredictor: Async VLM prediction with buffering

4. Route Builder (route_builder.py)
   - RouteBuilder: Load routes and extract waypoints
   - RouteCheckpoint: Checkpoint data structure
   - Route: Complete route with metadata

5. Main Driver (vlm_pure_drive.py)
   - PureVLMDriver: VLM-based autonomous driving
   - run_pure_vlm_drive: Main entry point

6. Dataset Recorder (dataset_recorder.py)
   - DatasetRecorder: Record driving datasets
   - RecorderConfig: Configuration for recording

7. Offline Tester (offline_tester.py)
   - OfflineTester: Evaluate VLM on recorded data
   - EvaluationMetrics: Per-sample metrics
   - EvaluationSummary: Summary statistics

Usage:
    # Import individual modules
    from core.pid_controller import VehiclePIDController
    from core.route_builder import RouteBuilder
    from core.traj_planner import TrajectoryEncoder, TrajectoryDecoder
    from core.vlm_inference import VLMTrajectoryPredictor
    from core.dataset_recorder import DatasetRecorder
    from core.offline_tester import OfflineTester

    # Or import from package
    from core import (
        VehiclePIDController,
        RouteBuilder,
        TrajectoryEncoder,
        VLMTrajectoryPredictor,
    )
"""

# Version
__version__ = "1.0.0"

# Module imports for convenience
from .pid_controller import VehiclePIDController
from .route_builder import RouteBuilder, RouteCheckpoint, Route, Waypoint3D
from .traj_planner import (
    CameraConfig,
    TrajectoryEncoder,
    TrajectoryDecoder,
    WaypointGenerator,
    Waypoint2D,
    Waypoint3D as TrajWaypoint3D,
)
from .vlm_inference import VLMTrajectoryPredictor, AsyncVLMPredictor, trajectory_to_waypoints_2d
from .dataset_recorder import DatasetRecorder, RecorderConfig
from .offline_tester import OfflineTester, EvaluationMetrics, EvaluationSummary

__all__ = [
    # PID Controller
    "VehiclePIDController",
    # Route Builder
    "RouteBuilder",
    "RouteCheckpoint",
    "Route",
    "Waypoint3D",
    # Trajectory Planner
    "CameraConfig",
    "TrajectoryEncoder",
    "TrajectoryDecoder",
    "WaypointGenerator",
    "Waypoint2D",
    "TrajWaypoint3D",
    # VLM Inference
    "VLMTrajectoryPredictor",
    "AsyncVLMPredictor",
    "trajectory_to_waypoints_2d",
    # Dataset Recorder
    "DatasetRecorder",
    "RecorderConfig",
    # Offline Tester
    "OfflineTester",
    "EvaluationMetrics",
    "EvaluationSummary",
]
