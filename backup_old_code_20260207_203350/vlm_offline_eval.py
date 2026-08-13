#!/usr/bin/env python3
"""
VLM Offline Evaluation Script

Evaluates VLM trajectory predictions against ground truth from HuggingFace dataset.
Runs offline without CARLA simulation, directly comparing predicted trajectories
with ground truth pixel coordinates.

Usage:
    # Evaluate on dataset samples
    python core/vlm_offline_eval.py --dataset VishwanathAS/your-dataset --samples 100

    # Evaluate with local VLM server
    python core/vlm_offline_eval.py --dataset VishwanathAS/your-dataset --vlm-preset local

    # Evaluate specific route
    python core/vlm_offline_eval.py --dataset VishwanathAS/your-dataset --route left_1
"""

import argparse
import json
import numpy as np
import math
import time
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import matplotlib.pyplot as plt
from PIL import Image

# Import VLM predictor
from vlm_inference import VLMTrajectoryPredictor

# Try to import datasets library
try:
    from datasets import load_dataset
    HF_AVAILABLE = True
except ImportError:
    print("Warning: datasets package not installed. Run: pip install datasets")
    HF_AVAILABLE = False

# Mock CARLA objects for offline evaluation (no simulator needed)
class MockCarlaLocation:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class MockCarlaRotation:
    def __init__(self, pitch=0, yaw=0, roll=0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

class MockCarlaTransform:
    def __init__(self, location=None, rotation=None):
        self.location = location or MockCarlaLocation()
        self.rotation = rotation or MockCarlaRotation()

# Create mock carla module for offline use
class MockCarla:
    Location = MockCarlaLocation
    Rotation = MockCarlaRotation
    Transform = MockCarlaTransform

# Use mock CARLA for offline evaluation
import sys
sys.modules['carla'] = MockCarla()


# =============================================================================
# EVALUATION METRICS
# =============================================================================

@dataclass
class EvaluationMetrics:
    """Metrics for trajectory evaluation."""
    sample_id: int
    route_name: str
    speed: float
    navigation: str

    # Prediction results
    predicted_trajectory: List[List[float]]
    ground_truth_trajectory: List[List[float]]
    inference_time: float

    # 2D Error metrics (pixels)
    endpoint_error_2d: float  # Distance to final waypoint
    average_error_2d: float   # Average distance across all waypoints
    max_error_2d: float       # Maximum distance error
    lateral_error_2d: float   # Average lateral (x-axis) error
    longitudinal_error_2d: float  # Average longitudinal (y-axis) error

    # 3D Error metrics (meters)
    endpoint_error_3d: float  # Distance to final waypoint
    average_error_3d: float   # Average distance across all waypoints
    max_error_3d: float       # Maximum distance error
    lateral_error_3d: float   # Average lateral (x-axis) error
    longitudinal_error_3d: float  # Average longitudinal (y-axis) error


def compute_endpoint_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute error to final waypoint."""
    if not pred or not gt:
        return float('inf')

    pred_end = pred[-1]
    gt_end = gt[-1]

    dx = pred_end[0] - gt_end[0]
    dy = pred_end[1] - gt_end[1]

    return math.sqrt(dx * dx + dy * dy)


def compute_average_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute average L2 distance between predicted and ground truth waypoints."""
    if not pred or not gt:
        return float('inf')

    # Match waypoints (use minimum length)
    min_len = min(len(pred), len(gt))

    errors = []
    for i in range(min_len):
        dx = pred[i][0] - gt[i][0]
        dy = pred[i][1] - gt[i][1]
        error = math.sqrt(dx * dx + dy * dy)
        errors.append(error)

    return sum(errors) / len(errors) if errors else float('inf')


def compute_max_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute maximum L2 distance between predicted and ground truth waypoints."""
    if not pred or not gt:
        return float('inf')

    min_len = min(len(pred), len(gt))

    max_err = 0.0
    for i in range(min_len):
        dx = pred[i][0] - gt[i][0]
        dy = pred[i][1] - gt[i][1]
        error = math.sqrt(dx * dx + dy * dy)
        max_err = max(max_err, error)

    return max_err


def compute_lateral_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute average lateral (x-axis) error."""
    if not pred or not gt:
        return float('inf')

    min_len = min(len(pred), len(gt))

    errors = [abs(pred[i][0] - gt[i][0]) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


def compute_longitudinal_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute average longitudinal (y-axis) error."""
    if not pred or not gt:
        return float('inf')

    min_len = min(len(pred), len(gt))

    errors = [abs(pred[i][1] - gt[i][1]) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


# =============================================================================
# 3D ERROR COMPUTATION (WORLD SPACE)
# =============================================================================

def compute_endpoint_error_3d(pred_3d: List, gt_3d: List) -> float:
    """Compute 3D endpoint error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    pred_end = pred_3d[-1]
    gt_end = gt_3d[-1]

    dx = pred_end.x - gt_end.x
    dy = pred_end.y - gt_end.y

    return math.sqrt(dx * dx + dy * dy)


def compute_average_error_3d(pred_3d: List, gt_3d: List) -> float:
    """Compute average 3D error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))

    errors = []
    for i in range(min_len):
        dx = pred_3d[i].x - gt_3d[i].x
        dy = pred_3d[i].y - gt_3d[i].y
        error = math.sqrt(dx * dx + dy * dy)
        errors.append(error)

    return sum(errors) / len(errors) if errors else float('inf')


def compute_max_error_3d(pred_3d: List, gt_3d: List) -> float:
    """Compute maximum 3D error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))

    max_err = 0.0
    for i in range(min_len):
        dx = pred_3d[i].x - gt_3d[i].x
        dy = pred_3d[i].y - gt_3d[i].y
        error = math.sqrt(dx * dx + dy * dy)
        max_err = max(max_err, error)

    return max_err


def compute_lateral_error_3d(pred_3d: List, gt_3d: List) -> float:
    """Compute average lateral (x-axis) error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))

    errors = [abs(pred_3d[i].x - gt_3d[i].x) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


def compute_longitudinal_error_3d(pred_3d: List, gt_3d: List) -> float:
    """Compute average longitudinal (y-axis) error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))

    errors = [abs(pred_3d[i].y - gt_3d[i].y) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


# =============================================================================
# VLM OFFLINE EVALUATOR
# =============================================================================

class VLMOfflineEvaluator:
    """Evaluates VLM predictions against ground truth from dataset."""

    def __init__(self,
                 vlm_base_url: str = "http://100.74.179.20:8000/v1",
                 vlm_model: str = "VishwanathAS/Qwen3-VLA-Driver-base",
                 output_dir: str = None):
        """
        Args:
            vlm_base_url: VLM API endpoint
            vlm_model: VLM model name
            output_dir: Directory to save evaluation results
        """
        self.predictor = VLMTrajectoryPredictor(
            base_url=vlm_base_url,
            model=vlm_model
        )

        # Import trajectory decoder for 3D conversion
        from traj_planner import TrajectoryDecoder, CameraConfig, Waypoint2D

        # Camera config (matching vlm_auto_drive.py)
        self.camera_config = CameraConfig(
            width=640,
            height=480,
            fov=90,
            x=2.0,
            y=0.0,
            z=1.8,
            pitch=-15,
            yaw=0,
            roll=0
        )
        self.decoder = TrajectoryDecoder(self.camera_config)

        # Auto-generate output directory
        if output_dir is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = f"./eval_results/offline_eval_{timestamp}"

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.metrics_list: List[EvaluationMetrics] = []

    def evaluate_sample(self, sample: Dict) -> EvaluationMetrics:
        """
        Evaluate a single dataset sample.

        Args:
            sample: Dataset sample with keys:
                - image: PIL Image
                - trajectory: Ground truth trajectory (string or list) [[x, y, d], ...]
                - speed_kmh: Current speed in km/h
                - navigation_command: Navigation command
                - route_name: Route identifier

        Returns:
            EvaluationMetrics for this sample
        """
        # Extract sample data
        image_pil = sample['image']
        gt_trajectory_raw = sample['trajectory']  # May be string or list
        speed = sample.get('speed_kmh', sample.get('speed', 30.0))
        navigation = sample.get('navigation_command', sample.get('navigation', 'lane_keeping'))
        route_name = sample.get('route_name', 'unknown')

        # Convert PIL image to numpy
        image_np = np.array(image_pil)

        # Parse trajectory if it's a string
        if isinstance(gt_trajectory_raw, str):
            import ast
            gt_trajectory_full = ast.literal_eval(gt_trajectory_raw)
        else:
            gt_trajectory_full = gt_trajectory_raw

        # Extract only x, y from ground truth (remove distance if present)
        if len(gt_trajectory_full[0]) >= 3:
            gt_trajectory = [[pt[0], pt[1]] for pt in gt_trajectory_full]
        else:
            gt_trajectory = gt_trajectory_full

        # Run VLM prediction
        pred_trajectory, inference_time = self.predictor.predict(
            image_np,
            current_speed=speed,
            navigation=navigation
        )

        # Compute 2D errors (pixels)
        endpoint_err_2d = compute_endpoint_error(pred_trajectory, gt_trajectory)
        avg_err_2d = compute_average_error(pred_trajectory, gt_trajectory)
        max_err_2d = compute_max_error(pred_trajectory, gt_trajectory)
        lateral_err_2d = compute_lateral_error(pred_trajectory, gt_trajectory)
        longitudinal_err_2d = compute_longitudinal_error(pred_trajectory, gt_trajectory)

        # Convert to 3D for world-space comparison
        from traj_planner import Waypoint2D, Waypoint3D
        import carla

        # Create dummy vehicle transform (we don't have actual vehicle position from dataset)
        # Assume vehicle at origin with forward orientation
        vehicle_transform = carla.Transform(
            carla.Location(x=0, y=0, z=1.0),
            carla.Rotation(pitch=0, yaw=0, roll=0)
        )
        road_height = 0.5  # Estimate

        # Convert predicted 2D → 3D
        pred_waypoints_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in pred_trajectory] if pred_trajectory else []
        pred_waypoints_3d = self.decoder.decode(pred_waypoints_2d, vehicle_transform, road_height) if pred_waypoints_2d else []

        # Convert ground truth 2D → 3D
        gt_waypoints_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in gt_trajectory]
        gt_waypoints_3d = self.decoder.decode(gt_waypoints_2d, vehicle_transform, road_height)

        # Compute 3D errors (meters)
        endpoint_err_3d = compute_endpoint_error_3d(pred_waypoints_3d, gt_waypoints_3d)
        avg_err_3d = compute_average_error_3d(pred_waypoints_3d, gt_waypoints_3d)
        max_err_3d = compute_max_error_3d(pred_waypoints_3d, gt_waypoints_3d)
        lateral_err_3d = compute_lateral_error_3d(pred_waypoints_3d, gt_waypoints_3d)
        longitudinal_err_3d = compute_longitudinal_error_3d(pred_waypoints_3d, gt_waypoints_3d)

        return EvaluationMetrics(
            sample_id=len(self.metrics_list),
            route_name=route_name,
            speed=speed,
            navigation=navigation,
            predicted_trajectory=pred_trajectory,
            ground_truth_trajectory=gt_trajectory,
            inference_time=inference_time,
            endpoint_error_2d=endpoint_err_2d,
            average_error_2d=avg_err_2d,
            max_error_2d=max_err_2d,
            lateral_error_2d=lateral_err_2d,
            longitudinal_error_2d=longitudinal_err_2d,
            endpoint_error_3d=endpoint_err_3d,
            average_error_3d=avg_err_3d,
            max_error_3d=max_err_3d,
            lateral_error_3d=lateral_err_3d,
            longitudinal_error_3d=longitudinal_err_3d
        )

    def evaluate_dataset(self,
                        dataset_name: str,
                        split: str = 'train',
                        num_samples: int = None,
                        route_filter: str = None):
        """
        Evaluate VLM on HuggingFace dataset or local dataset directory.

        Args:
            dataset_name: HuggingFace dataset identifier OR local path to dataset directory
            split: Dataset split ('train', 'test', etc.)
            num_samples: Number of samples to evaluate (None = all)
            route_filter: Only evaluate samples from this route
        """
        if not HF_AVAILABLE:
            raise ImportError("datasets package not installed. Run: pip install datasets")

        print(f"\n{'='*70}")
        print(f"VLM OFFLINE EVALUATION")
        print(f"{'='*70}")
        print(f"Dataset: {dataset_name}")
        print(f"Split: {split}")
        print(f"Output: {self.output_dir}")
        print(f"{'='*70}\n")

        # Load dataset
        print("Loading dataset...")

        # Check if it's a local path
        dataset_path = Path(dataset_name)
        if dataset_path.exists() and dataset_path.is_dir():
            print(f"Loading from local directory: {dataset_path}")
            from datasets import load_from_disk
            # Load all Arrow files using load_from_disk
            dataset = load_from_disk(str(dataset_path))
            print(f"Loaded {len(dataset)} samples from {dataset_path}")
        else:
            print(f"Loading from HuggingFace: {dataset_name}")
            dataset = load_dataset(dataset_name, split=split)

        # Filter by route if specified
        if route_filter:
            dataset = dataset.filter(lambda x: x.get('route_name') == route_filter)
            print(f"Filtered to route: {route_filter}")

        # Limit samples if specified
        if num_samples:
            dataset = dataset.select(range(min(num_samples, len(dataset))))

        total_samples = len(dataset)
        print(f"Evaluating {total_samples} samples...\n")

        # Evaluate each sample
        start_time = time.time()
        for i, sample in enumerate(dataset):
            metrics = self.evaluate_sample(sample)
            self.metrics_list.append(metrics)

            # Progress update
            if (i + 1) % 10 == 0 or i == total_samples - 1:
                elapsed = time.time() - start_time
                avg_time_per_sample = elapsed / (i + 1)
                eta = avg_time_per_sample * (total_samples - i - 1)

                print(f"Progress: {i+1}/{total_samples} | "
                      f"2D: {metrics.average_error_2d:.1f}px / 3D: {metrics.average_error_3d:.1f}m | "
                      f"ETA: {eta/60:.1f}min")

        elapsed = time.time() - start_time
        print(f"\n{'='*70}")
        print(f"Evaluation complete! Time: {elapsed/60:.1f} minutes")
        print(f"{'='*70}\n")

        # Generate summary
        self.print_summary()
        self.save_results()

    def print_summary(self):
        """Print evaluation summary statistics."""
        if not self.metrics_list:
            print("No evaluation results to summarize.")
            return

        # Aggregate 2D metrics (pixels)
        endpoint_errors_2d = [m.endpoint_error_2d for m in self.metrics_list if math.isfinite(m.endpoint_error_2d)]
        avg_errors_2d = [m.average_error_2d for m in self.metrics_list if math.isfinite(m.average_error_2d)]
        lateral_errors_2d = [m.lateral_error_2d for m in self.metrics_list if math.isfinite(m.lateral_error_2d)]
        longitudinal_errors_2d = [m.longitudinal_error_2d for m in self.metrics_list if math.isfinite(m.longitudinal_error_2d)]

        # Aggregate 3D metrics (meters)
        endpoint_errors_3d = [m.endpoint_error_3d for m in self.metrics_list if math.isfinite(m.endpoint_error_3d)]
        avg_errors_3d = [m.average_error_3d for m in self.metrics_list if math.isfinite(m.average_error_3d)]
        lateral_errors_3d = [m.lateral_error_3d for m in self.metrics_list if math.isfinite(m.lateral_error_3d)]
        longitudinal_errors_3d = [m.longitudinal_error_3d for m in self.metrics_list if math.isfinite(m.longitudinal_error_3d)]

        inference_times = [m.inference_time for m in self.metrics_list]

        print("EVALUATION SUMMARY")
        print(f"{'='*70}")
        print(f"Total Samples: {len(self.metrics_list)}")

        print(f"\n{'─'*70}")
        print("2D ERRORS (Pixel Space)")
        print(f"{'─'*70}")
        print(f"Endpoint Error:")
        print(f"  Mean: {np.mean(endpoint_errors_2d):.2f} px  |  Median: {np.median(endpoint_errors_2d):.2f} px  |  Std: {np.std(endpoint_errors_2d):.2f} px")
        print(f"Average Trajectory Error:")
        print(f"  Mean: {np.mean(avg_errors_2d):.2f} px  |  Median: {np.median(avg_errors_2d):.2f} px  |  Std: {np.std(avg_errors_2d):.2f} px")
        print(f"Lateral Error:")
        print(f"  Mean: {np.mean(lateral_errors_2d):.2f} px")
        print(f"Longitudinal Error:")
        print(f"  Mean: {np.mean(longitudinal_errors_2d):.2f} px")

        print(f"\n{'─'*70}")
        print("3D ERRORS (World Space - Meters)")
        print(f"{'─'*70}")
        print(f"Endpoint Error:")
        print(f"  Mean: {np.mean(endpoint_errors_3d):.2f} m  |  Median: {np.median(endpoint_errors_3d):.2f} m  |  Std: {np.std(endpoint_errors_3d):.2f} m")
        print(f"Average Trajectory Error:")
        print(f"  Mean: {np.mean(avg_errors_3d):.2f} m  |  Median: {np.median(avg_errors_3d):.2f} m  |  Std: {np.std(avg_errors_3d):.2f} m")
        print(f"Lateral Error (left/right):")
        print(f"  Mean: {np.mean(lateral_errors_3d):.2f} m")
        print(f"Longitudinal Error (forward/back):")
        print(f"  Mean: {np.mean(longitudinal_errors_3d):.2f} m")

        print(f"\n{'─'*70}")
        print(f"Inference Time:")
        print(f"  Mean: {np.mean(inference_times):.3f} s  |  Median: {np.median(inference_times):.3f} s")
        print(f"{'='*70}\n")

        # Per-route breakdown
        routes = {}
        for m in self.metrics_list:
            if m.route_name not in routes:
                routes[m.route_name] = []
            routes[m.route_name].append(m)

        if len(routes) > 1:
            print("\nPER-ROUTE BREAKDOWN:")
            print(f"{'='*70}")
            for route_name, route_metrics in sorted(routes.items()):
                route_avg_errors = [m.average_error for m in route_metrics if math.isfinite(m.average_error)]
                route_endpoint_errors = [m.endpoint_error for m in route_metrics if math.isfinite(m.endpoint_error)]

                print(f"\n{route_name} ({len(route_metrics)} samples):")
                print(f"  Avg Error: {np.mean(route_avg_errors):.2f}px")
                print(f"  Endpoint Error: {np.mean(route_endpoint_errors):.2f}px")
            print(f"\n{'='*70}\n")

    def save_results(self):
        """Save evaluation results to JSON."""
        results = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'total_samples': len(self.metrics_list),
                'vlm_model': self.predictor.model
            },
            'summary': {
                '2d_errors_px': {
                    'endpoint_error_mean': np.mean([m.endpoint_error_2d for m in self.metrics_list if math.isfinite(m.endpoint_error_2d)]),
                    'average_error_mean': np.mean([m.average_error_2d for m in self.metrics_list if math.isfinite(m.average_error_2d)]),
                    'lateral_error_mean': np.mean([m.lateral_error_2d for m in self.metrics_list if math.isfinite(m.lateral_error_2d)]),
                    'longitudinal_error_mean': np.mean([m.longitudinal_error_2d for m in self.metrics_list if math.isfinite(m.longitudinal_error_2d)]),
                },
                '3d_errors_m': {
                    'endpoint_error_mean': np.mean([m.endpoint_error_3d for m in self.metrics_list if math.isfinite(m.endpoint_error_3d)]),
                    'average_error_mean': np.mean([m.average_error_3d for m in self.metrics_list if math.isfinite(m.average_error_3d)]),
                    'lateral_error_mean': np.mean([m.lateral_error_3d for m in self.metrics_list if math.isfinite(m.lateral_error_3d)]),
                    'longitudinal_error_mean': np.mean([m.longitudinal_error_3d for m in self.metrics_list if math.isfinite(m.longitudinal_error_3d)]),
                },
                'inference_time_mean': np.mean([m.inference_time for m in self.metrics_list]),
            },
            'samples': [
                {
                    'sample_id': m.sample_id,
                    'route_name': m.route_name,
                    'speed': m.speed,
                    'navigation': m.navigation,
                    'predicted_trajectory': m.predicted_trajectory,
                    'ground_truth_trajectory': m.ground_truth_trajectory,
                    'inference_time': m.inference_time,
                    '2d_errors_px': {
                        'endpoint_error': m.endpoint_error_2d,
                        'average_error': m.average_error_2d,
                        'max_error': m.max_error_2d,
                        'lateral_error': m.lateral_error_2d,
                        'longitudinal_error': m.longitudinal_error_2d
                    },
                    '3d_errors_m': {
                        'endpoint_error': m.endpoint_error_3d,
                        'average_error': m.average_error_3d,
                        'max_error': m.max_error_3d,
                        'lateral_error': m.lateral_error_3d,
                        'longitudinal_error': m.longitudinal_error_3d
                    }
                }
                for m in self.metrics_list
            ]
        }

        output_file = self.output_dir / 'evaluation_results.json'
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"Results saved to: {output_file}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='VLM Offline Evaluation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Evaluate local dataset (default)
  python core/vlm_offline_eval.py --samples 100

  # Evaluate with local VLM server
  python core/vlm_offline_eval.py --vlm-preset local --samples 50

  # Evaluate specific route
  python core/vlm_offline_eval.py --route left_1 --samples 50

  # Evaluate from HuggingFace
  python core/vlm_offline_eval.py --dataset VishwanathAS/your-dataset --split test --samples 100

  # Evaluate custom local path
  python core/vlm_offline_eval.py --dataset ./path/to/dataset --samples 50
        """
    )

    # Default to hf_trajectory_dataset in same directory as this script
    default_dataset_path = str(Path(__file__).parent / 'hf_trajectory_dataset')
    parser.add_argument('--dataset', type=str, default=default_dataset_path,
                        help='HuggingFace dataset identifier OR local path to dataset directory (default: core/hf_trajectory_dataset)')

    parser.add_argument('--split', type=str, default='train',
                        help='Dataset split to evaluate (default: train)')

    parser.add_argument('--samples', type=int, default=None,
                        help='Number of samples to evaluate (default: all)')

    parser.add_argument('--route', type=str, default=None,
                        help='Filter to specific route name (optional)')

    parser.add_argument('--vlm-preset', type=str, default='cloud',
                        choices=['local', 'cloud'],
                        help='VLM endpoint preset (default: cloud)')

    parser.add_argument('--vlm-url', type=str, default=None,
                        help='Custom VLM API endpoint URL (overrides preset)')

    parser.add_argument('--vlm-model', type=str, default=None,
                        help='Custom VLM model name (overrides preset)')

    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for results (default: auto-generated)')

    args = parser.parse_args()

    # VLM endpoint presets
    VLM_ENDPOINTS = {
        'local': {
            'url': 'https://vishwanaths-mac-mini.woodpecker-bluegill.ts.net/v1',
            'model': 'qwen3-vl-driver-base'
        },
        'cloud': {
            # 'url': 'https://vish2kber--vlm-inference-serve.modal.run/v1',
            'url': 'http://100.74.179.20:8000/v1',
            'model': 'VishwanathAS/Qwen3-VLA-Driver-base'
        }
    }

    # Resolve VLM endpoint
    preset = VLM_ENDPOINTS[args.vlm_preset]
    vlm_url = args.vlm_url or preset['url']
    vlm_model = args.vlm_model or preset['model']

    # Create evaluator
    evaluator = VLMOfflineEvaluator(
        vlm_base_url=vlm_url,
        vlm_model=vlm_model,
        output_dir=args.output_dir
    )

    # Run evaluation
    evaluator.evaluate_dataset(
        dataset_name=args.dataset,
        split=args.split,
        num_samples=args.samples,
        route_filter=args.route
    )


if __name__ == '__main__':
    main()
