#!/usr/bin/env python3
"""
VLM Dataset Validator - Evaluate VLM predictions using raw dataset JSON labels.

This script reads the original JSON label files (core/datasets/labels/*.json)
instead of the simplified Arrow dataset, allowing us to:

1. Use actual vehicle transforms for accurate 3D error computation
2. Access trajectory_3d ground truth directly
3. Validate by route
4. Compare both 2D pixel-space and 3D world-space errors accurately

Usage:
    # Validate on first 100 samples
    python core/vlm_dataset_validator.py --samples 100

    # Validate specific route
    python core/vlm_dataset_validator.py --route left_1 --samples 50

    # Use local VLM server
    python core/vlm_dataset_validator.py --vlm-preset local --samples 20
"""

import json
import math
import numpy as np
from pathlib import Path
from PIL import Image
from typing import List, Dict, Tuple
from dataclasses import dataclass
import time
import argparse
from datetime import datetime

# Import VLM predictor
from vlm_inference import VLMTrajectoryPredictor

# Mock CARLA for offline use
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

class MockCarla:
    Location = MockCarlaLocation
    Rotation = MockCarlaRotation
    Transform = MockCarlaTransform

import sys
sys.modules['carla'] = MockCarla()


# =============================================================================
# VALIDATION METRICS
# =============================================================================

@dataclass
class ValidationMetrics:
    """Validation metrics for a single sample."""
    sample_id: str
    route_name: str
    speed_kmh: float
    navigation_command: str

    # Predictions
    predicted_trajectory_2d: List[List[float]]
    ground_truth_2d: List[List[float]]
    ground_truth_3d: List[List[float]]

    # 2D Errors (pixels)
    endpoint_error_2d: float
    average_error_2d: float
    lateral_error_2d: float
    longitudinal_error_2d: float

    # 3D Errors (meters) - Using actual vehicle transform
    endpoint_error_3d: float
    average_error_3d: float
    lateral_error_3d: float
    longitudinal_error_3d: float

    # Metadata
    inference_time: float
    vehicle_location: List[float]
    vehicle_rotation: List[float]


def compute_2d_endpoint_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute 2D endpoint error in pixels."""
    if not pred or not gt:
        return float('inf')

    pred_end = pred[-1]
    gt_end = gt[-1]

    dx = pred_end[0] - gt_end[0]
    dy = pred_end[1] - gt_end[1]

    return math.sqrt(dx * dx + dy * dy)


def compute_2d_average_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute average 2D error in pixels."""
    if not pred or not gt:
        return float('inf')

    min_len = min(len(pred), len(gt))

    errors = []
    for i in range(min_len):
        dx = pred[i][0] - gt[i][0]
        dy = pred[i][1] - gt[i][1]
        error = math.sqrt(dx * dx + dy * dy)
        errors.append(error)

    return sum(errors) / len(errors) if errors else float('inf')


def compute_2d_lateral_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute average lateral (x-axis) error in pixels."""
    if not pred or not gt:
        return float('inf')

    min_len = min(len(pred), len(gt))
    errors = [abs(pred[i][0] - gt[i][0]) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


def compute_2d_longitudinal_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute average longitudinal (y-axis) error in pixels."""
    if not pred or not gt:
        return float('inf')

    min_len = min(len(pred), len(gt))
    errors = [abs(pred[i][1] - gt[i][1]) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


def compute_3d_endpoint_error(pred_3d: List[List[float]], gt_3d: List[List[float]]) -> float:
    """Compute 3D endpoint error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    pred_end = pred_3d[-1]
    gt_end = gt_3d[-1]

    dx = pred_end[0] - gt_end[0]
    dy = pred_end[1] - gt_end[1]

    return math.sqrt(dx * dx + dy * dy)


def compute_3d_average_error(pred_3d: List[List[float]], gt_3d: List[List[float]]) -> float:
    """Compute average 3D error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))

    errors = []
    for i in range(min_len):
        dx = pred_3d[i][0] - gt_3d[i][0]
        dy = pred_3d[i][1] - gt_3d[i][1]
        error = math.sqrt(dx * dx + dy * dy)
        errors.append(error)

    return sum(errors) / len(errors) if errors else float('inf')


def compute_3d_lateral_error(pred_3d: List[List[float]], gt_3d: List[List[float]]) -> float:
    """Compute average lateral (x-axis) error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))
    errors = [abs(pred_3d[i][0] - gt_3d[i][0]) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


def compute_3d_longitudinal_error(pred_3d: List[List[float]], gt_3d: List[List[float]]) -> float:
    """Compute average longitudinal (y-axis) error in meters."""
    if not pred_3d or not gt_3d:
        return float('inf')

    min_len = min(len(pred_3d), len(gt_3d))
    errors = [abs(pred_3d[i][1] - gt_3d[i][1]) for i in range(min_len)]
    return sum(errors) / len(errors) if errors else float('inf')


# =============================================================================
# VLM DATASET VALIDATOR
# =============================================================================

class VLMDatasetValidator:
    """Validates VLM predictions using raw dataset JSON labels."""

    def __init__(self,
                 vlm_base_url: str = "https://vish2kber--vlm-inference-serve.modal.run/v1",
                 vlm_model: str = "VishwanathAS/Qwen3-VLA-Driver-base",
                 output_dir: str = None):
        """
        Initialize validator.

        Args:
            vlm_base_url: VLM API endpoint
            vlm_model: VLM model name
            output_dir: Directory to save results
        """
        self.predictor = VLMTrajectoryPredictor(
            base_url=vlm_base_url,
            model=vlm_model
        )

        # Import trajectory decoder
        from traj_planner import TrajectoryDecoder, CameraConfig, Waypoint2D

        # Camera config from dataset JSON
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
            output_dir = f"./eval_results/validator_{timestamp}"

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Metrics storage
        self.metrics_list: List[ValidationMetrics] = []

    def validate_sample(self, label_data: Dict, image: Image.Image) -> ValidationMetrics:
        """
        Validate a single sample from dataset.

        Args:
            label_data: Parsed JSON label data
            image: PIL Image

        Returns:
            ValidationMetrics
        """
        # Extract data from label
        sample_id = label_data['id']
        route_name = label_data['vehicle_state']['route_name']
        speed_kmh = label_data['vehicle_state']['speed_kmh']
        navigation_command = label_data['vehicle_state']['navigation_command']

        # Ground truth
        gt_2d = label_data['trajectory_2d']
        gt_3d = label_data['trajectory_3d']

        # Vehicle state
        vehicle_location = label_data['vehicle_state']['location']
        vehicle_rotation = label_data['vehicle_state']['rotation']

        # Convert PIL image to numpy
        image_np = np.array(image)

        # Run VLM prediction
        pred_trajectory_raw, inference_time = self.predictor.predict(
            image_np,
            current_speed=speed_kmh,
            navigation=navigation_command
        )

        # Extract x, y from prediction (ignore distance if present)
        # Handle both [[x,y], ...] and [[x,y,d], ...] formats
        pred_2d = [[pt[0], pt[1]] for pt in pred_trajectory_raw] if pred_trajectory_raw else []

        # Compute 2D errors
        endpoint_err_2d = compute_2d_endpoint_error(pred_2d, gt_2d)
        avg_err_2d = compute_2d_average_error(pred_2d, gt_2d)
        lateral_err_2d = compute_2d_lateral_error(pred_2d, gt_2d)
        longitudinal_err_2d = compute_2d_longitudinal_error(pred_2d, gt_2d)

        # Convert predicted 2D → 3D using ACTUAL vehicle transform
        import carla
        from traj_planner import Waypoint2D

        vehicle_transform = carla.Transform(
            carla.Location(
                x=vehicle_location[0],
                y=vehicle_location[1],
                z=vehicle_location[2]
            ),
            carla.Rotation(
                pitch=vehicle_rotation[0],
                yaw=vehicle_rotation[1],
                roll=vehicle_rotation[2]
            )
        )

        # Assume road height matches vehicle z
        road_height = vehicle_location[2]

        # Convert BOTH predicted 2D and ground truth 2D → 3D using decoder
        # This ensures apples-to-apples comparison
        pred_waypoints_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in pred_2d] if pred_2d else []
        pred_waypoints_3d_decoded = self.decoder.decode(pred_waypoints_2d, vehicle_transform, road_height) if pred_waypoints_2d else []

        gt_waypoints_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in gt_2d]
        gt_waypoints_3d_decoded = self.decoder.decode(gt_waypoints_2d, vehicle_transform, road_height)

        # Convert to list format for comparison
        pred_3d_decoded = [[wp.x, wp.y, 0.0] for wp in pred_waypoints_3d_decoded]
        gt_3d_decoded = [[wp.x, wp.y, 0.0] for wp in gt_waypoints_3d_decoded]

        # Compute 3D errors (both converted from 2D, same coordinate frame)
        endpoint_err_3d = compute_3d_endpoint_error(pred_3d_decoded, gt_3d_decoded)
        avg_err_3d = compute_3d_average_error(pred_3d_decoded, gt_3d_decoded)
        lateral_err_3d = compute_3d_lateral_error(pred_3d_decoded, gt_3d_decoded)
        longitudinal_err_3d = compute_3d_longitudinal_error(pred_3d_decoded, gt_3d_decoded)

        return ValidationMetrics(
            sample_id=sample_id,
            route_name=route_name,
            speed_kmh=speed_kmh,
            navigation_command=navigation_command,
            predicted_trajectory_2d=pred_2d,
            ground_truth_2d=gt_2d,
            ground_truth_3d=gt_3d,
            endpoint_error_2d=endpoint_err_2d,
            average_error_2d=avg_err_2d,
            lateral_error_2d=lateral_err_2d,
            longitudinal_error_2d=longitudinal_err_2d,
            endpoint_error_3d=endpoint_err_3d,
            average_error_3d=avg_err_3d,
            lateral_error_3d=lateral_err_3d,
            longitudinal_error_3d=longitudinal_err_3d,
            inference_time=inference_time,
            vehicle_location=vehicle_location,
            vehicle_rotation=vehicle_rotation
        )

    def validate_dataset(self,
                        dataset_dir: Path,
                        num_samples: int = None,
                        route_filter: str = None):
        """
        Validate VLM on dataset using raw JSON labels.

        Args:
            dataset_dir: Path to core/datasets directory
            num_samples: Number of samples to validate (None = all)
            route_filter: Only validate samples from this route
        """
        labels_dir = dataset_dir / "labels"
        images_dir = dataset_dir / "images"

        print(f"\n{'='*70}")
        print(f"VLM DATASET VALIDATION")
        print(f"{'='*70}")
        print(f"Dataset: {dataset_dir}")
        print(f"VLM: {self.predictor.model}")
        print(f"Output: {self.output_dir}")
        print(f"{'='*70}\n")

        # Get all label files
        label_files = sorted(labels_dir.glob("*.json"))

        if not label_files:
            print(f"No label files found in {labels_dir}")
            return

        print(f"Found {len(label_files)} label files")

        # Filter and limit
        samples_to_process = []
        for label_file in label_files:
            with open(label_file) as f:
                label_data = json.load(f)

            # Filter by route if specified
            if route_filter:
                if label_data['vehicle_state']['route_name'] != route_filter:
                    continue

            samples_to_process.append((label_file, label_data))

            # Limit samples
            if num_samples and len(samples_to_process) >= num_samples:
                break

        total_samples = len(samples_to_process)
        print(f"Validating {total_samples} samples...")
        if route_filter:
            print(f"Route filter: {route_filter}")

        # Validate each sample
        start_time = time.time()

        for i, (label_file, label_data) in enumerate(samples_to_process):
            # Load image
            image_filename = label_data['image_file']
            image_path = images_dir / image_filename

            if not image_path.exists():
                print(f"Warning: Image not found: {image_path}")
                continue

            image = Image.open(image_path)

            # Validate
            metrics = self.validate_sample(label_data, image)
            self.metrics_list.append(metrics)

            # Progress update
            if (i + 1) % 10 == 0 or i == total_samples - 1:
                elapsed = time.time() - start_time
                avg_time_per_sample = elapsed / (i + 1)
                eta = avg_time_per_sample * (total_samples - i - 1)

                print(f"Progress: {i+1}/{total_samples} | "
                      f"2D: {metrics.average_error_2d:.1f}px | "
                      f"3D: {metrics.average_error_3d:.2f}m | "
                      f"ETA: {eta/60:.1f}min")

        elapsed = time.time() - start_time
        print(f"\n{'='*70}")
        print(f"Validation complete! Time: {elapsed/60:.1f} minutes")
        print(f"{'='*70}\n")

        # Generate summary
        self.print_summary()
        self.save_results()

    def print_summary(self):
        """Print validation summary statistics."""
        if not self.metrics_list:
            print("No validation results to summarize.")
            return

        # Aggregate metrics
        endpoint_errors_2d = [m.endpoint_error_2d for m in self.metrics_list if math.isfinite(m.endpoint_error_2d)]
        avg_errors_2d = [m.average_error_2d for m in self.metrics_list if math.isfinite(m.average_error_2d)]
        lateral_errors_2d = [m.lateral_error_2d for m in self.metrics_list if math.isfinite(m.lateral_error_2d)]
        longitudinal_errors_2d = [m.longitudinal_error_2d for m in self.metrics_list if math.isfinite(m.longitudinal_error_2d)]

        endpoint_errors_3d = [m.endpoint_error_3d for m in self.metrics_list if math.isfinite(m.endpoint_error_3d)]
        avg_errors_3d = [m.average_error_3d for m in self.metrics_list if math.isfinite(m.average_error_3d)]
        lateral_errors_3d = [m.lateral_error_3d for m in self.metrics_list if math.isfinite(m.lateral_error_3d)]
        longitudinal_errors_3d = [m.longitudinal_error_3d for m in self.metrics_list if math.isfinite(m.longitudinal_error_3d)]

        inference_times = [m.inference_time for m in self.metrics_list]

        print("VALIDATION SUMMARY")
        print(f"{'='*70}")
        print(f"Total Samples: {len(self.metrics_list)}")

        print(f"\n{'─'*70}")
        print("2D ERRORS (Pixel Space)")
        print(f"{'─'*70}")
        print(f"Endpoint Error:")
        print(f"  Mean: {np.mean(endpoint_errors_2d):.2f} px  |  Median: {np.median(endpoint_errors_2d):.2f} px")
        print(f"Average Trajectory Error:")
        print(f"  Mean: {np.mean(avg_errors_2d):.2f} px  |  Median: {np.median(avg_errors_2d):.2f} px")
        print(f"Lateral Error:")
        print(f"  Mean: {np.mean(lateral_errors_2d):.2f} px")
        print(f"Longitudinal Error:")
        print(f"  Mean: {np.mean(longitudinal_errors_2d):.2f} px")

        print(f"\n{'─'*70}")
        print("3D ERRORS (World Space - Meters) - Using Actual Vehicle Transforms")
        print(f"{'─'*70}")
        print(f"Endpoint Error:")
        print(f"  Mean: {np.mean(endpoint_errors_3d):.2f} m  |  Median: {np.median(endpoint_errors_3d):.2f} m")
        print(f"Average Trajectory Error:")
        print(f"  Mean: {np.mean(avg_errors_3d):.2f} m  |  Median: {np.median(avg_errors_3d):.2f} m")
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
            print("\nPer-Route Breakdown:")
            print(f"{'─'*70}")
            for route_name, route_metrics in routes.items():
                route_avg_2d = [m.average_error_2d for m in route_metrics if math.isfinite(m.average_error_2d)]
                route_avg_3d = [m.average_error_3d for m in route_metrics if math.isfinite(m.average_error_3d)]
                print(f"{route_name}: {len(route_metrics)} samples | "
                      f"2D: {np.mean(route_avg_2d):.1f}px | "
                      f"3D: {np.mean(route_avg_3d):.2f}m")

    def save_results(self):
        """Save validation results to JSON."""
        results = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'total_samples': len(self.metrics_list),
                'vlm_model': self.predictor.model
            },
            'summary': {
                '2d_errors_px': {
                    'endpoint_error_mean': float(np.mean([m.endpoint_error_2d for m in self.metrics_list if math.isfinite(m.endpoint_error_2d)])),
                    'average_error_mean': float(np.mean([m.average_error_2d for m in self.metrics_list if math.isfinite(m.average_error_2d)])),
                    'lateral_error_mean': float(np.mean([m.lateral_error_2d for m in self.metrics_list if math.isfinite(m.lateral_error_2d)])),
                    'longitudinal_error_mean': float(np.mean([m.longitudinal_error_2d for m in self.metrics_list if math.isfinite(m.longitudinal_error_2d)])),
                },
                '3d_errors_m': {
                    'endpoint_error_mean': float(np.mean([m.endpoint_error_3d for m in self.metrics_list if math.isfinite(m.endpoint_error_3d)])),
                    'average_error_mean': float(np.mean([m.average_error_3d for m in self.metrics_list if math.isfinite(m.average_error_3d)])),
                    'lateral_error_mean': float(np.mean([m.lateral_error_3d for m in self.metrics_list if math.isfinite(m.lateral_error_3d)])),
                    'longitudinal_error_mean': float(np.mean([m.longitudinal_error_3d for m in self.metrics_list if math.isfinite(m.longitudinal_error_3d)])),
                },
                'inference_time_mean': float(np.mean([m.inference_time for m in self.metrics_list])),
            },
            'samples': [
                {
                    'sample_id': m.sample_id,
                    'route_name': m.route_name,
                    'speed_kmh': m.speed_kmh,
                    'navigation_command': m.navigation_command,
                    'predicted_trajectory_2d': m.predicted_trajectory_2d,
                    'ground_truth_2d': m.ground_truth_2d,
                    'ground_truth_3d': m.ground_truth_3d,
                    'vehicle_location': m.vehicle_location,
                    'vehicle_rotation': m.vehicle_rotation,
                    'inference_time': m.inference_time,
                    '2d_errors_px': {
                        'endpoint_error': m.endpoint_error_2d,
                        'average_error': m.average_error_2d,
                        'lateral_error': m.lateral_error_2d,
                        'longitudinal_error': m.longitudinal_error_2d
                    },
                    '3d_errors_m': {
                        'endpoint_error': m.endpoint_error_3d,
                        'average_error': m.average_error_3d,
                        'lateral_error': m.lateral_error_3d,
                        'longitudinal_error': m.longitudinal_error_3d
                    }
                }
                for m in self.metrics_list
            ]
        }

        output_file = self.output_dir / 'validation_results.json'
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"Results saved to: {output_file}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='VLM Dataset Validator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Validate first 100 samples
  python core/vlm_dataset_validator.py --samples 100

  # Validate specific route
  python core/vlm_dataset_validator.py --route left_1 --samples 50

  # Use local VLM server
  python core/vlm_dataset_validator.py --vlm-preset local --samples 20

  # Validate all samples (will take a long time!)
  python core/vlm_dataset_validator.py
        """
    )

    default_dataset_dir = './datasets'
    parser.add_argument('--dataset-dir', type=str, default=str(default_dataset_dir),
                        help='Path to core/datasets directory (default: core/datasets)')

    parser.add_argument('--samples', type=int, default=None,
                        help='Number of samples to validate (default: all)')

    parser.add_argument('--route', type=str, default=None,
                        help='Filter by route name (e.g., left_1, mix_8)')

    parser.add_argument('--vlm-preset', type=str, default='cloud',
                        choices=['local', 'cloud'],
                        help='VLM endpoint preset (default: cloud)')

    parser.add_argument('--vlm-url', type=str, default=None,
                        help='VLM API endpoint URL (overrides preset)')

    parser.add_argument('--vlm-model', type=str, default=None,
                        help='VLM model name (overrides preset)')

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
            'url': 'http://100.85.159.60:8000/v1',
            'model': 'VishwanathAS/Qwen3-VLA-Driver-base'
        }
    }

    # Resolve VLM endpoint
    if args.vlm_url and args.vlm_model:
        vlm_url = args.vlm_url
        vlm_model = args.vlm_model
    elif args.vlm_preset in VLM_ENDPOINTS:
        vlm_url = VLM_ENDPOINTS[args.vlm_preset]['url']
        vlm_model = VLM_ENDPOINTS[args.vlm_preset]['model']
    else:
        vlm_url = VLM_ENDPOINTS['cloud']['url']
        vlm_model = VLM_ENDPOINTS['cloud']['model']

    print(f"VLM Endpoint: {vlm_url}")
    print(f"VLM Model: {vlm_model}")

    # Initialize validator
    validator = VLMDatasetValidator(
        vlm_base_url=vlm_url,
        vlm_model=vlm_model,
        output_dir=args.output_dir
    )

    # Run validation
    dataset_dir = Path(args.dataset_dir)
    validator.validate_dataset(
        dataset_dir=dataset_dir,
        num_samples=args.samples,
        route_filter=args.route
    )


if __name__ == '__main__':
    main()
