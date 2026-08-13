#!/usr/bin/env python3
"""
Unified VLM Validator

Validates VLM predictions using raw dataset JSON labels with actual vehicle transforms.
Supports both 2D (pixels) and 3D (meters) validation.

Usage:
    # 3D validation (recommended) - uses actual vehicle transforms from dataset
    python core/vlm_validator.py --mode 3d --dataset ./datasets --route straight_1 --samples 50

    # 2D pixel validation
    python core/vlm_validator.py --mode 2d --dataset ./datasets --samples 100

    # Single route, 50 samples
    python core/vlm_validator.py --mode 3d --dataset ./datasets --route straight_1 --samples 50
"""

import argparse
import json
import math
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

from vlm_inference import VLMTrajectoryPredictor, trajectory_to_waypoints_2d
from traj_planner import CameraConfig, TrajectoryDecoder, Waypoint2D, Waypoint3D

# Mock CARLA for offline use
class MockLocation:
    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = x, y, z

class MockRotation:
    def __init__(self, pitch=0, yaw=0, roll=0):
        self.pitch, self.yaw, self.roll = pitch, yaw, roll

class MockTransform:
    def __init__(self, location=None, rotation=None):
        self.location = location or MockLocation()
        self.rotation = rotation or MockRotation()

import sys
class MockCarla:
    Location = MockLocation
    Rotation = MockRotation
    Transform = MockTransform
sys.modules['carla'] = MockCarla()

CAMERA_CONFIG = CameraConfig(width=640, height=480, fov=90, x=2.0, y=0.0, z=1.8, pitch=-15, yaw=0, roll=0)


# =============================================================================
# METRICS
# =============================================================================

@dataclass
class ValidationMetrics:
    """Unified validation metrics."""
    sample_id: int
    route_name: str
    speed_kmh: float
    navigation: str

    # 2D errors (pixels)
    endpoint_error_2d: float
    avg_error_2d: float
    lateral_error_2d: float

    # 3D errors (meters) - only if mode=3d
    endpoint_error_3d: Optional[float] = None
    avg_error_3d: Optional[float] = None
    lateral_error_3d: Optional[float] = None

    # Accuracy thresholds
    accuracy_strict: bool = False  # 5px / 1m
    accuracy_moderate: bool = False  # 10px / 2m
    accuracy_relaxed: bool = False  # 20px / 5m

    # Precision
    precision_strict: float = 0.0
    precision_moderate: float = 0.0
    precision_relaxed: float = 0.0

    # Quality
    direction_error_deg: float = float('inf')

    # Performance
    inference_time: float = 0.0
    success: bool = False


def compute_errors(pred: List, gt: List, is_3d: bool = False) -> Dict:
    """Compute errors for 2D (pixel) or 3D (meter) trajectories."""

    if not pred or not gt:
        thresholds = (1.0, 2.0, 5.0) if is_3d else (5.0, 10.0, 20.0)
        return {
            'endpoint_error': float('inf'),
            'avg_error': float('inf'),
            'lateral_error': float('inf'),
            'accuracy_strict': False,
            'accuracy_moderate': False,
            'accuracy_relaxed': False,
            'precision_strict': 0.0,
            'precision_moderate': 0.0,
            'precision_relaxed': 0.0,
            'direction_error': float('inf'),
        }

    min_len = min(len(pred), len(gt))

    # Compute per-point errors
    errors = []
    lateral_errors = []

    for i in range(min_len):
        if is_3d:
            dx = pred[i].x - gt[i].x
            dy = pred[i].y - gt[i].y
        else:
            dx = pred[i][0] - gt[i][0]
            dy = pred[i][1] - gt[i][1]

        error = math.sqrt(dx * dx + dy * dy)
        errors.append(error)
        lateral_errors.append(abs(dx))

    # Distance metrics
    endpoint_error = errors[-1]
    avg_error = sum(errors) / len(errors)
    lateral_error = sum(lateral_errors) / len(lateral_errors)

    # Thresholds: 3D (1m, 2m, 5m) vs 2D (5px, 10px, 20px)
    t1, t2, t3 = (1.0, 2.0, 5.0) if is_3d else (5.0, 10.0, 20.0)

    # Accuracy (all points within threshold)
    accuracy_strict = all(e <= t1 for e in errors)
    accuracy_moderate = all(e <= t2 for e in errors)
    accuracy_relaxed = all(e <= t3 for e in errors)

    # Precision (per-point accuracy)
    precision_strict = sum(1 for e in errors if e <= t1) / len(errors) * 100
    precision_moderate = sum(1 for e in errors if e <= t2) / len(errors) * 100
    precision_relaxed = sum(1 for e in errors if e <= t3) / len(errors) * 100

    # Direction error
    if len(pred) >= 2 and len(gt) >= 2:
        if is_3d:
            pred_vec = [pred[-1].x - pred[0].x, pred[-1].y - pred[0].y]
            gt_vec = [gt[-1].x - gt[0].x, gt[-1].y - gt[0].y]
        else:
            pred_vec = [pred[-1][0] - pred[0][0], pred[-1][1] - pred[0][1]]
            gt_vec = [gt[-1][0] - gt[0][0], gt[-1][1] - gt[0][1]]

        pred_norm = math.sqrt(pred_vec[0]**2 + pred_vec[1]**2)
        gt_norm = math.sqrt(gt_vec[0]**2 + gt_vec[1]**2)

        if pred_norm > 1e-6 and gt_norm > 1e-6:
            dot = pred_vec[0] * gt_vec[0] + pred_vec[1] * gt_vec[1]
            cos_angle = max(-1.0, min(1.0, dot / (pred_norm * gt_norm)))
            direction_error = math.degrees(math.acos(cos_angle))
        else:
            direction_error = float('inf')
    else:
        direction_error = float('inf')

    return {
        'endpoint_error': endpoint_error,
        'avg_error': avg_error,
        'lateral_error': lateral_error,
        'accuracy_strict': accuracy_strict,
        'accuracy_moderate': accuracy_moderate,
        'accuracy_relaxed': accuracy_relaxed,
        'precision_strict': precision_strict,
        'precision_moderate': precision_moderate,
        'precision_relaxed': precision_relaxed,
        'direction_error': direction_error,
    }


# =============================================================================
# VALIDATOR
# =============================================================================

class VLMValidator:
    """Unified VLM validator using raw dataset JSON labels."""

    def __init__(self, vlm_url: str, vlm_model: str, mode: str = '3d', output_dir: Optional[str] = None):

        print(f"\n{'='*70}")
        print(f"VLM VALIDATOR ({mode.upper()} MODE)")
        print(f"{'='*70}")
        print(f"VLM: {vlm_url}")
        print(f"Model: {vlm_model}")

        self.mode = mode
        self.predictor = VLMTrajectoryPredictor(base_url=vlm_url, model=vlm_model)
        self.decoder = TrajectoryDecoder(CAMERA_CONFIG)

        # Output directory
        if not output_dir:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = f"./eval_results/validation_{mode}_{timestamp}"

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        print(f"Output: {self.output_dir}")
        print(f"{'='*70}\n")

        self.metrics: List[ValidationMetrics] = []

    def validate_sample(self, label_data: Dict, image: Image.Image) -> ValidationMetrics:
        """Validate a single sample from raw JSON label."""

        # Extract from label
        sample_id = label_data['id']
        route_name = label_data['vehicle_state']['route_name']
        speed = label_data['vehicle_state']['speed_kmh']
        navigation = label_data['vehicle_state']['navigation_command']

        # Ground truth
        gt_2d = label_data['trajectory_2d']
        gt_3d = label_data.get('trajectory_3d', [])

        # Vehicle state
        vehicle_loc = label_data['vehicle_state']['location']
        vehicle_rot = label_data['vehicle_state']['rotation']

        # Convert image to numpy
        image_np = np.array(image)

        # Run VLM
        pred_2d, inference_time = self.predictor.predict(image_np, current_speed=speed, navigation=navigation)
        success = bool(pred_2d)

        if not success:
            return ValidationMetrics(
                sample_id=sample_id, route_name=route_name, speed_kmh=speed, navigation=navigation,
                endpoint_error_2d=float('inf'), avg_error_2d=float('inf'), lateral_error_2d=float('inf'),
                inference_time=inference_time, success=False
            )

        # Extract x, y from prediction
        pred_2d = [[pt[0], pt[1]] for pt in pred_2d]

        # 2D validation
        errors_2d = compute_errors(pred_2d, gt_2d, is_3d=False)

        # 3D validation (if mode=3d)
        errors_3d = None
        if self.mode == '3d':
            # Create vehicle transform with actual vehicle state
            vehicle_transform = MockTransform(
                location=MockLocation(x=vehicle_loc[0], y=vehicle_loc[1], z=vehicle_loc[2]),
                rotation=MockRotation(pitch=vehicle_rot[0], yaw=vehicle_rot[1], roll=vehicle_rot[2])
            )

            # Convert BOTH predicted and ground truth 2D → 3D using decoder
            # This ensures apples-to-apples comparison
            pred_waypoints_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in pred_2d]
            pred_waypoints_3d = self.decoder.decode(pred_waypoints_2d, vehicle_transform, vehicle_loc[2])

            gt_waypoints_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in gt_2d]
            gt_waypoints_3d = self.decoder.decode(gt_waypoints_2d, vehicle_transform, vehicle_loc[2])

            errors_3d = compute_errors(pred_waypoints_3d, gt_waypoints_3d, is_3d=True)

        return ValidationMetrics(
            sample_id=sample_id,
            route_name=route_name,
            speed_kmh=speed,
            navigation=navigation,
            endpoint_error_2d=errors_2d['endpoint_error'],
            avg_error_2d=errors_2d['avg_error'],
            lateral_error_2d=errors_2d['lateral_error'],
            endpoint_error_3d=errors_3d['endpoint_error'] if errors_3d else None,
            avg_error_3d=errors_3d['avg_error'] if errors_3d else None,
            lateral_error_3d=errors_3d['lateral_error'] if errors_3d else None,
            accuracy_strict=errors_3d['accuracy_strict'] if errors_3d else errors_2d['accuracy_strict'],
            accuracy_moderate=errors_3d['accuracy_moderate'] if errors_3d else errors_2d['accuracy_moderate'],
            accuracy_relaxed=errors_3d['accuracy_relaxed'] if errors_3d else errors_2d['accuracy_relaxed'],
            precision_strict=errors_3d['precision_strict'] if errors_3d else errors_2d['precision_strict'],
            precision_moderate=errors_3d['precision_moderate'] if errors_3d else errors_2d['precision_moderate'],
            precision_relaxed=errors_3d['precision_relaxed'] if errors_3d else errors_2d['precision_relaxed'],
            direction_error_deg=errors_3d['direction_error'] if errors_3d else errors_2d['direction_error'],
            inference_time=inference_time,
            success=success
        )

    def validate_dataset(self, dataset_path: str, num_samples: Optional[int] = None,
                        route_filter: Optional[str] = None):
        """Validate on dataset using raw JSON labels."""

        dataset_dir = Path(dataset_path)
        labels_dir = dataset_dir / "labels"
        images_dir = dataset_dir / "images"

        if not labels_dir.exists() or not images_dir.exists():
            raise ValueError(f"Dataset directory must contain 'labels' and 'images' subdirectories")

        print(f"Dataset: {dataset_dir}")

        # Get all label files
        label_files = sorted(labels_dir.glob("*.json"))
        print(f"Found {len(label_files)} label files")

        # Filter and limit
        samples_to_process = []
        for label_file in label_files:
            with open(label_file) as f:
                label_data = json.load(f)

            # Filter by route
            if route_filter and label_data['vehicle_state']['route_name'] != route_filter:
                continue

            samples_to_process.append((label_file, label_data))

            # Limit samples
            if num_samples and len(samples_to_process) >= num_samples:
                break

        total = len(samples_to_process)
        if route_filter:
            print(f"Filtered to route: {route_filter}")
        print(f"Validating {total} samples...\n")

        # Validate
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
            self.metrics.append(metrics)

            # Progress
            if (i + 1) % 10 == 0 or i == total - 1:
                elapsed = time.time() - start_time
                eta = (elapsed / (i + 1)) * (total - i - 1)

                if self.mode == '3d' and metrics.avg_error_3d:
                    err = metrics.avg_error_3d if math.isfinite(metrics.avg_error_3d) else 999
                    unit = 'm'
                else:
                    err = metrics.avg_error_2d if math.isfinite(metrics.avg_error_2d) else 999
                    unit = 'px'

                print(f"[{i+1}/{total}] {metrics.route_name:15s} | "
                      f"2D: {metrics.avg_error_2d:.1f}px | 3D: {err:.2f}{unit} | ETA: {eta/60:.1f}min")

        elapsed = time.time() - start_time
        print(f"\n{'='*70}")
        print(f"Complete! Time: {elapsed/60:.1f}min")
        print(f"{'='*70}\n")

        self.print_summary()
        self.save_results()

    def print_summary(self):
        """Print summary."""
        successful = [m for m in self.metrics if m.success]

        print(f"\n{'='*70}")
        print(f"VALIDATION SUMMARY ({self.mode.upper()})")
        print(f"{'='*70}")
        print(f"Total: {len(self.metrics)} | Success: {len(successful)} ({len(successful)/len(self.metrics)*100:.1f}%)")

        if not successful:
            return

        # Choose metrics based on mode
        if self.mode == '3d':
            endpoint_errors = [m.endpoint_error_3d for m in successful if m.endpoint_error_3d]
            avg_errors = [m.avg_error_3d for m in successful if m.avg_error_3d]
            lateral_errors = [m.lateral_error_3d for m in successful if m.lateral_error_3d]
            unit = 'm'
            thresholds = ['<1m', '<2m', '<5m']
        else:
            endpoint_errors = [m.endpoint_error_2d for m in successful]
            avg_errors = [m.avg_error_2d for m in successful]
            lateral_errors = [m.lateral_error_2d for m in successful]
            unit = 'px'
            thresholds = ['<5px', '<10px', '<20px']

        direction_errors = [m.direction_error_deg for m in successful if math.isfinite(m.direction_error_deg)]

        print(f"\n{'─'*70}")
        print(f"DISTANCE ERRORS ({unit})")
        print(f"{'─'*70}")
        print(f"Endpoint:  Mean={np.mean(endpoint_errors):6.2f}{unit} | Median={np.median(endpoint_errors):6.2f}{unit}")
        print(f"Average:   Mean={np.mean(avg_errors):6.2f}{unit} | Median={np.median(avg_errors):6.2f}{unit}")
        print(f"Lateral:   Mean={np.mean(lateral_errors):6.2f}{unit}")

        print(f"\n{'─'*70}")
        print("ACCURACY & PRECISION")
        print(f"{'─'*70}")
        acc_strict = sum(m.accuracy_strict for m in successful) / len(successful) * 100
        acc_mod = sum(m.accuracy_moderate for m in successful) / len(successful) * 100
        acc_relax = sum(m.accuracy_relaxed for m in successful) / len(successful) * 100

        prec_strict = np.mean([m.precision_strict for m in successful])
        prec_mod = np.mean([m.precision_moderate for m in successful])
        prec_relax = np.mean([m.precision_relaxed for m in successful])

        print(f"Accuracy (all waypoints):  {thresholds[0]}: {acc_strict:5.1f}% | {thresholds[1]}: {acc_mod:5.1f}% | {thresholds[2]}: {acc_relax:5.1f}%")
        print(f"Precision (per-waypoint):  {thresholds[0]}: {prec_strict:5.1f}% | {thresholds[1]}: {prec_mod:5.1f}% | {thresholds[2]}: {prec_relax:5.1f}%")

        if direction_errors:
            print(f"\nDirection Error:  Mean={np.mean(direction_errors):6.2f}° | Median={np.median(direction_errors):6.2f}°")

        # Per-route
        routes: Dict[str, List] = {}
        for m in successful:
            routes.setdefault(m.route_name, []).append(m)

        if len(routes) > 1:
            print(f"\n{'─'*70}")
            print("PER-ROUTE")
            print(f"{'─'*70}")
            for name in sorted(routes.keys()):
                metrics_list = routes[name]
                if self.mode == '3d':
                    errs = [m.avg_error_3d for m in metrics_list if m.avg_error_3d]
                else:
                    errs = [m.avg_error_2d for m in metrics_list]
                print(f"{name:20s} | N={len(metrics_list):3d} | Avg={np.mean(errs):5.2f}{unit}")

        print(f"\n{'='*70}\n")

    def save_results(self):
        """Save results to JSON."""
        successful = [m for m in self.metrics if m.success]

        results = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'mode': self.mode,
                'total_samples': len(self.metrics),
                'successful': len(successful),
                'vlm_model': self.predictor.model,
            },
            'samples': [asdict(m) for m in self.metrics]
        }

        output_file = self.output_dir / 'results.json'
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"Results: {output_file}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Unified VLM Validator - Validates VLM predictions using raw dataset labels',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Validate 50 samples in 3D mode
  python core/vlm_validator.py --mode 3d --dataset ./datasets --samples 50

  # Validate single route with 50 samples
  python core/vlm_validator.py --mode 3d --dataset ./datasets --route straight_1 --samples 50

  # 2D validation with all samples
  python core/vlm_validator.py --mode 2d --dataset ./datasets

Dataset structure expected:
  datasets/
  ├── labels/
  │   ├── 000000.json
  │   ├── 000001.json
  │   └── ...
  └── images/
      ├── 000000.png
      ├── 000001.png
      └── ...
        """
    )

    parser.add_argument('--mode', type=str, default='3d', choices=['2d', '3d'],
                       help='Validation mode: 2d (pixels) or 3d (meters)')
    parser.add_argument('--dataset', type=str, required=True,
                       help='Dataset directory path (must contain labels/ and images/ subdirectories)')
    parser.add_argument('--route', type=str,
                       help='Filter to single route name')
    parser.add_argument('--samples', type=int,
                       help='Number of samples to validate (default: all)')
    parser.add_argument('--vlm-preset', type=str, default='cloud', choices=['local', 'cloud'])
    parser.add_argument('--vlm-url', type=str)
    parser.add_argument('--vlm-model', type=str)
    parser.add_argument('--output', type=str,
                       help='Output directory (default: auto-generated)')

    args = parser.parse_args()

    # VLM endpoints
    presets = {
        'local': {'url': 'http://localhost:1234/v1', 'model': 'local-model'},
        'cloud': {'url': 'https://337a6d421e56.woodpecker-bluegill.ts.net/v1', 'model': 'VishwanathAS/Qwen3-VLA-Driver-base'},
    }

    preset = presets[args.vlm_preset]
    vlm_url = args.vlm_url or preset['url']
    vlm_model = args.vlm_model or preset['model']

    # Create validator
    validator = VLMValidator(
        vlm_url=vlm_url,
        vlm_model=vlm_model,
        mode=args.mode,
        # routes_dir=None,  # Not needed with raw labels approach
        output_dir=args.output
    )

    # Validate
    validator.validate_dataset(
        dataset_path=args.dataset,
        num_samples=args.samples,
        route_filter=args.route
    )


if __name__ == '__main__':
    main()
