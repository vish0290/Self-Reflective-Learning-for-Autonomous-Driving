#!/usr/bin/env python3
"""
Offline Tester Module for VLM Trajectory Evaluation

Evaluates VLM trajectory predictions against ground truth from datasets.
Runs offline without CARLA simulation, directly comparing predicted trajectories
with ground truth pixel coordinates.

This module consolidates and unifies vlm_offline_eval.py and vlm_dataset_validator.py.

Usage:
    from offline_tester import OfflineTester

    tester = OfflineTester(vlm_base_url='http://localhost:8000/v1')

    # Evaluate from HuggingFace dataset
    tester.evaluate_hf_dataset('VishwanathAS/driving-dataset', num_samples=100)

    # Evaluate from local JSON labels
    tester.evaluate_local_dataset('./datasets', num_samples=100)

    # Print and save results
    tester.print_summary()
    tester.save_results()
"""

import json
import math
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image


# =============================================================================
# MOCK CARLA FOR OFFLINE USE
# =============================================================================


class MockCarlaLocation:
    """Mock CARLA location for offline evaluation."""

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x = x
        self.y = y
        self.z = z


class MockCarlaRotation:
    """Mock CARLA rotation for offline evaluation."""

    def __init__(self, pitch: float = 0, yaw: float = 0, roll: float = 0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class MockCarlaTransform:
    """Mock CARLA transform for offline evaluation."""

    def __init__(self, location=None, rotation=None):
        self.location = location or MockCarlaLocation()
        self.rotation = rotation or MockCarlaRotation()


# =============================================================================
# EVALUATION METRICS
# =============================================================================


@dataclass
class EvaluationMetrics:
    """Metrics for a single trajectory evaluation."""

    sample_id: str
    route_name: str
    speed: float
    navigation: str

    # Predictions
    predicted_trajectory: List[List[float]]
    ground_truth_trajectory: List[List[float]]
    inference_time: float

    # 2D Error metrics (pixels)
    endpoint_error_2d: float
    average_error_2d: float
    max_error_2d: float
    lateral_error_2d: float
    longitudinal_error_2d: float

    # 3D Error metrics (meters)
    endpoint_error_3d: float
    average_error_3d: float
    max_error_3d: float
    lateral_error_3d: float
    longitudinal_error_3d: float

    # Optional metadata
    vehicle_location: Optional[List[float]] = None
    vehicle_rotation: Optional[List[float]] = None


@dataclass
class EvaluationSummary:
    """Summary statistics for evaluation."""

    total_samples: int
    successful_predictions: int
    failed_predictions: int

    # 2D metrics (pixels)
    endpoint_error_2d_mean: float
    endpoint_error_2d_median: float
    endpoint_error_2d_std: float
    average_error_2d_mean: float
    average_error_2d_median: float
    lateral_error_2d_mean: float
    longitudinal_error_2d_mean: float

    # 3D metrics (meters)
    endpoint_error_3d_mean: float
    endpoint_error_3d_median: float
    endpoint_error_3d_std: float
    average_error_3d_mean: float
    average_error_3d_median: float
    lateral_error_3d_mean: float
    longitudinal_error_3d_mean: float

    # Timing
    inference_time_mean: float
    inference_time_median: float
    total_time: float


# =============================================================================
# ERROR COMPUTATION FUNCTIONS
# =============================================================================


def compute_endpoint_error(
    pred: List[List[float]], gt: List[List[float]]
) -> float:
    """Compute L2 distance to final waypoint."""
    if not pred or not gt:
        return float("inf")

    pred_end = pred[-1]
    gt_end = gt[-1]

    dx = pred_end[0] - gt_end[0]
    dy = pred_end[1] - gt_end[1]

    return math.sqrt(dx * dx + dy * dy)


def compute_average_error(
    pred: List[List[float]], gt: List[List[float]]
) -> float:
    """Compute average L2 distance between predicted and ground truth waypoints."""
    if not pred or not gt:
        return float("inf")

    min_len = min(len(pred), len(gt))

    errors = []
    for i in range(min_len):
        dx = pred[i][0] - gt[i][0]
        dy = pred[i][1] - gt[i][1]
        errors.append(math.sqrt(dx * dx + dy * dy))

    return sum(errors) / len(errors) if errors else float("inf")


def compute_max_error(pred: List[List[float]], gt: List[List[float]]) -> float:
    """Compute maximum L2 distance error."""
    if not pred or not gt:
        return float("inf")

    min_len = min(len(pred), len(gt))

    max_err = 0.0
    for i in range(min_len):
        dx = pred[i][0] - gt[i][0]
        dy = pred[i][1] - gt[i][1]
        max_err = max(max_err, math.sqrt(dx * dx + dy * dy))

    return max_err


def compute_lateral_error(
    pred: List[List[float]], gt: List[List[float]]
) -> float:
    """Compute average lateral (x-axis) error."""
    if not pred or not gt:
        return float("inf")

    min_len = min(len(pred), len(gt))
    errors = [abs(pred[i][0] - gt[i][0]) for i in range(min_len)]

    return sum(errors) / len(errors) if errors else float("inf")


def compute_longitudinal_error(
    pred: List[List[float]], gt: List[List[float]]
) -> float:
    """Compute average longitudinal (y-axis) error."""
    if not pred or not gt:
        return float("inf")

    min_len = min(len(pred), len(gt))
    errors = [abs(pred[i][1] - gt[i][1]) for i in range(min_len)]

    return sum(errors) / len(errors) if errors else float("inf")


# =============================================================================
# OFFLINE TESTER
# =============================================================================


class OfflineTester:
    """
    Evaluates VLM predictions against ground truth datasets.

    Supports both HuggingFace datasets and local JSON label files.
    """

    # VLM endpoint presets
    VLM_PRESETS = {
        "local": {
            "url": "http://localhost:1234/v1",
            "model": "local-model",
        },
        "cloud": {
            "url": "http://100.85.159.60:8000/v1",
            "model": "VishwanathAS/Qwen3-VLA-Driver-base",
        },
    }

    def __init__(
        self,
        vlm_base_url: Optional[str] = None,
        vlm_model: Optional[str] = None,
        vlm_preset: str = "cloud",
        output_dir: Optional[str] = None,
    ):
        """
        Initialize offline tester.

        Args:
            vlm_base_url: VLM API endpoint (overrides preset)
            vlm_model: VLM model name (overrides preset)
            vlm_preset: Preset to use if url/model not provided
            output_dir: Directory to save evaluation results
        """
        # Resolve VLM endpoint
        preset = self.VLM_PRESETS.get(vlm_preset, self.VLM_PRESETS["cloud"])
        self.vlm_base_url = vlm_base_url or preset["url"]
        self.vlm_model = vlm_model or preset["model"]

        # Initialize VLM predictor
        from vlm_inference import VLMTrajectoryPredictor

        self.predictor = VLMTrajectoryPredictor(
            base_url=self.vlm_base_url, model=self.vlm_model
        )

        # Initialize trajectory decoder for 3D conversion
        from traj_planner import CameraConfig, TrajectoryDecoder

        self.camera_config = CameraConfig(
            width=640,
            height=480,
            fov=90,
            x=2.0,
            y=0.0,
            z=1.8,
            pitch=-15,
            yaw=0,
            roll=0,
        )
        self.decoder = TrajectoryDecoder(self.camera_config)

        # Output directory
        if output_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = f"./eval_results/offline_eval_{timestamp}"

        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Metrics storage
        self.metrics_list: List[EvaluationMetrics] = []
        self.start_time: Optional[float] = None

    def evaluate_sample(
        self,
        image: np.ndarray,
        ground_truth_trajectory: List[List[float]],
        speed_kmh: float = 30.0,
        navigation: str = "lane_keeping",
        route_name: str = "unknown",
        sample_id: str = "0",
        vehicle_location: Optional[List[float]] = None,
        vehicle_rotation: Optional[List[float]] = None,
    ) -> EvaluationMetrics:
        """
        Evaluate a single sample.

        Args:
            image: RGB image as numpy array
            ground_truth_trajectory: Ground truth [[x, y], ...] or [[x, y, d], ...]
            speed_kmh: Current speed in km/h
            navigation: Navigation command
            route_name: Route identifier
            sample_id: Sample identifier
            vehicle_location: Optional vehicle location [x, y, z]
            vehicle_rotation: Optional vehicle rotation [pitch, yaw, roll]

        Returns:
            EvaluationMetrics for this sample
        """
        # Extract only x, y from ground truth
        if ground_truth_trajectory and len(ground_truth_trajectory[0]) >= 3:
            gt_2d = [[pt[0], pt[1]] for pt in ground_truth_trajectory]
        else:
            gt_2d = ground_truth_trajectory

        # Run VLM prediction
        pred_trajectory_raw, inference_time = self.predictor.predict(
            image, current_speed=speed_kmh, navigation=navigation
        )

        # Extract x, y from prediction
        pred_2d = (
            [[pt[0], pt[1]] for pt in pred_trajectory_raw]
            if pred_trajectory_raw
            else []
        )

        # Compute 2D errors (pixels)
        endpoint_err_2d = compute_endpoint_error(pred_2d, gt_2d)
        avg_err_2d = compute_average_error(pred_2d, gt_2d)
        max_err_2d = compute_max_error(pred_2d, gt_2d)
        lateral_err_2d = compute_lateral_error(pred_2d, gt_2d)
        longitudinal_err_2d = compute_longitudinal_error(pred_2d, gt_2d)

        # Convert to 3D for world-space comparison
        pred_3d_list, gt_3d_list = self._convert_to_3d(
            pred_2d, gt_2d, vehicle_location, vehicle_rotation
        )

        # Compute 3D errors (meters)
        endpoint_err_3d = compute_endpoint_error(pred_3d_list, gt_3d_list)
        avg_err_3d = compute_average_error(pred_3d_list, gt_3d_list)
        max_err_3d = compute_max_error(pred_3d_list, gt_3d_list)
        lateral_err_3d = compute_lateral_error(pred_3d_list, gt_3d_list)
        longitudinal_err_3d = compute_longitudinal_error(pred_3d_list, gt_3d_list)

        return EvaluationMetrics(
            sample_id=sample_id,
            route_name=route_name,
            speed=speed_kmh,
            navigation=navigation,
            predicted_trajectory=pred_2d,
            ground_truth_trajectory=gt_2d,
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
            longitudinal_error_3d=longitudinal_err_3d,
            vehicle_location=vehicle_location,
            vehicle_rotation=vehicle_rotation,
        )

    def _convert_to_3d(
        self,
        pred_2d: List[List[float]],
        gt_2d: List[List[float]],
        vehicle_location: Optional[List[float]],
        vehicle_rotation: Optional[List[float]],
    ) -> Tuple[List[List[float]], List[List[float]]]:
        """Convert 2D trajectories to 3D using decoder."""
        from traj_planner import Waypoint2D

        # Create vehicle transform
        if vehicle_location and vehicle_rotation:
            vehicle_transform = MockCarlaTransform(
                MockCarlaLocation(*vehicle_location),
                MockCarlaRotation(*vehicle_rotation),
            )
            road_height = vehicle_location[2]
        else:
            vehicle_transform = MockCarlaTransform(
                MockCarlaLocation(0, 0, 1.0), MockCarlaRotation(0, 0, 0)
            )
            road_height = 0.5

        # Convert predicted 2D to 3D
        pred_wp_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in pred_2d] if pred_2d else []
        pred_wp_3d = (
            self.decoder.decode(pred_wp_2d, vehicle_transform, road_height)
            if pred_wp_2d
            else []
        )
        pred_3d_list = [[wp.x, wp.y] for wp in pred_wp_3d]

        # Convert ground truth 2D to 3D
        gt_wp_2d = [Waypoint2D(u=pt[0], v=pt[1]) for pt in gt_2d]
        gt_wp_3d = self.decoder.decode(gt_wp_2d, vehicle_transform, road_height)
        gt_3d_list = [[wp.x, wp.y] for wp in gt_wp_3d]

        return pred_3d_list, gt_3d_list

    def evaluate_hf_dataset(
        self,
        dataset_name: str,
        split: str = "train",
        num_samples: Optional[int] = None,
        route_filter: Optional[str] = None,
    ) -> None:
        """
        Evaluate VLM on HuggingFace dataset.

        Args:
            dataset_name: HuggingFace dataset identifier or local path
            split: Dataset split
            num_samples: Number of samples to evaluate
            route_filter: Only evaluate samples from this route
        """
        try:
            from datasets import load_dataset, load_from_disk
        except ImportError:
            raise ImportError("datasets package not installed. Run: pip install datasets")

        self._print_header(f"HuggingFace Dataset: {dataset_name}")

        # Load dataset
        print("Loading dataset...")
        dataset_path = Path(dataset_name)
        if dataset_path.exists() and dataset_path.is_dir():
            print(f"Loading from local directory: {dataset_path}")
            dataset = load_from_disk(str(dataset_path))
        else:
            print(f"Loading from HuggingFace: {dataset_name}")
            dataset = load_dataset(dataset_name, split=split)

        # Filter and limit
        if route_filter:
            dataset = dataset.filter(lambda x: x.get("route_name") == route_filter)
            print(f"Filtered to route: {route_filter}")

        if num_samples:
            dataset = dataset.select(range(min(num_samples, len(dataset))))

        self._run_evaluation(dataset, self._process_hf_sample)

    def evaluate_local_dataset(
        self,
        dataset_dir: str,
        num_samples: Optional[int] = None,
        route_filter: Optional[str] = None,
    ) -> None:
        """
        Evaluate VLM on local dataset with JSON labels.

        Args:
            dataset_dir: Path to dataset directory (with labels/ and images/)
            num_samples: Number of samples to evaluate
            route_filter: Only evaluate samples from this route
        """
        dataset_path = Path(dataset_dir)
        labels_dir = dataset_path / "labels"
        images_dir = dataset_path / "images"

        self._print_header(f"Local Dataset: {dataset_dir}")

        if not labels_dir.exists():
            raise FileNotFoundError(f"Labels directory not found: {labels_dir}")

        # Load samples
        label_files = sorted(labels_dir.glob("*.json"))
        samples = []

        for label_file in label_files:
            with open(label_file) as f:
                label_data = json.load(f)

            # Apply route filter
            if route_filter:
                route_name = label_data.get("vehicle_state", {}).get("route_name", "")
                if route_name != route_filter:
                    continue

            samples.append((label_file, label_data, images_dir))

            if num_samples and len(samples) >= num_samples:
                break

        print(f"Found {len(samples)} samples")
        if route_filter:
            print(f"Route filter: {route_filter}")

        self._run_evaluation(samples, self._process_local_sample)

    def _process_hf_sample(self, sample: Dict) -> EvaluationMetrics:
        """Process a HuggingFace dataset sample."""
        import ast

        # Extract data
        image_pil = sample["image"]
        gt_trajectory_raw = sample["trajectory"]
        speed = sample.get("speed_kmh", sample.get("speed", 30.0))
        navigation = sample.get(
            "navigation_command", sample.get("navigation", "lane_keeping")
        )
        route_name = sample.get("route_name", "unknown")

        # Convert PIL image to numpy
        image_np = np.array(image_pil)

        # Parse trajectory if string
        if isinstance(gt_trajectory_raw, str):
            gt_trajectory = ast.literal_eval(gt_trajectory_raw)
        else:
            gt_trajectory = gt_trajectory_raw

        return self.evaluate_sample(
            image=image_np,
            ground_truth_trajectory=gt_trajectory,
            speed_kmh=speed,
            navigation=navigation,
            route_name=route_name,
            sample_id=str(len(self.metrics_list)),
        )

    def _process_local_sample(
        self, sample: Tuple[Path, Dict, Path]
    ) -> EvaluationMetrics:
        """Process a local dataset sample."""
        label_file, label_data, images_dir = sample

        # Extract data
        sample_id = label_data["id"]
        vehicle_state = label_data.get("vehicle_state", {})
        speed_kmh = vehicle_state.get("speed_kmh", 30.0)
        navigation = vehicle_state.get("navigation_command", "lane_keeping")
        route_name = vehicle_state.get("route_name", "unknown")

        # Ground truth
        gt_2d = label_data.get("trajectory_2d", label_data.get("trajectory", []))

        # Vehicle transform
        vehicle_location = vehicle_state.get("location")
        vehicle_rotation = vehicle_state.get("rotation")

        # Load image
        image_filename = label_data.get("image_file", f"{sample_id}.jpg")
        image_path = images_dir / image_filename

        if not image_path.exists():
            # Try other extensions
            for ext in [".jpg", ".png", ".npz"]:
                alt_path = images_dir / f"{sample_id}{ext}"
                if alt_path.exists():
                    image_path = alt_path
                    break

        if image_path.suffix == ".npz":
            data = np.load(image_path)
            image_np = data["image"]
        else:
            image_np = np.array(Image.open(image_path))

        return self.evaluate_sample(
            image=image_np,
            ground_truth_trajectory=gt_2d,
            speed_kmh=speed_kmh,
            navigation=navigation,
            route_name=route_name,
            sample_id=sample_id,
            vehicle_location=vehicle_location,
            vehicle_rotation=vehicle_rotation,
        )

    def _run_evaluation(self, samples, process_fn) -> None:
        """Run evaluation on samples."""
        self.start_time = time.time()
        total_samples = len(samples)

        print(f"Evaluating {total_samples} samples...\n")

        for i, sample in enumerate(samples):
            metrics = process_fn(sample)
            self.metrics_list.append(metrics)

            # Progress update
            if (i + 1) % 10 == 0 or i == total_samples - 1:
                elapsed = time.time() - self.start_time
                avg_time_per_sample = elapsed / (i + 1)
                eta = avg_time_per_sample * (total_samples - i - 1)

                print(
                    f"Progress: {i + 1}/{total_samples} | "
                    f"2D: {metrics.average_error_2d:.1f}px | "
                    f"3D: {metrics.average_error_3d:.2f}m | "
                    f"ETA: {eta / 60:.1f}min"
                )

        elapsed = time.time() - self.start_time
        print(f"\n{'=' * 70}")
        print(f"Evaluation complete! Time: {elapsed / 60:.1f} minutes")
        print(f"{'=' * 70}\n")

    def _print_header(self, dataset_name: str) -> None:
        """Print evaluation header."""
        print(f"\n{'=' * 70}")
        print("VLM OFFLINE EVALUATION")
        print(f"{'=' * 70}")
        print(f"Dataset: {dataset_name}")
        print(f"VLM: {self.vlm_base_url}")
        print(f"Model: {self.vlm_model}")
        print(f"Output: {self.output_dir}")
        print(f"{'=' * 70}\n")

    def get_summary(self) -> EvaluationSummary:
        """Compute summary statistics."""
        if not self.metrics_list:
            raise ValueError("No evaluation results available")

        # Filter valid metrics
        def get_valid(attr):
            return [
                getattr(m, attr) for m in self.metrics_list if math.isfinite(getattr(m, attr))
            ]

        endpoint_errors_2d = get_valid("endpoint_error_2d")
        avg_errors_2d = get_valid("average_error_2d")
        lateral_errors_2d = get_valid("lateral_error_2d")
        longitudinal_errors_2d = get_valid("longitudinal_error_2d")

        endpoint_errors_3d = get_valid("endpoint_error_3d")
        avg_errors_3d = get_valid("average_error_3d")
        lateral_errors_3d = get_valid("lateral_error_3d")
        longitudinal_errors_3d = get_valid("longitudinal_error_3d")

        inference_times = [m.inference_time for m in self.metrics_list]

        failed = sum(1 for m in self.metrics_list if not m.predicted_trajectory)

        return EvaluationSummary(
            total_samples=len(self.metrics_list),
            successful_predictions=len(self.metrics_list) - failed,
            failed_predictions=failed,
            endpoint_error_2d_mean=float(np.mean(endpoint_errors_2d)),
            endpoint_error_2d_median=float(np.median(endpoint_errors_2d)),
            endpoint_error_2d_std=float(np.std(endpoint_errors_2d)),
            average_error_2d_mean=float(np.mean(avg_errors_2d)),
            average_error_2d_median=float(np.median(avg_errors_2d)),
            lateral_error_2d_mean=float(np.mean(lateral_errors_2d)),
            longitudinal_error_2d_mean=float(np.mean(longitudinal_errors_2d)),
            endpoint_error_3d_mean=float(np.mean(endpoint_errors_3d)),
            endpoint_error_3d_median=float(np.median(endpoint_errors_3d)),
            endpoint_error_3d_std=float(np.std(endpoint_errors_3d)),
            average_error_3d_mean=float(np.mean(avg_errors_3d)),
            average_error_3d_median=float(np.median(avg_errors_3d)),
            lateral_error_3d_mean=float(np.mean(lateral_errors_3d)),
            longitudinal_error_3d_mean=float(np.mean(longitudinal_errors_3d)),
            inference_time_mean=float(np.mean(inference_times)),
            inference_time_median=float(np.median(inference_times)),
            total_time=time.time() - self.start_time if self.start_time else 0.0,
        )

    def print_summary(self) -> None:
        """Print evaluation summary."""
        if not self.metrics_list:
            print("No evaluation results to summarize.")
            return

        summary = self.get_summary()

        print("EVALUATION SUMMARY")
        print("=" * 70)
        print(f"Total Samples: {summary.total_samples}")
        print(f"Successful: {summary.successful_predictions}")
        print(f"Failed: {summary.failed_predictions}")

        print(f"\n{'-' * 70}")
        print("2D ERRORS (Pixel Space)")
        print(f"{'-' * 70}")
        print(f"Endpoint Error:")
        print(
            f"  Mean: {summary.endpoint_error_2d_mean:.2f} px | "
            f"Median: {summary.endpoint_error_2d_median:.2f} px | "
            f"Std: {summary.endpoint_error_2d_std:.2f} px"
        )
        print(f"Average Trajectory Error:")
        print(
            f"  Mean: {summary.average_error_2d_mean:.2f} px | "
            f"Median: {summary.average_error_2d_median:.2f} px"
        )
        print(f"Lateral Error: {summary.lateral_error_2d_mean:.2f} px")
        print(f"Longitudinal Error: {summary.longitudinal_error_2d_mean:.2f} px")

        print(f"\n{'-' * 70}")
        print("3D ERRORS (World Space - Meters)")
        print(f"{'-' * 70}")
        print(f"Endpoint Error:")
        print(
            f"  Mean: {summary.endpoint_error_3d_mean:.2f} m | "
            f"Median: {summary.endpoint_error_3d_median:.2f} m | "
            f"Std: {summary.endpoint_error_3d_std:.2f} m"
        )
        print(f"Average Trajectory Error:")
        print(
            f"  Mean: {summary.average_error_3d_mean:.2f} m | "
            f"Median: {summary.average_error_3d_median:.2f} m"
        )
        print(f"Lateral Error: {summary.lateral_error_3d_mean:.2f} m")
        print(f"Longitudinal Error: {summary.longitudinal_error_3d_mean:.2f} m")

        print(f"\n{'-' * 70}")
        print("Inference Time:")
        print(
            f"  Mean: {summary.inference_time_mean:.3f}s | "
            f"Median: {summary.inference_time_median:.3f}s"
        )
        print("=" * 70)

        # Per-route breakdown
        routes = {}
        for m in self.metrics_list:
            if m.route_name not in routes:
                routes[m.route_name] = []
            routes[m.route_name].append(m)

        if len(routes) > 1:
            print("\nPer-Route Breakdown:")
            print("-" * 70)
            for route_name, route_metrics in sorted(routes.items()):
                route_avg_2d = [
                    m.average_error_2d
                    for m in route_metrics
                    if math.isfinite(m.average_error_2d)
                ]
                route_avg_3d = [
                    m.average_error_3d
                    for m in route_metrics
                    if math.isfinite(m.average_error_3d)
                ]
                print(
                    f"{route_name}: {len(route_metrics)} samples | "
                    f"2D: {np.mean(route_avg_2d):.1f}px | "
                    f"3D: {np.mean(route_avg_3d):.2f}m"
                )

    def save_results(self, filename: str = "evaluation_results.json") -> Path:
        """Save evaluation results to JSON."""
        summary = self.get_summary()

        results = {
            "metadata": {
                "timestamp": datetime.now().isoformat(),
                "total_samples": summary.total_samples,
                "vlm_model": self.vlm_model,
                "vlm_url": self.vlm_base_url,
            },
            "summary": {
                "2d_errors_px": {
                    "endpoint_error_mean": summary.endpoint_error_2d_mean,
                    "endpoint_error_median": summary.endpoint_error_2d_median,
                    "average_error_mean": summary.average_error_2d_mean,
                    "lateral_error_mean": summary.lateral_error_2d_mean,
                    "longitudinal_error_mean": summary.longitudinal_error_2d_mean,
                },
                "3d_errors_m": {
                    "endpoint_error_mean": summary.endpoint_error_3d_mean,
                    "endpoint_error_median": summary.endpoint_error_3d_median,
                    "average_error_mean": summary.average_error_3d_mean,
                    "lateral_error_mean": summary.lateral_error_3d_mean,
                    "longitudinal_error_mean": summary.longitudinal_error_3d_mean,
                },
                "inference_time_mean": summary.inference_time_mean,
            },
            "samples": [
                {
                    "sample_id": m.sample_id,
                    "route_name": m.route_name,
                    "speed": m.speed,
                    "navigation": m.navigation,
                    "inference_time": m.inference_time,
                    "2d_errors_px": {
                        "endpoint_error": m.endpoint_error_2d,
                        "average_error": m.average_error_2d,
                        "max_error": m.max_error_2d,
                        "lateral_error": m.lateral_error_2d,
                        "longitudinal_error": m.longitudinal_error_2d,
                    },
                    "3d_errors_m": {
                        "endpoint_error": m.endpoint_error_3d,
                        "average_error": m.average_error_3d,
                        "max_error": m.max_error_3d,
                        "lateral_error": m.lateral_error_3d,
                        "longitudinal_error": m.longitudinal_error_3d,
                    },
                }
                for m in self.metrics_list
            ],
        }

        output_file = self.output_dir / filename
        with open(output_file, "w") as f:
            json.dump(results, f, indent=2)

        print(f"Results saved to: {output_file}")
        return output_file


# =============================================================================
# CLI
# =============================================================================


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="VLM Offline Evaluation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Evaluate from HuggingFace dataset
  python offline_tester.py --hf-dataset VishwanathAS/driving-dataset --samples 100

  # Evaluate from local dataset
  python offline_tester.py --local-dataset ./datasets --samples 100

  # Use local VLM server
  python offline_tester.py --hf-dataset ./hf_trajectory_dataset --vlm-preset local

  # Filter by route
  python offline_tester.py --local-dataset ./datasets --route left_1 --samples 50
        """,
    )

    # Dataset source (mutually exclusive)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--hf-dataset",
        type=str,
        help="HuggingFace dataset identifier or local path",
    )
    group.add_argument(
        "--local-dataset",
        type=str,
        help="Local dataset directory (with labels/ and images/)",
    )

    parser.add_argument(
        "--split", type=str, default="train", help="Dataset split (default: train)"
    )
    parser.add_argument(
        "--samples", type=int, default=None, help="Number of samples to evaluate"
    )
    parser.add_argument(
        "--route", type=str, default=None, help="Filter by route name"
    )
    parser.add_argument(
        "--vlm-preset",
        type=str,
        default="cloud",
        choices=["local", "cloud"],
        help="VLM endpoint preset (default: cloud)",
    )
    parser.add_argument(
        "--vlm-url", type=str, default=None, help="Custom VLM API endpoint URL"
    )
    parser.add_argument(
        "--vlm-model", type=str, default=None, help="Custom VLM model name"
    )
    parser.add_argument(
        "--output-dir", type=str, default=None, help="Output directory for results"
    )

    args = parser.parse_args()

    # Create tester
    tester = OfflineTester(
        vlm_base_url=args.vlm_url,
        vlm_model=args.vlm_model,
        vlm_preset=args.vlm_preset,
        output_dir=args.output_dir,
    )

    # Run evaluation
    if args.hf_dataset:
        tester.evaluate_hf_dataset(
            dataset_name=args.hf_dataset,
            split=args.split,
            num_samples=args.samples,
            route_filter=args.route,
        )
    else:
        tester.evaluate_local_dataset(
            dataset_dir=args.local_dataset,
            num_samples=args.samples,
            route_filter=args.route,
        )

    # Print and save results
    tester.print_summary()
    tester.save_results()


if __name__ == "__main__":
    main()
