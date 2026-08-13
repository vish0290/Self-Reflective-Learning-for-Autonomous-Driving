#!/usr/bin/env python3
"""
Dataset Recorder Module for Autonomous Driving Data Collection

Unified module for recording driving datasets, including:
- Async data collection with background saving
- Trajectory and navigation token recording
- HuggingFace dataset formatting and upload
- Post-processing utilities (NPZ to JPEG conversion, train/val split)

This module consolidates data_collector.py, record_dataset.py, and create_hf_dataset.py.

Usage:
    from dataset_recorder import DatasetRecorder

    # Create recorder
    recorder = DatasetRecorder(output_dir='./datasets/my_dataset')

    # In main loop
    recorder.record_sample(
        rgb_image=sensor_data.rgb_image,
        waypoints_2d=waypoints_2d,
        waypoints_3d=waypoints_3d,
        vehicle_state=vehicle_state,
        control=control,
        route_name='highway_1'
    )

    # After recording
    recorder.stop()
    recorder.convert_to_jpeg()
    recorder.create_hf_dataset(push_to_hub='username/dataset')
"""

import json
import queue
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class RecorderConfig:
    """Configuration for dataset recorder."""

    output_dir: str
    max_queue_size: int = 500
    save_interval: int = 1
    min_speed_kmh: float = 15.0
    min_waypoints: int = 5
    jpeg_quality: int = 95

    # Camera config (for metadata)
    camera_width: int = 640
    camera_height: int = 480
    camera_fov: float = 90.0
    camera_position: List[float] = None
    camera_rotation: List[float] = None

    # Waypoint distances
    waypoint_distances: List[float] = None

    def __post_init__(self):
        if self.camera_position is None:
            self.camera_position = [2.0, 0.0, 1.8]
        if self.camera_rotation is None:
            self.camera_rotation = [-15.0, 0.0, 0.0]
        if self.waypoint_distances is None:
            self.waypoint_distances = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]


class DatasetRecorder:
    """
    Async data collection for VLM training datasets.

    Features:
    - Non-blocking background saving
    - NPZ format for fast recording
    - JPEG conversion post-processing
    - HuggingFace dataset creation
    """

    def __init__(self, config: Optional[RecorderConfig] = None, output_dir: Optional[str] = None):
        """
        Initialize dataset recorder.

        Args:
            config: RecorderConfig object (preferred)
            output_dir: Output directory (creates default config if provided alone)
        """
        if config is None:
            if output_dir is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = f"./datasets/dataset_{timestamp}"
            config = RecorderConfig(output_dir=output_dir)

        self.config = config
        self.output_dir = Path(config.output_dir)

        # Create directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "raw").mkdir(exist_ok=True)
        (self.output_dir / "images").mkdir(exist_ok=True)
        (self.output_dir / "labels").mkdir(exist_ok=True)

        # Queue and threading
        self.data_queue = queue.Queue(maxsize=config.max_queue_size)
        self.worker_thread: Optional[threading.Thread] = None
        self.running = False

        # Statistics
        self.stats = {
            "queued": 0,
            "saved": 0,
            "dropped": 0,
            "skipped_speed": 0,
            "skipped_waypoints": 0,
            "start_time": None,
            "save_times": [],
        }

        # Counters
        self.sample_count = 0
        self.queue_count = 0

        # Route tracking
        self.route_usage: Dict[str, int] = {}

    def start(self) -> None:
        """Start the background saving thread."""
        if self.running:
            return

        self.running = True
        self.stats["start_time"] = time.time()
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()
        print(f"[DatasetRecorder] Started. Saving to: {self.output_dir}")

    def stop(self) -> None:
        """Stop the recorder and wait for queue to drain."""
        if not self.running:
            return

        print(f"[DatasetRecorder] Stopping... ({self.data_queue.qsize()} items in queue)")
        self.running = False

        # Wait for queue to drain
        timeout = 30
        start = time.time()
        while not self.data_queue.empty() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if self.worker_thread:
            self.worker_thread.join(timeout=5)

        # Save manifest
        self._save_manifest()

        print(f"[DatasetRecorder] Stopped. Stats: {self.get_stats()}")

    def record_sample(
        self,
        rgb_image: np.ndarray,
        waypoints_2d: List,
        waypoints_3d: List,
        vehicle_state: Dict,
        control: Dict,
        route_name: str = "unknown",
        kept_indices: Optional[List[int]] = None,
        navigation_command: Optional[str] = None,
        decision_reasoning: Optional[str] = None,
        timestamp: Optional[float] = None,
    ) -> bool:
        """
        Record a single sample.

        Args:
            rgb_image: RGB camera image as numpy array
            waypoints_2d: 2D waypoints (Waypoint2D objects or [[u, v], ...])
            waypoints_3d: 3D waypoints (Waypoint3D objects or [[x, y, z], ...])
            vehicle_state: Vehicle state dict with speed_kmh, location, rotation
            control: Control dict with throttle, brake, steer
            route_name: Current route name
            kept_indices: Indices of original waypoints that were kept
            navigation_command: Navigation token (auto-generated if not provided)
            decision_reasoning: Reasoning text for decision
            timestamp: Sample timestamp (auto-generated if not provided)

        Returns:
            True if sample was queued, False if skipped or dropped
        """
        self.queue_count += 1

        # Apply save interval
        if self.queue_count % self.config.save_interval != 0:
            return True

        # Check speed threshold
        speed_kmh = vehicle_state.get("speed_kmh", 0)
        if speed_kmh < self.config.min_speed_kmh:
            self.stats["skipped_speed"] += 1
            return False

        # Check waypoint count
        if len(waypoints_2d) < self.config.min_waypoints:
            self.stats["skipped_waypoints"] += 1
            return False

        # Convert waypoints to list format
        wp_2d_list = self._to_list(waypoints_2d, ["u", "v"])
        wp_3d_list = self._to_list(waypoints_3d, ["x", "y", "z"])

        # Generate navigation command if not provided
        if navigation_command is None:
            navigation_command = self._generate_navigation_token(wp_3d_list)

        # Prepare sample data
        sample = {
            "image": rgb_image.copy(),
            "waypoints_2d": wp_2d_list,
            "waypoints_3d": wp_3d_list,
            "kept_indices": kept_indices or list(range(len(wp_2d_list))),
            "vehicle_state": vehicle_state,
            "control": control,
            "route_name": route_name,
            "navigation_command": navigation_command,
            "decision_reasoning": decision_reasoning or "",
            "timestamp": timestamp or time.time(),
        }

        try:
            self.data_queue.put_nowait(sample)
            self.stats["queued"] += 1
            return True
        except queue.Full:
            self.stats["dropped"] += 1
            return False

    @staticmethod
    def _to_list(waypoints: List, attrs: List[str]) -> List[List[float]]:
        """Convert waypoint objects to list format."""
        result = []
        for wp in waypoints:
            if hasattr(wp, attrs[0]):
                # Object with attributes
                result.append([getattr(wp, attr) for attr in attrs])
            elif isinstance(wp, (list, tuple)):
                # Already a list
                result.append(list(wp[: len(attrs)]))
            else:
                raise ValueError(f"Unknown waypoint format: {type(wp)}")
        return result

    def _generate_navigation_token(self, waypoints_3d: List[List[float]]) -> str:
        """Generate navigation token from 3D waypoints."""
        try:
            from navigation_analyzer import generate_navigation_token

            return generate_navigation_token(waypoints_3d, use_map_api=False)
        except ImportError:
            return "lane_keeping"

    def _worker_loop(self) -> None:
        """Background worker that saves data from queue."""
        while self.running or not self.data_queue.empty():
            try:
                sample = self.data_queue.get(timeout=0.5)
                self._save_sample(sample)
                self.data_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[DatasetRecorder] Error saving: {e}")

    def _save_sample(self, sample: Dict) -> None:
        """Save a single sample to disk."""
        start_time = time.time()

        sample_id = f"{self.sample_count:06d}"

        # Build trajectory with distances (hybrid format for VLM)
        trajectory_hybrid = []
        for i, (u, v) in enumerate(sample["waypoints_2d"]):
            if sample["kept_indices"] and i < len(sample["kept_indices"]):
                orig_idx = sample["kept_indices"][i]
                dist = (
                    self.config.waypoint_distances[orig_idx]
                    if orig_idx < len(self.config.waypoint_distances)
                    else 0
                )
            else:
                dist = 0
            trajectory_hybrid.append([int(u), int(v), dist])

        # Save image as NPZ (fast)
        npz_path = self.output_dir / "raw" / f"{sample_id}.npz"
        np.savez_compressed(npz_path, image=sample["image"])

        # Save label as JSON
        label = {
            "id": sample_id,
            "timestamp": datetime.fromtimestamp(sample["timestamp"]).isoformat(),
            "navigation_token": sample["navigation_command"],
            "trajectory": trajectory_hybrid,
            "trajectory_2d": sample["waypoints_2d"],
            "kept_indices": sample["kept_indices"],
            "trajectory_3d": sample["waypoints_3d"],
            "vehicle_state": {
                **sample["vehicle_state"],
                "navigation_command": sample["navigation_command"],
                "decision_reasoning": sample["decision_reasoning"],
                "route_name": sample["route_name"],
            },
            "control": sample["control"],
            "image_file": f"{sample_id}.npz",
            "camera": {
                "width": self.config.camera_width,
                "height": self.config.camera_height,
                "fov": self.config.camera_fov,
                "position": self.config.camera_position,
                "rotation": self.config.camera_rotation,
            },
        }

        label_path = self.output_dir / "labels" / f"{sample_id}.json"
        with open(label_path, "w") as f:
            json.dump(label, f, indent=2)

        # Track route usage
        route_name = sample["route_name"]
        self.route_usage[route_name] = self.route_usage.get(route_name, 0) + 1

        self.sample_count += 1
        self.stats["saved"] += 1
        self.stats["save_times"].append(time.time() - start_time)

    def _save_manifest(self) -> None:
        """Save manifest with collection statistics."""
        elapsed = time.time() - self.stats["start_time"] if self.stats["start_time"] else 0
        avg_save_time = np.mean(self.stats["save_times"]) if self.stats["save_times"] else 0

        manifest = {
            "total_samples": self.sample_count,
            "collection_time_seconds": elapsed,
            "samples_per_second": self.sample_count / elapsed if elapsed > 0 else 0,
            "dropped_samples": self.stats["dropped"],
            "skipped_speed": self.stats["skipped_speed"],
            "skipped_waypoints": self.stats["skipped_waypoints"],
            "avg_save_time_ms": avg_save_time * 1000,
            "waypoint_distances": self.config.waypoint_distances,
            "camera_config": {
                "width": self.config.camera_width,
                "height": self.config.camera_height,
                "fov": self.config.camera_fov,
                "position": self.config.camera_position,
                "rotation": self.config.camera_rotation,
            },
            "route_usage": self.route_usage,
            "format": "npz_raw",
            "created": datetime.now().isoformat(),
        }

        manifest_path = self.output_dir / "manifest.json"
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=2)

        print(f"[DatasetRecorder] Manifest saved: {self.sample_count} samples")

    def get_stats(self) -> Dict:
        """Get collection statistics."""
        elapsed = time.time() - self.stats["start_time"] if self.stats["start_time"] else 0
        return {
            "saved": self.stats["saved"],
            "queued": self.stats["queued"],
            "dropped": self.stats["dropped"],
            "skipped_speed": self.stats["skipped_speed"],
            "skipped_waypoints": self.stats["skipped_waypoints"],
            "queue_size": self.data_queue.qsize(),
            "elapsed_seconds": elapsed,
            "samples_per_second": self.stats["saved"] / elapsed if elapsed > 0 else 0,
        }

    # =========================================================================
    # POST-PROCESSING
    # =========================================================================

    def convert_to_jpeg(self, quality: Optional[int] = None, delete_npz: bool = False) -> int:
        """
        Convert NPZ images to JPEG format.

        Args:
            quality: JPEG quality (1-100), defaults to config value
            delete_npz: Whether to delete NPZ files after conversion

        Returns:
            Number of files converted
        """
        from PIL import Image

        quality = quality or self.config.jpeg_quality
        raw_dir = self.output_dir / "raw"
        images_dir = self.output_dir / "images"
        labels_dir = self.output_dir / "labels"

        if not raw_dir.exists():
            print(f"No raw/ directory found in {self.output_dir}")
            return 0

        npz_files = sorted(raw_dir.glob("*.npz"))
        print(f"Converting {len(npz_files)} NPZ files to JPEG...")

        converted = 0
        errors = 0

        for npz_path in npz_files:
            try:
                sample_id = npz_path.stem

                # Load image from NPZ
                data = np.load(npz_path)
                image = data["image"]

                # Save as JPEG
                jpg_path = images_dir / f"{sample_id}.jpg"
                Image.fromarray(image).save(jpg_path, quality=quality)

                # Update label to point to JPEG
                label_path = labels_dir / f"{sample_id}.json"
                if label_path.exists():
                    with open(label_path, "r") as f:
                        label = json.load(f)
                    label["image_file"] = f"{sample_id}.jpg"
                    with open(label_path, "w") as f:
                        json.dump(label, f, indent=2)

                # Optionally delete NPZ
                if delete_npz:
                    npz_path.unlink()

                converted += 1

                if converted % 100 == 0:
                    print(f"  Converted {converted}/{len(npz_files)}...")

            except Exception as e:
                print(f"  Error converting {npz_path}: {e}")
                errors += 1

        # Update manifest
        manifest_path = self.output_dir / "manifest.json"
        if manifest_path.exists():
            with open(manifest_path, "r") as f:
                manifest = json.load(f)
            manifest["format"] = "jpeg"
            manifest["jpeg_quality"] = quality
            manifest["conversion_time"] = datetime.now().isoformat()
            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)

        print(f"Conversion complete! Converted: {converted}, Errors: {errors}")
        return converted

    def create_train_val_split(self, train_ratio: float = 0.9) -> None:
        """
        Create train/val split files.

        Args:
            train_ratio: Ratio of samples for training
        """
        labels_dir = self.output_dir / "labels"

        # Get all sample IDs
        label_files = sorted(labels_dir.glob("*.json"))
        sample_ids = [f.stem for f in label_files]

        # Shuffle and split
        np.random.seed(42)
        np.random.shuffle(sample_ids)

        split_idx = int(len(sample_ids) * train_ratio)
        train_ids = sample_ids[:split_idx]
        val_ids = sample_ids[split_idx:]

        # Save splits
        with open(self.output_dir / "train.txt", "w") as f:
            f.write("\n".join(train_ids))

        with open(self.output_dir / "val.txt", "w") as f:
            f.write("\n".join(val_ids))

        print(f"Created train/val split:")
        print(f"  Train: {len(train_ids)} samples")
        print(f"  Val: {len(val_ids)} samples")

    def create_hf_dataset(
        self,
        output_dir: Optional[str] = None,
        push_to_hub: Optional[str] = None,
    ) -> None:
        """
        Create HuggingFace dataset from recorded data.

        Args:
            output_dir: Output directory for HF dataset
            push_to_hub: HuggingFace Hub path (e.g., 'username/dataset')
        """
        try:
            from datasets import Dataset, Image as HFImage
        except ImportError:
            raise ImportError("datasets package not installed. Run: pip install datasets")

        images_dir = self.output_dir / "images"
        labels_dir = self.output_dir / "labels"

        # Check if images have been converted
        jpg_files = list(images_dir.glob("*.jpg"))
        if not jpg_files:
            print("No JPEG images found. Converting from NPZ first...")
            self.convert_to_jpeg()
            jpg_files = list(images_dir.glob("*.jpg"))

        # Load samples
        samples = []
        label_files = sorted(labels_dir.glob("*.json"))

        print(f"Loading {len(label_files)} samples...")

        for i, label_path in enumerate(label_files):
            sample_id = label_path.stem
            img_path = images_dir / f"{sample_id}.jpg"

            if not img_path.exists():
                continue

            with open(label_path, "r") as f:
                label = json.load(f)

            trajectory = label.get("trajectory", [])
            vehicle_state = label.get("vehicle_state", {})

            if not trajectory:
                continue

            samples.append(
                {
                    "image": str(img_path.absolute()),
                    "trajectory": json.dumps(trajectory),
                    "speed_kmh": float(f"{vehicle_state.get('speed_kmh', 0):.2f}"),
                    "sample_id": sample_id,
                    "navigation_command": vehicle_state.get(
                        "navigation_command", "lane_keeping"
                    ),
                    "reasoning": vehicle_state.get("decision_reasoning", ""),
                    "route_name": vehicle_state.get("route_name", "unknown"),
                }
            )

            if (i + 1) % 1000 == 0:
                print(f"  Processed {i + 1}/{len(label_files)}...")

        print(f"Loaded {len(samples)} valid samples")

        # Create dataset
        dataset = Dataset.from_list(samples)
        dataset = dataset.cast_column("image", HFImage())

        # Save locally
        if output_dir is None:
            output_dir = str(self.output_dir / "hf_dataset")

        print(f"Saving to {output_dir}...")
        dataset.save_to_disk(output_dir)

        # Preview
        print("\n" + "=" * 50)
        print("Sample preview:")
        print(f"  Image: {dataset[0]['image']}")
        print(f"  Trajectory: {dataset[0]['trajectory'][:80]}...")
        print(f"  Speed (km/h): {dataset[0]['speed_kmh']}")
        print(f"  Navigation: {dataset[0]['navigation_command']}")
        print(f"  Route: {dataset[0]['route_name']}")
        print("=" * 50)

        # Push to Hub if requested
        if push_to_hub:
            print(f"\nPushing to HuggingFace Hub: {push_to_hub}...")
            dataset.push_to_hub(push_to_hub)
            print("Done!")

        print(f"\nDataset saved to {output_dir}/")
        print(f"  Total samples: {len(dataset)}")

    def verify_dataset(self) -> Dict:
        """
        Verify dataset integrity.

        Returns:
            Dict with verification results
        """
        manifest_path = self.output_dir / "manifest.json"
        if not manifest_path.exists():
            print("No manifest.json found!")
            return {}

        with open(manifest_path, "r") as f:
            manifest = json.load(f)

        print(f"Dataset: {self.output_dir}")
        print(f"  Total samples: {manifest.get('total_samples', 'unknown')}")
        print(f"  Format: {manifest.get('format', 'unknown')}")
        print(f"  Waypoint distances: {manifest.get('waypoint_distances', 'unknown')}")

        # Check files
        labels_dir = self.output_dir / "labels"
        images_dir = self.output_dir / "images"
        raw_dir = self.output_dir / "raw"

        label_count = len(list(labels_dir.glob("*.json")))
        jpg_count = len(list(images_dir.glob("*.jpg")))
        npz_count = len(list(raw_dir.glob("*.npz"))) if raw_dir.exists() else 0

        print(f"\nFile counts:")
        print(f"  Labels (.json): {label_count}")
        print(f"  Images (.jpg): {jpg_count}")
        print(f"  Raw (.npz): {npz_count}")

        # Check for missing files
        missing = []
        for label_file in labels_dir.glob("*.json"):
            sample_id = label_file.stem
            jpg_path = images_dir / f"{sample_id}.jpg"
            npz_path = raw_dir / f"{sample_id}.npz"

            if not jpg_path.exists() and not npz_path.exists():
                missing.append(sample_id)

        if missing:
            print(f"\nWarning: Missing images for {len(missing)} labels!")
            print(f"  First few: {missing[:5]}")
        else:
            print(f"\nAll labels have corresponding images")

        return {
            "manifest": manifest,
            "label_count": label_count,
            "jpg_count": jpg_count,
            "npz_count": npz_count,
            "missing_images": missing,
        }


# =============================================================================
# CLI
# =============================================================================


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Dataset Recording and Processing Tools",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert NPZ to JPEG
  python dataset_recorder.py --convert ./datasets/my_dataset

  # Create train/val split
  python dataset_recorder.py --split ./datasets/my_dataset --train-ratio 0.9

  # Create HuggingFace dataset
  python dataset_recorder.py --create-hf ./datasets/my_dataset

  # Push to HuggingFace Hub
  python dataset_recorder.py --create-hf ./datasets/my_dataset --push username/dataset

  # Verify dataset
  python dataset_recorder.py --verify ./datasets/my_dataset
        """,
    )

    parser.add_argument(
        "--convert", type=str, help="Convert NPZ to JPEG in directory"
    )
    parser.add_argument(
        "--quality", type=int, default=95, help="JPEG quality (1-100)"
    )
    parser.add_argument(
        "--delete-npz", action="store_true", help="Delete NPZ after conversion"
    )
    parser.add_argument(
        "--split", type=str, help="Create train/val split for directory"
    )
    parser.add_argument(
        "--train-ratio", type=float, default=0.9, help="Train split ratio"
    )
    parser.add_argument(
        "--create-hf", type=str, help="Create HuggingFace dataset from directory"
    )
    parser.add_argument(
        "--push", type=str, help="Push to HuggingFace Hub (with --create-hf)"
    )
    parser.add_argument(
        "--verify", type=str, help="Verify dataset integrity"
    )

    args = parser.parse_args()

    if args.convert:
        recorder = DatasetRecorder(output_dir=args.convert)
        recorder.convert_to_jpeg(quality=args.quality, delete_npz=args.delete_npz)

    if args.split:
        recorder = DatasetRecorder(output_dir=args.split)
        recorder.create_train_val_split(train_ratio=args.train_ratio)

    if args.create_hf:
        recorder = DatasetRecorder(output_dir=args.create_hf)
        recorder.create_hf_dataset(push_to_hub=args.push)

    if args.verify:
        recorder = DatasetRecorder(output_dir=args.verify)
        recorder.verify_dataset()

    if not any([args.convert, args.split, args.create_hf, args.verify]):
        parser.print_help()


if __name__ == "__main__":
    main()
