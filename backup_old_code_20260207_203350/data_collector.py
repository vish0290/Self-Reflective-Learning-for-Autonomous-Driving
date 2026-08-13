#!/usr/bin/env python3
"""
Async Data Collection Pipeline for VLM Training

Features:
1. Non-blocking data saving using background thread
2. Saves to .npz format (fast) during driving
3. Separate conversion to JPEG after driving session
4. Queue-based buffering to prevent frame drops

Usage:
    # During driving (in main.py)
    collector = AsyncDataCollector(output_dir='./trajectory_data')
    collector.start()
    
    # In main loop
    collector.queue_sample(image, waypoints_2d, waypoints_3d, ...)
    
    # After driving
    collector.stop()
    
    # Later, convert to JPEG
    python data_collector.py --convert ./trajectory_data
"""

import numpy as np
import json
import threading
import queue
import time
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional
import argparse
from navigation_analyzer import generate_navigation_token


class AsyncDataCollector:
    """
    Asynchronous data collector that doesn't block the main driving loop.
    
    Strategy:
    1. Main thread queues data (fast - just memory copy)
    2. Background thread saves to .npz files (slower but async)
    3. Post-processing converts .npz to JPEG
    """
    
    def __init__(self, output_dir: str, 
                 max_queue_size: int = 100,
                 save_interval: int = 1):
        """
        Args:
            output_dir: Directory to save data
            max_queue_size: Max items in queue before dropping
            save_interval: Save every N queued samples (1 = save all)
        """
        self.output_dir = Path(output_dir)
        self.max_queue_size = max_queue_size
        self.save_interval = save_interval
        
        # Create directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'raw').mkdir(exist_ok=True)  # .npz files
        (self.output_dir / 'images').mkdir(exist_ok=True)  # JPEG (after conversion)
        (self.output_dir / 'labels').mkdir(exist_ok=True)  # JSON labels
        
        # Queue and threading
        self.data_queue = queue.Queue(maxsize=max_queue_size)
        self.worker_thread = None
        self.running = False
        
        # Statistics
        self.stats = {
            'queued': 0,
            'saved': 0,
            'dropped': 0,
            'start_time': None,
            'save_times': []
        }
        
        # Sample counter
        self.sample_count = 0
        self.queue_count = 0
        
        # Camera config (will be set from first sample)
        self.camera_config = None
        self.waypoint_distances = None
    
    def _generate_navigation_token(self, waypoints_3d: List[List[float]]) -> str:
        """Generate navigation token from 3D waypoints."""
        if not waypoints_3d or len(waypoints_3d) < 2:
            return 'lane_keeping'
        return generate_navigation_token(waypoints_3d, use_map_api=False, carla_map=None)
    
    def _format_ego_stats(self, vehicle_state: dict) -> str:
        """Format ego vehicle stats as JSON string."""
        speed_kmh = vehicle_state.get('speed_kmh', 0)
        rotation = vehicle_state.get('rotation', [0, 0, 0, 0])
        location = vehicle_state.get('location', [0, 0, 0])
        control = vehicle_state.get('control', {})
        
        ego_stats = {
            'speed_kmh': speed_kmh,
            'heading': rotation[1] if rotation else 0.0,
            'location': location,
            'control': {
                'throttle': control.get('throttle', 0),
                'brake': control.get('brake', 0),
                'steer': control.get('steer', 0)
            }
        }
        return json.dumps(ego_stats)
    
    def start(self):
        """Start the background saving thread."""
        if self.running:
            return
        
        self.running = True
        self.stats['start_time'] = time.time()
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()
        print(f"[DataCollector] Started. Saving to: {self.output_dir}")
    
    def stop(self):
        """Stop the collector and wait for queue to drain."""
        if not self.running:
            return
        
        print(f"[DataCollector] Stopping... ({self.data_queue.qsize()} items in queue)")
        self.running = False
        
        # Wait for queue to drain (with timeout)
        timeout = 30  # seconds
        start = time.time()
        while not self.data_queue.empty() and (time.time() - start) < timeout:
            time.sleep(0.1)
        
        if self.worker_thread:
            self.worker_thread.join(timeout=5)
        
        # Save manifest
        self._save_manifest()
        
        print(f"[DataCollector] Stopped. Stats: {self.get_stats()}")
    
    def set_config(self, camera_config: dict, waypoint_distances: list):
        """Set camera and waypoint configuration."""
        self.camera_config = camera_config
        self.waypoint_distances = waypoint_distances
    
    def queue_sample(self, 
                     rgb_image: np.ndarray,
                     waypoints_2d: List,
                     waypoints_3d: List,
                     kept_indices: List[int],
                     vehicle_state: dict,
                     control: dict,
                     timestamp: float = None) -> bool:
        """
        Queue a sample for async saving.
        
        Returns True if queued, False if dropped (queue full).
        """
        self.queue_count += 1
        
        # Apply save interval
        if self.queue_count % self.save_interval != 0:
            return True  # Skip but don't count as dropped
        
        # Prepare sample data
        sample = {
            'image': rgb_image.copy(),  # Copy to prevent mutation
            'waypoints_2d': [[int(wp.u), int(wp.v)] for wp in waypoints_2d],
            'waypoints_3d': [[wp.x, wp.y, wp.z] for wp in waypoints_3d],
            'kept_indices': kept_indices,
            'vehicle_state': vehicle_state,
            'control': control,
            'timestamp': timestamp or time.time()
        }
        
        try:
            self.data_queue.put_nowait(sample)
            self.stats['queued'] += 1
            return True
        except queue.Full:
            self.stats['dropped'] += 1
            return False
    
    def _worker_loop(self):
        """Background worker that saves data from queue."""
        while self.running or not self.data_queue.empty():
            try:
                sample = self.data_queue.get(timeout=0.5)
                self._save_sample(sample)
                self.data_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[DataCollector] Error saving: {e}")
    
    def _save_sample(self, sample: dict):
        """Save a single sample to disk."""
        start_time = time.time()
        
        sample_id = f"{self.sample_count:06d}"
        
        # Build trajectory with distances (hybrid format for VLM)
        trajectory_hybrid = []
        for i, (u, v) in enumerate(sample['waypoints_2d']):
            if sample['kept_indices'] and i < len(sample['kept_indices']):
                orig_idx = sample['kept_indices'][i]
                dist = self.waypoint_distances[orig_idx] if self.waypoint_distances else 0
            else:
                dist = 0
            trajectory_hybrid.append([u, v, dist])
        
        # Generate navigation token and ego stats
        navigation_token = self._generate_navigation_token(sample['waypoints_3d'])
        ego_stats_json = self._format_ego_stats(sample['vehicle_state'])
        
        # Save image as .npz (fast!)
        npz_path = self.output_dir / 'raw' / f"{sample_id}.npz"
        np.savez_compressed(npz_path, image=sample['image'])
        
        # Save label as JSON
        label = {
            'id': sample_id,
            'timestamp': datetime.fromtimestamp(sample['timestamp']).isoformat(),
            
            # Navigation token
            'navigation_token': navigation_token,
            
            # Ego stats (JSON string)
            'ego_stats': ego_stats_json,
            
            # Primary format: [u, v, distance] for VLM training
            'trajectory': trajectory_hybrid,
            
            # Raw 2D coordinates
            'trajectory_2d': sample['waypoints_2d'],
            
            # Which waypoints are visible
            'kept_indices': sample['kept_indices'],
            
            # 3D for validation
            'trajectory_3d': sample['waypoints_3d'],
            
            # Vehicle state
            'vehicle_state': sample['vehicle_state'],
            
            # Control (for imitation learning)
            'control': sample['control'],
            
            # Image reference
            'image_file': f"{sample_id}.npz",
            
            # Camera config
            'camera': self.camera_config
        }
        
        label_path = self.output_dir / 'labels' / f"{sample_id}.json"
        with open(label_path, 'w') as f:
            json.dump(label, f, indent=2)
        
        self.sample_count += 1
        self.stats['saved'] += 1
        self.stats['save_times'].append(time.time() - start_time)
    
    def _save_manifest(self):
        """Save manifest with collection statistics."""
        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        avg_save_time = np.mean(self.stats['save_times']) if self.stats['save_times'] else 0
        
        manifest = {
            'total_samples': self.sample_count,
            'collection_time_seconds': elapsed,
            'samples_per_second': self.sample_count / elapsed if elapsed > 0 else 0,
            'dropped_samples': self.stats['dropped'],
            'avg_save_time_ms': avg_save_time * 1000,
            'waypoint_distances': self.waypoint_distances,
            'camera_config': self.camera_config,
            'format': 'npz_raw',  # Indicates images need conversion
            'created': datetime.now().isoformat()
        }
        
        manifest_path = self.output_dir / 'manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        
        print(f"[DataCollector] Manifest saved: {self.sample_count} samples")
    
    def get_stats(self) -> dict:
        """Get collection statistics."""
        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        return {
            'saved': self.stats['saved'],
            'queued': self.stats['queued'],
            'dropped': self.stats['dropped'],
            'queue_size': self.data_queue.qsize(),
            'elapsed_seconds': elapsed,
            'samples_per_second': self.stats['saved'] / elapsed if elapsed > 0 else 0
        }


# =============================================================================
# POST-PROCESSING: Convert .npz to JPEG
# =============================================================================

def convert_npz_to_jpeg(data_dir: str, quality: int = 95, delete_npz: bool = False):
    """
    Convert .npz images to JPEG format.
    
    Run this AFTER the driving session to convert raw data.
    
    Args:
        data_dir: Directory containing raw/ folder with .npz files
        quality: JPEG quality (1-100)
        delete_npz: Whether to delete .npz files after conversion
    """
    from PIL import Image
    
    data_path = Path(data_dir)
    raw_dir = data_path / 'raw'
    images_dir = data_path / 'images'
    labels_dir = data_path / 'labels'
    
    if not raw_dir.exists():
        print(f"No raw/ directory found in {data_dir}")
        return
    
    npz_files = sorted(raw_dir.glob('*.npz'))
    print(f"Converting {len(npz_files)} .npz files to JPEG...")
    
    converted = 0
    errors = 0
    
    for npz_path in npz_files:
        try:
            sample_id = npz_path.stem
            
            # Load image from .npz
            data = np.load(npz_path)
            image = data['image']
            
            # Save as JPEG
            jpg_path = images_dir / f"{sample_id}.jpg"
            Image.fromarray(image).save(jpg_path, quality=quality)
            
            # Update label to point to JPEG
            label_path = labels_dir / f"{sample_id}.json"
            if label_path.exists():
                with open(label_path, 'r') as f:
                    label = json.load(f)
                label['image_file'] = f"{sample_id}.jpg"
                with open(label_path, 'w') as f:
                    json.dump(label, f, indent=2)
            
            # Optionally delete .npz
            if delete_npz:
                npz_path.unlink()
            
            converted += 1
            
            if converted % 100 == 0:
                print(f"  Converted {converted}/{len(npz_files)}...")
                
        except Exception as e:
            print(f"  Error converting {npz_path}: {e}")
            errors += 1
    
    # Update manifest
    manifest_path = data_path / 'manifest.json'
    if manifest_path.exists():
        with open(manifest_path, 'r') as f:
            manifest = json.load(f)
        manifest['format'] = 'jpeg'
        manifest['jpeg_quality'] = quality
        manifest['conversion_time'] = datetime.now().isoformat()
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
    
    print(f"\nConversion complete!")
    print(f"  Converted: {converted}")
    print(f"  Errors: {errors}")
    if delete_npz:
        print(f"  Deleted .npz files: {converted}")


def create_training_split(data_dir: str, train_ratio: float = 0.9):
    """
    Create train/val split files.
    
    Args:
        data_dir: Directory with manifest.json
        train_ratio: Ratio of samples for training (rest is validation)
    """
    data_path = Path(data_dir)
    labels_dir = data_path / 'labels'
    
    # Get all sample IDs
    label_files = sorted(labels_dir.glob('*.json'))
    sample_ids = [f.stem for f in label_files]
    
    # Shuffle and split
    np.random.seed(42)
    np.random.shuffle(sample_ids)
    
    split_idx = int(len(sample_ids) * train_ratio)
    train_ids = sample_ids[:split_idx]
    val_ids = sample_ids[split_idx:]
    
    # Save splits
    with open(data_path / 'train.txt', 'w') as f:
        f.write('\n'.join(train_ids))
    
    with open(data_path / 'val.txt', 'w') as f:
        f.write('\n'.join(val_ids))
    
    print(f"Created train/val split:")
    print(f"  Train: {len(train_ids)} samples")
    print(f"  Val: {len(val_ids)} samples")


def verify_dataset(data_dir: str):
    """Verify dataset integrity."""
    data_path = Path(data_dir)
    
    manifest_path = data_path / 'manifest.json'
    if not manifest_path.exists():
        print("No manifest.json found!")
        return
    
    with open(manifest_path, 'r') as f:
        manifest = json.load(f)
    
    print(f"Dataset: {data_dir}")
    print(f"  Total samples: {manifest.get('total_samples', 'unknown')}")
    print(f"  Format: {manifest.get('format', 'unknown')}")
    print(f"  Waypoint distances: {manifest.get('waypoint_distances', 'unknown')}")
    
    # Check files
    labels_dir = data_path / 'labels'
    images_dir = data_path / 'images'
    raw_dir = data_path / 'raw'
    
    label_count = len(list(labels_dir.glob('*.json')))
    jpg_count = len(list(images_dir.glob('*.jpg')))
    npz_count = len(list(raw_dir.glob('*.npz'))) if raw_dir.exists() else 0
    
    print(f"\nFile counts:")
    print(f"  Labels (.json): {label_count}")
    print(f"  Images (.jpg): {jpg_count}")
    print(f"  Raw (.npz): {npz_count}")
    
    # Check for missing files
    missing = []
    for label_file in labels_dir.glob('*.json'):
        sample_id = label_file.stem
        jpg_path = images_dir / f"{sample_id}.jpg"
        npz_path = raw_dir / f"{sample_id}.npz"
        
        if not jpg_path.exists() and not npz_path.exists():
            missing.append(sample_id)
    
    if missing:
        print(f"\n⚠ Missing images for {len(missing)} labels!")
        print(f"  First few: {missing[:5]}")
    else:
        print(f"\n✓ All labels have corresponding images")


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Data Collection Pipeline Tools')
    parser.add_argument('--convert', type=str, help='Convert .npz to JPEG in directory')
    parser.add_argument('--quality', type=int, default=95, help='JPEG quality (1-100)')
    parser.add_argument('--delete-npz', action='store_true', help='Delete .npz after conversion')
    parser.add_argument('--split', type=str, help='Create train/val split for directory')
    parser.add_argument('--train-ratio', type=float, default=0.9, help='Train split ratio')
    parser.add_argument('--verify', type=str, help='Verify dataset integrity')
    
    args = parser.parse_args()
    
    if args.convert:
        convert_npz_to_jpeg(args.convert, args.quality, args.delete_npz)
    
    if args.split:
        create_training_split(args.split, args.train_ratio)
    
    if args.verify:
        verify_dataset(args.verify)
    
    if not any([args.convert, args.split, args.verify]):
        parser.print_help()
        print("\nExamples:")
        print("  python data_collector.py --convert ./trajectory_data")
        print("  python data_collector.py --convert ./trajectory_data --delete-npz")
        print("  python data_collector.py --split ./trajectory_data --train-ratio 0.9")
        print("  python data_collector.py --verify ./trajectory_data")
