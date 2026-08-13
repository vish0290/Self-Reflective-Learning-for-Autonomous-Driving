#!/usr/bin/env python3
"""
Simple script to convert trajectory_data to HuggingFace Dataset.

Usage:
    python create_hf_dataset.py
    python create_hf_dataset.py --push YOUR_HF_USERNAME/dataset_name
"""

import json
from pathlib import Path
from PIL import Image
from datasets import Dataset, Features, Value, Sequence, Image as HFImage
import random

# Config
DATA_DIR = "./trajectory_data"
OUTPUT_DIR = "./hf_trajectory_dataset"

# Instruction templates for variety during training
INSTRUCTIONS = [
    "Predict the trajectory waypoints for this driving scene as [[x, y, distance], ...] where x,y are pixel coordinates and distance is in meters.",
    "Given this front camera view, output the planned driving trajectory as pixel coordinates with distances: [[x, y, d], ...]",
    "Analyze this driving image and predict waypoints along the road. Format: [[pixel_x, pixel_y, distance_m], ...]",
    "What is the trajectory path for this driving scene? Output as [[x, y, distance], ...] in pixel coordinates.",
]


def load_trajectory_data(data_dir: str):
    """Load all trajectory data samples."""
    data_path = Path(data_dir)
    images_dir = data_path / 'images'
    labels_dir = data_path / 'labels'
    
    samples = []
    label_files = sorted(labels_dir.glob('*.json'))
    
    print(f"Found {len(label_files)} label files")
    
    for i, label_path in enumerate(label_files):
        if i % 1000 == 0:
            print(f"  Processing {i}/{len(label_files)}...")
        
        sample_id = label_path.stem
        img_path = images_dir / f"{sample_id}.jpg"
        
        if not img_path.exists():
            continue
        
        # Load label
        with open(label_path, 'r') as f:
            label = json.load(f)
        
        # Get trajectory (already in hybrid format [[x, y, distance], ...])
        trajectory = label.get('trajectory', [])
        vehicle_state = label.get('vehicle_state', {})
        speed_kmh = vehicle_state.get('speed_kmh', 0.0)
        speed_kmh = float(f"{speed_kmh:.2f}")
        navigation_command = vehicle_state.get('navigation_command', 'UNKNOWN')
        decision_reasoning = vehicle_state.get('decision_reasoning', '')
        route_name = vehicle_state.get('route_name', 'unknown_route')
        if not trajectory:
            continue
        
        samples.append({
            'image': str(img_path.absolute()),
            'trajectory': json.dumps(trajectory),  # String format for VLM output
            'speed_kmh': speed_kmh,
            'sample_id': sample_id,
            'navigation_command': navigation_command,
            'reasoning': decision_reasoning,
            'route_name': route_name

        })
    
    print(f"Loaded {len(samples)} valid samples")
    return samples


def create_dataset(samples: list):
    """Create HuggingFace Dataset from samples."""
    
    print(f"Total samples: {len(samples)}")
    
    # Create dataset
    dataset = Dataset.from_list(samples)
    
    # Cast image column to Image type
    dataset = dataset.cast_column("image", HFImage())
    
    return dataset


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default=DATA_DIR, help='Input data directory')
    parser.add_argument('--output', default=OUTPUT_DIR, help='Output directory')
    parser.add_argument('--push', default=None, help='Push to HuggingFace Hub (e.g., username/dataset_name)')
    args = parser.parse_args()
    
    # Load data
    print(f"Loading data from {args.input}...")
    samples = load_trajectory_data(args.input)
    
    if not samples:
        print("No samples found!")
        return
    
    # Create dataset
    print("Creating HuggingFace Dataset...")
    dataset = create_dataset(samples)
    
    # Save locally
    print(f"Saving to {args.output}...")
    dataset.save_to_disk(args.output)
    
    # Preview
    print("\n" + "="*50)
    print("Sample preview:")
    print(f"  Image: {dataset[0]['image']}")
    print(f"  Trajectory: {dataset[0]['trajectory'][:80]}...")
    print(f"  Speed (km/h): {dataset[0]['speed_kmh']}")
    print(f"  Navigation Command: {dataset[0]['navigation_command']}")
    print(f"  Reasoning: {dataset[0]['reasoning'][:80]}...")
    print(f"  Route Name: {dataset[0]['route_name']}")
    print("="*50)
    
    # Push to Hub if requested
    if args.push:
        print(f"\nPushing to HuggingFace Hub: {args.push}...")
        dataset.push_to_hub(args.push)
        print("Done!")
    
    print(f"\nDataset saved to {args.output}/")
    print(f"  Total samples: {len(dataset)}")


if __name__ == '__main__':
    main()
