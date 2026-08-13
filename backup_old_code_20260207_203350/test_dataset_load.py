#!/usr/bin/env python3
"""Test loading the local dataset to debug evaluation issues."""

from datasets import Dataset
import glob
from pathlib import Path
import ast
import numpy as np

# Load dataset
dataset_path = Path("./hf_trajectory_dataset")
arrow_files = glob.glob(str(dataset_path / "data-*.arrow"))
print(f"Found {len(arrow_files)} Arrow files in {dataset_path.absolute()}")

dataset = Dataset.from_file(arrow_files[0])
print(f"\nLoaded dataset with {len(dataset)} samples")
print(f"Schema: {dataset.features}\n")

# Test loading first sample
sample = dataset[0]
print("First sample:")
print(f"  Route: {sample.get('route_name', 'N/A')}")
print(f"  Speed: {sample.get('speed_kmh', 'N/A')} km/h")
print(f"  Navigation: {sample.get('navigation_command', 'N/A')}")
print(f"  Image type: {type(sample['image'])}")
print(f"  Image size: {sample['image'].size if hasattr(sample['image'], 'size') else 'N/A'}")

# Parse trajectory
trajectory_raw = sample['trajectory']
print(f"  Trajectory (raw): {trajectory_raw[:100]}...")
print(f"  Trajectory type: {type(trajectory_raw)}")

if isinstance(trajectory_raw, str):
    trajectory = ast.literal_eval(trajectory_raw)
    print(f"  Trajectory (parsed): {trajectory[:3]}... ({len(trajectory)} waypoints)")
else:
    print(f"  Trajectory (list): {trajectory_raw[:3]}... ({len(trajectory_raw)} waypoints)")

# Convert image to numpy
image_np = np.array(sample['image'])
print(f"  Image numpy shape: {image_np.shape}")
print(f"  Image numpy dtype: {image_np.dtype}")

print("\n✓ Dataset loading test passed!")
