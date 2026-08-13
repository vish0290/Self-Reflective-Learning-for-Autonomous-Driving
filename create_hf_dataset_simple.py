#!/usr/bin/env python3
"""
Simple HF Dataset Creator - No Overcomplication!

Usage:
    python create_hf_dataset_simple.py
    python create_hf_dataset_simple.py --dataset ./core/dataset
    python create_hf_dataset_simple.py --output my_hf_dataset
"""

import json
from pathlib import Path
from datasets import Dataset, Features, Value, Image as HFImage

def create_hf_dataset(dataset_dir="./core/dataset", output_dir="./hf_dataset"):
    """
    Create HF dataset from CARLA data.

    Simple and straightforward - just reads labels and matches images.
    """
    dataset_path = Path(dataset_dir)
    labels_dir = dataset_path / 'labels'
    images_dir = dataset_path / 'images'

    print(f"Reading from: {dataset_dir}")
    print(f"  Labels: {labels_dir}")
    print(f"  Images: {images_dir}")

    # Get all label files
    label_files = sorted(labels_dir.glob('*.json'))
    print(f"\nFound {len(label_files)} samples")

    if len(label_files) == 0:
        raise ValueError(f"No label files found in {labels_dir}")

    # Collect data
    data = {
        'image': [],
        'speed': [],
        'steer': [],
        'throttle': [],
        'brake': [],
        'navigation': [],
        'route': [],
    }

    print("Processing samples...")
    for i, label_file in enumerate(label_files):
        # Load label
        with open(label_file) as f:
            label = json.load(f)

        # Get image path
        image_file = label['image_file']
        image_path = images_dir / image_file

        if not image_path.exists():
            print(f"  Warning: Image not found: {image_file}")
            continue

        # Extract data
        control = label['control']

        data['image'].append(str(image_path))
        data['speed'].append(label['speed_kmh'])
        data['steer'].append(control['steer'])
        data['throttle'].append(control['throttle'])
        data['brake'].append(control['brake'])
        data['navigation'].append(label['train_nav_command'])
        data['route'].append(label['route_name'])

        if (i + 1) % 1000 == 0:
            print(f"  Processed {i + 1}/{len(label_files)}...")

    print(f"\nCollected {len(data['image'])} valid samples")

    # Define features
    features = Features({
        'image': HFImage(),
        'speed': Value('float32'),
        'steer': Value('float32'),
        'throttle': Value('float32'),
        'brake': Value('float32'),
        'navigation': Value('string'),
        'route': Value('string'),
    })

    # Create dataset
    print("\nCreating HuggingFace dataset...")
    dataset = Dataset.from_dict(data, features=features)

    # Save
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    dataset.save_to_disk(str(output_path))

    print(f"\n✓ Dataset saved to: {output_path}")
    print(f"  Total samples: {len(dataset)}")
    print(f"\nDataset info:")
    print(dataset)

    # Show sample
    print("\nFirst sample:")
    sample = dataset[0]
    print(f"  Speed: {sample['speed']:.1f} km/h")
    print(f"  Navigation: {sample['navigation']}")
    print(f"  Steer: {sample['steer']:.3f}")
    print(f"  Throttle: {sample['throttle']:.2f}")
    print(f"  Brake: {sample['brake']:.1f}")
    print(f"  Route: {sample['route']}")

    return dataset

def push_to_hub(dataset_dir="./hf_dataset", repo_id="VishwanathAS/Carla_dataset_ego_state_town10"):
    from datasets import load_from_disk
    from huggingface_hub import login
    
    login()  # Or pass token='hf_...'
    dataset = load_from_disk(dataset_dir)
    dataset.push_to_hub(repo_id)
    print(f"✓ Uploaded to https://huggingface.co/datasets/{repo_id}")

# Then call: push_to_hub("./hf_dataset", "vishwanath/autonomous-driving")

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Create HF dataset - Simple version')
    parser.add_argument('--dataset', type=str, default='./core/dataset',
                        help='Path to dataset directory (default: ./core/dataset)')
    parser.add_argument('--output', type=str, default='./hf_dataset',
                        help='Output directory (default: ./hf_dataset)')
    parser.add_argument('--push', action='store_true',
                        help='Push to HuggingFace Hub (requires --repo-id)')


    args = parser.parse_args()

    create_hf_dataset(args.dataset, args.output)
    
    if args.push:
        push_to_hub(args.output, "VishwanathAS/carla_ego_state_town10")