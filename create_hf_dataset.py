"""
Create Hugging Face dataset from CARLA simulation data.
Reads labels from core/datasets/labels/ and images from core/datasets/images/
"""

import json
import os
from pathlib import Path
from PIL import Image
from datasets import Dataset, Features, Value, Image as HFImage
from typing import Dict, List, Any
import pandas as pd


class DatasetCreator:
    def __init__(self, dataset_dir: str = None, labels_dir: str = None, images_dir: str = None):
        """
        Initialize DatasetCreator.

        Args:
            dataset_dir: Path to dataset directory (e.g., "./datasets/dataset_TIMESTAMP")
                        If provided, labels_dir and images_dir are ignored
            labels_dir: Direct path to labels directory (legacy)
            images_dir: Direct path to images directory (legacy)
        """
        if dataset_dir is not None:
            # Modern approach: use dataset directory structure
            dataset_path = Path(dataset_dir)
            self.labels_dir = dataset_path / 'labels'
            self.images_dir = dataset_path / 'images'
            print(f"Using dataset directory: {dataset_dir}")
        elif labels_dir is not None and images_dir is not None:
            # Legacy approach: direct paths
            self.labels_dir = Path(labels_dir)
            self.images_dir = Path(images_dir)
            print(f"Using custom paths:")
            print(f"  Labels: {labels_dir}")
            print(f"  Images: {images_dir}")
        else:
            # Auto-detect latest dataset
            datasets_root = Path("./datasets")
            if datasets_root.exists():
                dataset_dirs = sorted([d for d in datasets_root.iterdir() if d.is_dir() and d.name.startswith("dataset_")])
                if dataset_dirs:
                    latest_dataset = dataset_dirs[-1]  # Most recent
                    self.labels_dir = latest_dataset / 'labels'
                    self.images_dir = latest_dataset / 'images'
                    print(f"Auto-detected latest dataset: {latest_dataset.name}")
                else:
                    raise ValueError("No dataset directories found in ./datasets/")
            else:
                raise ValueError("No datasets directory found! Run record_dataset.py first.")

        # Validate paths exist
        if not self.labels_dir.exists():
            raise ValueError(f"Labels directory not found: {self.labels_dir}")
        if not self.images_dir.exists():
            raise ValueError(f"Images directory not found: {self.images_dir}")

        print(f"  Labels directory: {self.labels_dir} ({len(list(self.labels_dir.glob('*.json')))} files)")
        print(f"  Images directory: {self.images_dir} ({len(list(self.images_dir.glob('*.jpg')))} files)")

    def parse_label_file(self, label_path: Path) -> Dict[str, Any]:
        """Parse a single label JSON file and extract required fields."""
        with open(label_path, 'r') as f:
            data = json.load(f)

        # Extract control information
        control = data.get('control', {})
        vehicle_state = data.get('vehicle_state', {})

        # Extract path_id from route_name
        route_name = vehicle_state.get('route_name', 'unknown')
        navigation = data.get('navigation_token', '')

        return {
            'frame_id': data.get('id', ''),
            'current_speed': vehicle_state.get('speed_kmh', 0.0),
            'steer': control.get('steer', 0.0),
            'throttle': control.get('throttle', 0.0),
            'brake': control.get('brake', 0.0),
            'path_id': route_name,
            'navigation': navigation,
            'image_file': data.get('image_file', '')
        }

    def scan_and_collect_data(self) -> List[Dict[str, Any]]:
        """Scan all label files and collect data."""
        dataset_entries = []

        # Get all JSON files from labels directory
        label_files = sorted(self.labels_dir.glob('*.json'))

        print(f"Found {len(label_files)} label files")

        for label_file in label_files:
            try:
                # Parse label
                entry = self.parse_label_file(label_file)

                # Check if corresponding image exists
                image_path = self.images_dir / entry['image_file']

                if image_path.exists():
                    entry['frame'] = str(image_path)
                    dataset_entries.append(entry)
                else:
                    print(f"Warning: Image not found for {entry['frame_id']}: {image_path}")

            except Exception as e:
                print(f"Error processing {label_file}: {e}")
                continue

        print(f"Successfully processed {len(dataset_entries)} entries")
        return dataset_entries

    def create_huggingface_dataset(self, output_dir: str = "hf_dataset") -> Dataset:
        """Create and save Hugging Face dataset."""
        # Collect all data
        data_entries = self.scan_and_collect_data()

        if not data_entries:
            raise ValueError("No data entries found!")

        # Convert to format suitable for HF Dataset
        dataset_dict = {
            'frame': [],
            'current_speed': [],
            'steer': [],
            'throttle': [],
            'brake': [],
            'frame_id': [],
            'path_id': [],
            'navigation': []
        }

        for entry in data_entries:
            dataset_dict['frame'].append(entry['frame'])
            dataset_dict['current_speed'].append(entry['current_speed'])
            dataset_dict['steer'].append(entry['steer'])
            dataset_dict['throttle'].append(entry['throttle'])
            dataset_dict['brake'].append(entry['brake'])
            dataset_dict['frame_id'].append(entry['frame_id'])
            dataset_dict['path_id'].append(entry['path_id'])
            dataset_dict['navigation'].append(entry.get('navigation', ''))
        # Define features schema
        features = Features({
            'frame': HFImage(),
            'current_speed': Value('float32'),
            'steer': Value('float32'),
            'throttle': Value('float32'),
            'brake': Value('float32'),
            'frame_id': Value('string'),
            'path_id': Value('string'),
            'navigation': Value('string')
        })

        # Create dataset
        dataset = Dataset.from_dict(dataset_dict, features=features)

        # Save to disk
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True, parents=True)
        dataset.save_to_disk(str(output_path))

        print(f"\nDataset saved to {output_path}")
        print(f"Total samples: {len(dataset)}")

        return dataset

    def create_dataframe_preview(self) -> pd.DataFrame:
        """Create a pandas DataFrame for quick preview (without loading images)."""
        data_entries = self.scan_and_collect_data()

        df_data = {
            'frame_id': [e['frame_id'] for e in data_entries],
            'current_speed': [e['current_speed'] for e in data_entries],
            'steer': [e['steer'] for e in data_entries],
            'throttle': [e['throttle'] for e in data_entries],
            'brake': [e['brake'] for e in data_entries],
            'path_id': [e['path_id'] for e in data_entries],
            'image_path': [e['frame'] for e in data_entries],
            'navigation': [e.get('navigation', '') for e in data_entries]
        }

        return pd.DataFrame(df_data)


def main(dataset_dir: str = None):
    """
    Main function to create the dataset.

    Args:
        dataset_dir: Path to dataset directory (e.g., "./datasets/dataset_20260214_143022")
                    If None, auto-detects the latest dataset
    """
    print("=" * 60)
    print("CARLA Dataset Creator for Hugging Face")
    print("=" * 60)

    # Initialize creator
    creator = DatasetCreator(dataset_dir=dataset_dir)

    # Create preview DataFrame
    print("\nCreating preview DataFrame...")
    df = creator.create_dataframe_preview()
    print("\nDataset Preview:")
    print(df.head(10))
    print(f"\nDataset Statistics:")
    print(df.describe())
    print(f"\nUnique paths: {df['path_id'].unique()}")

    # Create Hugging Face dataset
    print("\n" + "=" * 60)
    print("Creating Hugging Face Dataset...")
    print("=" * 60)
    dataset = creator.create_huggingface_dataset()

    print("\nDataset Info:")
    print(dataset)

    # Optional: Push to Hugging Face Hub
    # Uncomment and configure if you want to push to HF Hub
    dataset.push_to_hub("VishwanathAS/carla_ego_state_town10")

    return dataset


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Create Hugging Face dataset from CARLA data')
    parser.add_argument('--dataset', type=str, default=None,
                        help='Path to dataset directory (e.g., ./datasets/dataset_20260214_143022). '
                             'If not specified, uses the latest dataset.')
    parser.add_argument('--output', type=str, default='hf_dataset',
                        help='Output directory for HF dataset (default: hf_dataset)')

    args = parser.parse_args()

    # Update creator to accept output path
    print("=" * 60)
    print("CARLA Dataset Creator for Hugging Face")
    print("=" * 60)

    creator = DatasetCreator(dataset_dir=args.dataset)

    # Create preview DataFrame
    print("\nCreating preview DataFrame...")
    df = creator.create_dataframe_preview()
    print("\nDataset Preview:")
    print(df.head(10))
    print(f"\nDataset Statistics:")
    print(df.describe())
    print(f"\nUnique paths: {df['path_id'].unique()}")

    # Create Hugging Face dataset
    print("\n" + "=" * 60)
    print("Creating Hugging Face Dataset...")
    print("=" * 60)
    dataset = creator.create_huggingface_dataset(output_dir=args.output)

    print("\nDataset Info:")
    print(dataset)

    # Optional: Push to Hugging Face Hub
    # Uncomment and configure if you want to push to HF Hub
    # dataset.push_to_hub("VishwanathAS/carla_ego_state_town10")
