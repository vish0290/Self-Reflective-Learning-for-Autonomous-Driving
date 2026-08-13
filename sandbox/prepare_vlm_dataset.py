#!/usr/bin/env python3
"""
Dataset Preparation for Qwen3-VL Fine-tuning

Converts trajectory data to the format expected by Unsloth/Qwen3-VL:

{
    "messages": [
        {"role": "user", "content": [
            {"type": "text", "text": "instruction"},
            {"type": "image", "image": PIL_Image}
        ]},
        {"role": "assistant", "content": [
            {"type": "text", "text": "trajectory output"}
        ]}
    ]
}

Usage:
    python prepare_vlm_dataset.py --input ./trajectory_data --output ./vlm_dataset
"""

import json
import numpy as np
from pathlib import Path
from PIL import Image
from typing import List, Dict, Tuple, Optional
import argparse
from datasets import Dataset, DatasetDict
import os
from reasoning_generator import generate_reasoning
from multiprocessing import Pool, cpu_count


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def load_sample(images_dir: Path, labels_dir: Path, sample_id: str) -> Tuple[Optional[Image.Image], Optional[dict]]:
    """
    Load a single image and its corresponding label.
    
    Args:
        images_dir: Directory containing images
        labels_dir: Directory containing label JSON files
        sample_id: ID of the sample (filename without extension)
    
    Returns:
        Tuple of (PIL Image, label dict) or (None, None) if loading fails
    """
    try:
        image_path = images_dir / f'{sample_id}.jpg'
        label_path = labels_dir / f'{sample_id}.json'
        
        if not image_path.exists() or not label_path.exists():
            return None, None
        
        image = Image.open(image_path)
        
        with open(label_path, 'r') as f:
            label = json.load(f)
        
        return image, label
    
    except Exception as e:
        print(f"Error loading sample {sample_id}: {e}")
        return None, None


# =============================================================================
# INSTRUCTION TEMPLATES
# =============================================================================

# Different instruction variations for training robustness
INSTRUCTION_TEMPLATES = [
    "Predict the trajectory waypoints for this driving scene as [[x, y, distance], ...] where x,y are pixel coordinates and distance is in meters.",
    
    "Given this front camera view, output the planned driving trajectory as pixel coordinates with distances: [[x, y, d], ...]",
    
    "Analyze this driving image and predict waypoints along the road. Format: [[pixel_x, pixel_y, distance_m], ...]",
    
    "What is the trajectory path for this driving scene? Output as [[x, y, distance], ...] in pixel coordinates.",
    
    "Predict the driving path waypoints from this camera image. Return [[x_pixel, y_pixel, dist_meters], ...]",
]

# Simple template for inference
DEFAULT_INSTRUCTION = "Predict the trajectory waypoints for this driving scene as [[x, y, distance], ...] where x,y are pixel coordinates and distance is in meters."




def create_vlm_sample(image: Image.Image,
                       label: dict,
                       instruction: Optional[str] = None,
                       include_reasoning: bool = False) -> dict:
    """
    Create a VLM training sample in simple flat format.

    Args:
        image: PIL Image
        label: Dictionary with trajectory data
        instruction: Base instruction text
        include_reasoning: If True, include reasoning field

    Returns:
        Simple dict with separate fields for each component.
    """
    # Use provided instruction or random from templates
    if instruction is None:
        import random
        instruction = random.choice(INSTRUCTION_TEMPLATES)

    # Get navigation token
    nav_token = label.get('navigation_token', 'lane_keeping')

    # Get ego stats (handle both string and dict)
    ego_stats_str = label.get('ego_stats', '{}')

    # Get trajectory (already mapped to image resolution in traj_planner)
    trajectory = label.get('trajectory', label.get('trajectory_2d', []))

    # Build sample with separate fields
    sample = {
        'image': image,
        'navigation': nav_token,
        'ego_state': ego_stats_str,
        'instruction': instruction,
        'trajectory': json.dumps(trajectory, ensure_ascii=False),
    }

    # Add reasoning if requested
    if include_reasoning:
        vehicle_state = label.get('vehicle_state', {})
        waypoints_3d = label.get('trajectory_3d', [])
        reasoning = generate_reasoning(nav_token, vehicle_state, waypoints_3d)
        sample['reasoning'] = reasoning

    return sample


# =============================================================================
# DATASET CREATION
# =============================================================================

def prepare_dataset(input_dir: str,
                    output_dir: str = None,
                    train_ratio: float = 0.9,
                    use_instruction_variations: bool = True,
                    max_samples: int = None,
                    include_reasoning: bool = False,
                    push_dataset: str = None) -> DatasetDict:
    """
    Prepare dataset for VLM fine-tuning.

    Args:
        input_dir: Directory with trajectory data (images/, labels/, raw/)
        output_dir: Optional output directory to save processed dataset
        train_ratio: Train/val split ratio
        use_instruction_variations: Use different instruction templates
        max_samples: Maximum samples to process (None = all)
        include_reasoning: If True, include reasoning field
        push_dataset: If specified, push the dataset to HuggingFace Hub with this repo name

    Returns:
        DatasetDict with 'train' and 'validation' splits

    Dataset format (simple flat structure):
        - image: PIL Image
        - navigation: str (e.g., "lane_keeping", "turn_left")
        - ego_state: str (JSON string of vehicle state)
        - instruction: str
        - reasoning: str (only if include_reasoning=True)
        - trajectory: str (JSON string of trajectory points)
        - sample_id: str
    """
    input_path = Path(input_dir)
    images_dir = input_path / 'images'
    labels_dir = input_path / 'labels'

    # Get all sample IDs
    sample_ids = []
    for label_file in sorted(labels_dir.glob('*.json')):
        sample_ids.append(label_file.stem)

    if max_samples:
        sample_ids = sample_ids[:max_samples]

    print(f"Found {len(sample_ids)} samples")

    # Process samples
    samples = []
    skipped = 0

    for i, sample_id in enumerate(sample_ids):
        if i % 500 == 0:
            print(f"Processing {i}/{len(sample_ids)}...")

        image, label = load_sample(images_dir, labels_dir, sample_id)

        if image is None or label is None:
            skipped += 1
            continue

        # Select instruction
        if use_instruction_variations:
            import random
            instruction = random.choice(INSTRUCTION_TEMPLATES)
        else:
            instruction = DEFAULT_INSTRUCTION

        # Create sample
        sample = create_vlm_sample(
            image,
            label,
            instruction=instruction,
            include_reasoning=include_reasoning
        )
        sample['sample_id'] = sample_id
        samples.append(sample)

    print(f"Processed {len(samples)} samples, skipped {skipped}")

    if len(samples) == 0:
        raise ValueError("No samples were processed successfully")

    # Split into train/val
    import random
    random.shuffle(samples)
    split_idx = int(len(samples) * train_ratio)
    train_samples = samples[:split_idx]
    val_samples = samples[split_idx:]

    print(f"Train: {len(train_samples)}, Validation: {len(val_samples)}")

    # Create HuggingFace datasets
    train_dataset = Dataset.from_list(train_samples)
    val_dataset = Dataset.from_list(val_samples)

    dataset_dict = DatasetDict({
        'train': train_dataset,
        'validation': val_dataset
    })

    # Save if output directory specified
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save as HuggingFace dataset
        hf_path = output_path / 'hf_dataset'
        dataset_dict.save_to_disk(str(hf_path))
        print(f"Saved HuggingFace dataset to {hf_path}")

    # Preview dataset
    preview_dataset(dataset_dict, num_samples=2)

    # Push to HuggingFace Hub if requested
    if push_dataset:
        print(f"Pushing dataset to HuggingFace Hub...")
        dataset_dict.push_to_hub(push_dataset)
        print(f"Dataset pushed to {push_dataset}")

    return dataset_dict

# =============================================================================
# CONVERSION FUNCTION FOR TRAINING
# =============================================================================

def convert_to_vlm_format(sample: dict) -> dict:
    """
    Convert simple flat sample to VLM conversational format for training.

    Use this in your training script's map function:

        dataset = dataset.map(convert_to_vlm_format)

    Input format (flat):
        - image, navigation, ego_state, instruction, reasoning (optional), trajectory

    Output format (conversational):
        - messages: [{role: user, content: [...]}, {role: assistant, content: [...]}]
    """
    # Build user content
    user_content = [
        {"type": "image", "image": sample['image']},
        {"type": "text", "text": f"Navigation: {sample['navigation']}"},
        {"type": "text", "text": f"Ego state: {sample['ego_state']}"},
        {"type": "text", "text": sample['instruction']},
    ]

    # Build assistant content
    assistant_content = []
    if 'reasoning' in sample and sample['reasoning']:
        assistant_content.append({"type": "text", "text": f"Reasoning: {sample['reasoning']}"})
    assistant_content.append({"type": "text", "text": f"Trajectory: {sample['trajectory']}"})

    return {
        "messages": [
            {"role": "user", "content": user_content},
            {"role": "assistant", "content": assistant_content}
        ]
    }


def get_training_prompt(instruction: str = None) -> str:
    """Get the prompt for inference."""
    return instruction or DEFAULT_INSTRUCTION


def preview_dataset(dataset_dict: DatasetDict, num_samples: int = 2):
    """
    Preview samples from the dataset to verify format.

    Args:
        dataset_dict: The dataset to preview
        num_samples: Number of samples to show from each split
    """
    print("\n" + "=" * 70)
    print("DATASET PREVIEW")
    print("=" * 70)

    for split_name, dataset in dataset_dict.items():
        print(f"\n{'─' * 70}")
        print(f"Split: {split_name} ({len(dataset)} samples)")
        print(f"{'─' * 70}")

        for i in range(min(num_samples, len(dataset))):
            sample = dataset[i]
            print(f"\n┌── Sample {i + 1} (ID: {sample.get('sample_id', 'N/A')}) ──┐")

            # Image with dimensions
            img = sample.get('image')
            if img:
                if hasattr(img, 'size'):
                    w, h = img.size
                    print(f"  image:       {w}x{h}")
                else:
                    print(f"  image:       PIL Image")

            # Navigation
            print(f"  navigation:  {sample.get('navigation', 'N/A')}")

            # Ego state (truncate if long)
            ego_state = sample.get('ego_state', '{}')
            if len(ego_state) > 60:
                ego_state = ego_state[:60] + "..."
            print(f"  ego_state:   {ego_state}")

            # Instruction (truncate if long)
            instruction = sample.get('instruction', '')
            if len(instruction) > 60:
                instruction = instruction[:60] + "..."
            print(f"  instruction: {instruction}")

            # Reasoning (if present)
            if 'reasoning' in sample and sample['reasoning']:
                reasoning = sample['reasoning']
                if len(reasoning) > 60:
                    reasoning = reasoning[:60] + "..."
                print(f"  reasoning:   {reasoning}")

            # Trajectory (truncate if long)
            trajectory = sample.get('trajectory', '[]')
            if len(trajectory) > 60:
                trajectory = trajectory[:60] + "..."
            print(f"  trajectory:  {trajectory}")

            print(f"└{'─' * 50}┘")

    print("\n" + "=" * 70)
    print("END PREVIEW")
    print("=" * 70 + "\n")



# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Prepare dataset for VLM fine-tuning')
    parser.add_argument('--input', type=str, default='./trajectory_data',
                        help='Input trajectory data directory')
    parser.add_argument('--output', type=str, default='./vlm_dataset',
                        help='Output directory for processed dataset')
    parser.add_argument('--train-ratio', type=float, default=0.9,
                        help='Train split ratio (default: 0.9)')
    parser.add_argument('--max-samples', type=int, default=None,
                        help='Maximum samples to process')
    parser.add_argument('--with-reasoning', action='store_true',
                        help='Include reasoning field in dataset')
    parser.add_argument('--push-to-hub', type=str, default=None,
                        help='Push the dataset to HuggingFace Hub with this repo name (e.g., username/dataset-name)')

    args = parser.parse_args()

    prepare_dataset(
        input_dir=args.input,
        output_dir=args.output,
        train_ratio=args.train_ratio,
        max_samples=args.max_samples,
        include_reasoning=args.with_reasoning,
        push_dataset=args.push_to_hub
    )
