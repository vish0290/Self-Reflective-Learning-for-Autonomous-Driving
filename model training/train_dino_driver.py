#!/usr/bin/env python3
"""
Training script for DINOv2 Vision-Action Model.

Loads dataset collected by record_dataset.py (JPEG + seg/depth .npy + label JSON),
trains trajectory prediction head with auxiliary seg/depth losses.

Usage:
    python "model training/train_dino_driver.py" --data ./dataset --epochs 30
    python "model training/train_dino_driver.py" --data ./dataset --epochs 50 --batch-size 32 --wandb
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader, random_split
from torchvision import transforms
from PIL import Image

# Add core/ to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'core'))
from dino_driver import (
    DINODriver, compute_loss, NAV_COMMANDS,
    NUM_WAYPOINTS, IMG_W, IMG_H,
)


# =============================================================================
# DATASET
# =============================================================================

class DINODriverDataset(Dataset):
    """
    Loads frames from record_dataset.py output.

    Expected directory structure:
        data_dir/
            images/     frame_000000.jpg, ...
            labels/     frame_000000.json, ...
            raw/        frame_000000_seg.npy, frame_000000_depth.npy (optional)
    """

    def __init__(self, data_dir: str, use_aux: bool = True):
        self.data_dir = Path(data_dir)
        self.images_dir = self.data_dir / 'images'
        self.labels_dir = self.data_dir / 'labels'
        self.raw_dir = self.data_dir / 'raw'
        self.use_aux = use_aux

        # Find all label files
        self.samples = sorted(self.labels_dir.glob('*.json'))
        if not self.samples:
            raise ValueError(f"No label files found in {self.labels_dir}")

        # Filter to only samples that have trajectory_2d
        valid = []
        for label_path in self.samples:
            with open(label_path) as f:
                label = json.load(f)
            if 'trajectory_2d' in label and len(label['trajectory_2d']) >= NUM_WAYPOINTS:
                valid.append(label_path)
        self.samples = valid
        print(f"Dataset: {len(self.samples)} valid samples from {data_dir}")

        self.to_tensor = transforms.ToTensor()  # PIL -> (3, H, W) float [0,1]

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        label_path = self.samples[idx]
        with open(label_path) as f:
            label = json.load(f)

        frame_id = label_path.stem

        # Load RGB image
        img_file = label.get('image_file', f'{frame_id}.jpg')
        img_path = self.images_dir / img_file
        if not img_path.exists():
            img_path = self.images_dir / f'{frame_id}.jpg'
        image = Image.open(img_path).convert('RGB')
        image = self.to_tensor(image)  # (3, H, W) float [0, 1]

        # Speed (km/h)
        speed = torch.tensor([label.get('speed_kmh', 0.0)], dtype=torch.float32)

        # Navigation command
        nav_str = label.get('train_nav_command', 'follow_lane')
        nav_cmd = torch.tensor(NAV_COMMANDS.get(nav_str, 0), dtype=torch.long)

        # Trajectory 2D (normalized to [0,1])
        traj_2d = label['trajectory_2d'][:NUM_WAYPOINTS]
        # Pad if fewer than NUM_WAYPOINTS
        while len(traj_2d) < NUM_WAYPOINTS:
            traj_2d.append(traj_2d[-1])
        traj = torch.tensor(traj_2d, dtype=torch.float32)
        traj[:, 0] /= IMG_W  # normalize u
        traj[:, 1] /= IMG_H  # normalize v

        sample = {
            'image': image,
            'speed': speed,
            'nav_cmd': nav_cmd,
            'traj': traj,
        }

        # Auxiliary data (optional)
        if self.use_aux:
            seg_path = self.raw_dir / f'{frame_id}_seg.npy'
            if seg_path.exists():
                seg = np.load(seg_path).astype(np.int64)
                sample['seg'] = torch.from_numpy(seg)

            depth_path = self.raw_dir / f'{frame_id}_depth.npy'
            if depth_path.exists():
                depth = np.load(depth_path).astype(np.float32)
                sample['depth'] = torch.from_numpy(depth)

        return sample


def collate_fn(batch):
    """Custom collate that handles optional seg/depth."""
    result = {
        'image': torch.stack([s['image'] for s in batch]),
        'speed': torch.stack([s['speed'] for s in batch]),
        'nav_cmd': torch.stack([s['nav_cmd'] for s in batch]),
        'traj': torch.stack([s['traj'] for s in batch]),
    }
    if 'seg' in batch[0]:
        result['seg'] = torch.stack([s['seg'] for s in batch])
    if 'depth' in batch[0]:
        result['depth'] = torch.stack([s['depth'] for s in batch])
    return result


# =============================================================================
# TRAINING
# =============================================================================

def train(args):
    device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    print(f"Device: {device}")

    # W&B
    wandb_run = None
    if args.wandb:
        import wandb
        wandb_run = wandb.init(
            project=args.wandb_project,
            name=args.wandb_name or f"dino-driver-{time.strftime('%Y%m%d_%H%M')}",
            config=vars(args),
        )

    # Dataset
    dataset = DINODriverDataset(args.data, use_aux=args.aux)
    n_val = max(1, int(len(dataset) * 0.1))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(dataset, [n_train, n_val],
                                     generator=torch.Generator().manual_seed(42))
    print(f"Train: {n_train}, Val: {n_val}")

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,
                              num_workers=args.workers, collate_fn=collate_fn,
                              pin_memory=True, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False,
                            num_workers=args.workers, collate_fn=collate_fn,
                            pin_memory=True)

    # Model
    model = DINODriver().to(device)
    if args.resume:
        model.load_heads(args.resume, map_location=device)
        print(f"Resumed from {args.resume}")

    trainable = sum(p.numel() for p in model.trainable_params())
    print(f"Trainable params: {trainable:,} ({trainable/1e3:.1f}K)")

    optimizer = torch.optim.AdamW(model.trainable_params(),
                                   lr=args.lr, weight_decay=args.wd)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs * len(train_loader))

    # Output dir
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    best_val_traj = float('inf')

    for epoch in range(args.epochs):
        # Train
        model.train()
        # Keep backbone frozen
        model.backbone.eval()

        epoch_losses = {'loss_total': 0, 'loss_traj': 0, 'loss_seg': 0, 'loss_depth': 0}
        n_batches = 0

        for batch in train_loader:
            images = batch['image'].to(device)
            speed = batch['speed'].to(device)
            nav_cmd = batch['nav_cmd'].to(device)
            traj_gt = batch['traj'].to(device)

            seg_gt = batch.get('seg')
            depth_gt = batch.get('depth')

            if args.aux and seg_gt is not None and depth_gt is not None:
                seg_gt = seg_gt.to(device)
                depth_gt = depth_gt.to(device)
                traj_pred, seg_pred, depth_pred = model(images, speed, nav_cmd, return_aux=True)
                loss, loss_dict = compute_loss(traj_pred, traj_gt, seg_pred, seg_gt, depth_pred, depth_gt)
            else:
                traj_pred = model(images, speed, nav_cmd, return_aux=False)
                loss, loss_dict = compute_loss(traj_pred, traj_gt)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.trainable_params(), 1.0)
            optimizer.step()
            scheduler.step()

            for k, v in loss_dict.items():
                epoch_losses[k] = epoch_losses.get(k, 0) + v
            n_batches += 1

        # Average training losses
        for k in epoch_losses:
            epoch_losses[k] /= max(n_batches, 1)

        # Validate
        model.eval()
        val_losses = {'loss_total': 0, 'loss_traj': 0}
        n_val_batches = 0
        with torch.no_grad():
            for batch in val_loader:
                images = batch['image'].to(device)
                speed = batch['speed'].to(device)
                nav_cmd = batch['nav_cmd'].to(device)
                traj_gt = batch['traj'].to(device)
                traj_pred = model(images, speed, nav_cmd, return_aux=False)
                _, ld = compute_loss(traj_pred, traj_gt)
                for k, v in ld.items():
                    val_losses[k] = val_losses.get(k, 0) + v
                n_val_batches += 1

        for k in val_losses:
            val_losses[k] /= max(n_val_batches, 1)

        lr_now = optimizer.param_groups[0]['lr']
        print(f"[Epoch {epoch+1}/{args.epochs}] "
              f"train_traj={epoch_losses['loss_traj']:.5f} "
              f"val_traj={val_losses['loss_traj']:.5f} "
              f"lr={lr_now:.2e}")

        if wandb_run:
            import wandb
            log = {f'train/{k}': v for k, v in epoch_losses.items()}
            log.update({f'val/{k}': v for k, v in val_losses.items()})
            log['lr'] = lr_now
            wandb.log(log, step=epoch)

        # Save best
        if val_losses['loss_traj'] < best_val_traj:
            best_val_traj = val_losses['loss_traj']
            save_path = out_dir / 'best_heads.pt'
            model.save_heads(save_path)
            print(f"  -> Saved best model (val_traj={best_val_traj:.5f})")

        # Periodic checkpoint
        if (epoch + 1) % 10 == 0:
            model.save_heads(out_dir / f'heads_epoch_{epoch+1}.pt')

    # Final save
    model.save_heads(out_dir / 'final_heads.pt')
    print(f"\nTraining complete. Best val_traj={best_val_traj:.5f}")
    print(f"Checkpoints in {out_dir}")

    if wandb_run:
        import wandb
        wandb.finish()


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train DINOv2 Driver')
    parser.add_argument('--data', type=str, required=True, help='Dataset directory')
    parser.add_argument('--output', type=str, default='./dino_driver_checkpoints',
                        help='Output directory for checkpoints')
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--lr', type=float, default=3e-4)
    parser.add_argument('--wd', type=float, default=1e-4, help='Weight decay')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--no-aux', dest='aux', action='store_false',
                        help='Disable auxiliary seg/depth losses')
    parser.add_argument('--resume', type=str, help='Resume from heads checkpoint')
    parser.add_argument('--wandb', action='store_true', help='Enable W&B logging')
    parser.add_argument('--wandb-project', type=str, default='dino-driver')
    parser.add_argument('--wandb-name', type=str, default=None)
    args = parser.parse_args()
    train(args)
