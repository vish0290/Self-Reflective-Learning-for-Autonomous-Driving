#!/usr/bin/env python3
"""
DINOv2 Vision-Action Model for Autonomous Driving

Frozen DINOv2 ViT-B/14 backbone with lightweight trainable heads:
- Trajectory head: predicts 10 future waypoints as [u,v] pixel coords
- Segmentation head: per-patch semantic segmentation (training only)
- Depth head: per-patch depth estimation (training only)

~350K trainable params. Inference ~20-50ms/frame.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from torchvision import transforms


# Navigation command vocabulary (matches control_navigation.py)
NAV_COMMANDS = {
    'follow_lane': 0,
    'slight_left': 1,
    'slight_right': 2,
    'turn_left': 3,
    'turn_right': 4,
    'sharp_turn_left': 5,
    'sharp_turn_right': 6,
}
NAV_COMMANDS_INV = {v: k for k, v in NAV_COMMANDS.items()}
NUM_NAV_COMMANDS = len(NAV_COMMANDS)

# Image dimensions
IMG_W, IMG_H = 640, 480
DINO_SIZE = 448
NUM_WAYPOINTS = 10
NUM_SEG_CLASSES = 23  # CARLA semantic classes


def get_dino_transform():
    """DINOv2 input transform: resize + center crop + ImageNet normalize."""
    return transforms.Compose([
        transforms.Resize(DINO_SIZE, antialias=True),
        transforms.CenterCrop(DINO_SIZE),
        transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])


class DINODriver(nn.Module):
    """
    DINOv2-based trajectory prediction model.

    Architecture:
        DINOv2 ViT-B/14 (frozen) -> CLS token + patch tokens
        CLS (768) -> proj (256) + speed (64) + nav (64) = shared state (384)
        Shared state -> MLP -> 10 waypoints [u, v]
        Patch tokens -> seg head (23 classes) + depth head (1) [training only]
    """

    def __init__(self):
        super().__init__()

        # Frozen DINOv2 backbone
        self.backbone = torch.hub.load('facebookresearch/dinov2', 'dinov2_vitb14')
        for p in self.backbone.parameters():
            p.requires_grad = False
        self.backbone.eval()

        # Trainable heads
        self.cls_proj = nn.Linear(768, 256)
        self.speed_proj = nn.Linear(1, 64)
        self.nav_embed = nn.Embedding(NUM_NAV_COMMANDS, 64)

        self.traj_head = nn.Sequential(
            nn.Linear(384, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, NUM_WAYPOINTS * 2),
            nn.Sigmoid(),  # normalize to [0, 1]
        )

        # Auxiliary heads (training only)
        self.seg_head = nn.Linear(768, NUM_SEG_CLASSES)
        self.depth_head = nn.Linear(768, 1)

        self.dino_transform = get_dino_transform()

    def forward(self, images, speed, nav_cmd, return_aux=False):
        """
        Args:
            images: (B, 3, H, W) tensor, already normalized uint8->float [0,1]
            speed: (B, 1) tensor, speed in km/h (raw, not normalized)
            nav_cmd: (B,) tensor of int nav command indices
            return_aux: if True, also return seg and depth predictions

        Returns:
            traj: (B, NUM_WAYPOINTS, 2) pixel coords in [0,1] normalized space
            seg: (B, NUM_SEG_CLASSES, 32, 32) if return_aux (patch-resolution)
            depth: (B, 1, 32, 32) if return_aux
        """
        # Transform for DINOv2
        x = self.dino_transform(images)

        # Forward through frozen backbone
        with torch.no_grad():
            out = self.backbone.forward_features(x)
            cls_token = out['x_norm_clstoken']   # (B, 768)
            patch_tokens = out['x_norm_patchtokens']  # (B, 1024, 768)

        # Build shared state
        cls_feat = self.cls_proj(cls_token)  # (B, 256)
        speed_feat = self.speed_proj(speed)  # (B, 64)
        nav_feat = self.nav_embed(nav_cmd)   # (B, 64)
        shared = torch.cat([cls_feat, speed_feat, nav_feat], dim=1)  # (B, 384)

        # Trajectory prediction
        traj_flat = self.traj_head(shared)  # (B, 20)
        traj = traj_flat.view(-1, NUM_WAYPOINTS, 2)  # (B, 10, 2)

        if not return_aux:
            return traj

        # Auxiliary heads on patch tokens
        seg_logits = self.seg_head(patch_tokens)  # (B, 1024, 23)
        seg_logits = seg_logits.permute(0, 2, 1).view(-1, NUM_SEG_CLASSES, 32, 32)

        depth_pred = self.depth_head(patch_tokens)  # (B, 1024, 1)
        depth_pred = depth_pred.permute(0, 2, 1).view(-1, 1, 32, 32)

        return traj, seg_logits, depth_pred

    def predict_pixels(self, images, speed, nav_cmd):
        """
        Predict waypoints in original 640x480 pixel coordinates.

        Args:
            images: (B, 3, H, W) float tensor
            speed: (B, 1)
            nav_cmd: (B,)

        Returns:
            (B, 10, 2) pixel coords in [0, IMG_W] x [0, IMG_H]
        """
        traj = self.forward(images, speed, nav_cmd, return_aux=False)
        # Scale from [0,1] to pixel space
        traj[:, :, 0] *= IMG_W
        traj[:, :, 1] *= IMG_H
        return traj

    def trainable_params(self):
        """Return only trainable parameters (for optimizer)."""
        return [p for p in self.parameters() if p.requires_grad]

    def save_heads(self, path):
        """Save only trainable head weights (not DINOv2)."""
        state = {
            'cls_proj': self.cls_proj.state_dict(),
            'speed_proj': self.speed_proj.state_dict(),
            'nav_embed': self.nav_embed.state_dict(),
            'traj_head': self.traj_head.state_dict(),
            'seg_head': self.seg_head.state_dict(),
            'depth_head': self.depth_head.state_dict(),
        }
        torch.save(state, path)

    def load_heads(self, path, map_location='cpu'):
        """Load trainable head weights."""
        state = torch.load(path, map_location=map_location, weights_only=True)
        self.cls_proj.load_state_dict(state['cls_proj'])
        self.speed_proj.load_state_dict(state['speed_proj'])
        self.nav_embed.load_state_dict(state['nav_embed'])
        self.traj_head.load_state_dict(state['traj_head'])
        self.seg_head.load_state_dict(state['seg_head'])
        self.depth_head.load_state_dict(state['depth_head'])


def compute_loss(traj_pred, traj_gt, seg_pred=None, seg_gt=None,
                 depth_pred=None, depth_gt=None):
    """
    Multi-task loss: trajectory + optional auxiliary losses.

    Args:
        traj_pred: (B, 10, 2) predicted normalized waypoints
        traj_gt: (B, 10, 2) ground truth normalized waypoints
        seg_pred: (B, 23, 32, 32) segmentation logits (optional)
        seg_gt: (B, H, W) segmentation labels uint8 (optional)
        depth_pred: (B, 1, 32, 32) depth prediction (optional)
        depth_gt: (B, H, W) depth ground truth float32 (optional)

    Returns:
        (total_loss, dict of individual losses)
    """
    loss_traj = F.mse_loss(traj_pred, traj_gt)
    losses = {'loss_traj': loss_traj.item()}
    total = loss_traj

    if seg_pred is not None and seg_gt is not None:
        # Downsample ground truth to patch resolution
        seg_gt_down = F.interpolate(
            seg_gt.unsqueeze(1).float(), size=(32, 32), mode='nearest'
        ).squeeze(1).long()
        loss_seg = F.cross_entropy(seg_pred, seg_gt_down)
        total = total + 0.5 * loss_seg
        losses['loss_seg'] = loss_seg.item()

    if depth_pred is not None and depth_gt is not None:
        # Downsample ground truth to patch resolution
        depth_gt_down = F.interpolate(
            depth_gt.unsqueeze(1), size=(32, 32), mode='bilinear',
            align_corners=False,
        )
        loss_depth = F.smooth_l1_loss(depth_pred, depth_gt_down)
        total = total + 0.5 * loss_depth
        losses['loss_depth'] = loss_depth.item()

    losses['loss_total'] = total.item()
    return total, losses


# =============================================================================
# SANITY CHECK
# =============================================================================

if __name__ == '__main__':
    print("DINOv2 Driver - Sanity Check")
    print("=" * 50)

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = DINODriver().to(device)

    # Count params
    total = sum(p.numel() for p in model.parameters())
    trainable = sum(p.numel() for p in model.trainable_params())
    print(f"Total params: {total:,} ({total/1e6:.1f}M)")
    print(f"Trainable:    {trainable:,} ({trainable/1e3:.1f}K)")

    # Random forward pass
    B = 2
    images = torch.randn(B, 3, IMG_H, IMG_W).to(device)
    speed = torch.tensor([[25.0], [30.0]]).to(device)
    nav = torch.tensor([0, 3]).to(device)

    traj, seg, depth = model(images, speed, nav, return_aux=True)
    print(f"\nForward pass shapes:")
    print(f"  traj:  {traj.shape}")   # (2, 10, 2)
    print(f"  seg:   {seg.shape}")    # (2, 23, 32, 32)
    print(f"  depth: {depth.shape}")  # (2, 1, 32, 32)

    # Test loss
    traj_gt = torch.rand(B, NUM_WAYPOINTS, 2).to(device)
    seg_gt = torch.randint(0, NUM_SEG_CLASSES, (B, IMG_H, IMG_W)).to(device)
    depth_gt = torch.rand(B, IMG_H, IMG_W).to(device)

    loss, loss_dict = compute_loss(traj, traj_gt, seg, seg_gt, depth, depth_gt)
    print(f"\nLosses: {loss_dict}")

    # Test pixel prediction
    pixels = model.predict_pixels(images, speed, nav)
    print(f"\nPixel waypoints: {pixels.shape}, range u=[{pixels[:,:,0].min():.0f}, {pixels[:,:,0].max():.0f}], v=[{pixels[:,:,1].min():.0f}, {pixels[:,:,1].max():.0f}]")

    print("\nSanity check passed.")
