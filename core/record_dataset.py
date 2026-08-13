#!/usr/bin/env python3
"""
Route-Based Autonomous Data Collection System
==============================================

Option A: Direct Control Output (discrete steering angles)

Training output format:
    {"steer": -15, "throttle": 0.4, "brake": 0.0}

FIXES from original:
1. Discrete steering angles (degrees) instead of continuous floats
2. Discrete throttle/brake bins for clean model output  
3. Route-based navigation detection using road_id/lane_id/yaw changes
4. Lowered min_speed to 3 km/h to capture turn frames
5. Navigation detector resets per route
6. Steering distribution logging for data quality checks
7. Proper absolute waypoint distances (not accumulated)
8. FIXED: CARLA yaw convention (positive yaw = LEFT, negative = RIGHT)
9. FIXED: Duplicate transition detection at road_id boundaries

Usage:
    python record_dataset.py --routes ./routes
    python record_dataset.py --routes ./routes --loops 3
    python record_dataset.py --route ./routes/route_city_left_01.json --loops 5
    python record_dataset.py --routes ./routes --no-display --target-speed 25
"""

import carla
import numpy as np
import math
import time
import argparse
import signal
import sys
import json
import os
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from collections import Counter
from threading import Thread, Event
from queue import Queue, Full
import traceback

# Optional imports
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not available - running headless")

try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# Import your existing trajectory modules
# Adjust this path to match your project structure
try:
    from traj_planner import (
        CameraConfig as TrajCameraConfig,
        TrajectoryEncoder,
        TrajectoryDecoder,
        Waypoint3D,
        Waypoint2D
    )
    TRAJ_AVAILABLE = True
except ImportError:
    TRAJ_AVAILABLE = False
    print("Warning: traj_planner not found - trajectory encoding disabled")

# Import unified navigation
from control_navigation import SteeringNavigationAnalyzer, get_nav_prompt


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

# Waypoint distances (meters) - ABSOLUTE from vehicle, not accumulated
WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]

# Recording thresholds
MIN_SPEED_KMH = 3.0       # Capture turn frames (was 15, too high)
MIN_WAYPOINTS = 3          # Minimum visible waypoints (was 5, too strict)
ROUTE_END_THRESHOLD = 5.0  # Meters to last checkpoint = route complete


# =============================================================================
# CAMERA CONFIG
# =============================================================================

@dataclass
class CameraConfig:
    """Camera configuration matching your CARLA setup."""
    width: int = WINDOW_WIDTH
    height: int = WINDOW_HEIGHT
    fov: float = 90.0
    x: float = 2.0
    y: float = 0.0
    z: float = 1.8
    pitch: float = -15.0
    yaw: float = 0.0
    roll: float = 0.0

CAMERA_CONFIG = CameraConfig()


# =============================================================================
# DISCRETE CONTROL ENCODER
# =============================================================================

class DiscreteControlEncoder:
    """
    Converts continuous CARLA controls to discrete training labels.
    
    Steering: continuous float → nearest degree angle
    Throttle: continuous float → nearest bin
    Brake:    continuous float → nearest bin
    
    This makes the VLM output a classification problem instead of regression.
    """

    # Steering angle bins (degrees)
    # Fine-grained near center for lane keeping, wider for turns
    STEER_ANGLES = [-60, -45, -30, -20, -15, -10, -5, -2, 0, 2, 5, 10, 15, 20, 30, 45, 60]

    # CARLA max steering angle (degrees) - maps steer=1.0 to this
    MAX_STEER_ANGLE = 70.0

    # Throttle bins
    THROTTLE_BINS = [0.0, 0.2, 0.4, 0.6, 0.8]

    # Brake bins
    BRAKE_BINS = [0.0, 0.3, 0.6, 1.0]

    @classmethod
    def encode_steer(cls, steer_value: float) -> int:
        raw_angle = steer_value * cls.MAX_STEER_ANGLE
        nearest = min(cls.STEER_ANGLES, key=lambda a: abs(a - raw_angle))
        return nearest

    @classmethod
    def encode_throttle(cls, value: float) -> float:
        return min(cls.THROTTLE_BINS, key=lambda x: abs(x - value))

    @classmethod
    def encode_brake(cls, value: float) -> float:
        return min(cls.BRAKE_BINS, key=lambda x: abs(x - value))

    @classmethod
    def encode(cls, control) -> dict:
        return {
            "steer": cls.encode_steer(control.steer),
            "throttle": cls.encode_throttle(control.throttle),
            "brake": cls.encode_brake(control.brake)
        }

    @staticmethod
    def decode_steer(angle_degrees: int, max_steer_angle: float = 70.0) -> float:
        return np.clip(angle_degrees / max_steer_angle, -1.0, 1.0)

    @staticmethod
    def train_nav(steer_angle: int) -> str:
        """Original working logic - don't change unless testing shows better results."""
        if steer_angle < -5:
           return "turn_left"
        elif steer_angle < 0 and steer_angle >= -5:
            return "slight_left"
        elif steer_angle > 5:
            return "turn_right"
        elif steer_angle > 0 and steer_angle <= 5:
            return "slight_right"
        else:
            return "follow_lane"

# =============================================================================
# ROUTE DATA STRUCTURES
# =============================================================================

@dataclass
class RouteCheckpoint:
    """Single checkpoint in a route."""
    x: float
    y: float
    z: float
    yaw: float
    road_id: int
    lane_id: int


# =============================================================================
# ROUTE LOADER
# =============================================================================

class RouteLoader:
    """Loads and manages route checkpoint files."""

    def __init__(self, routes_dir: str = None, route_file: str = None):
        self.routes = []
        if route_file:
            self.load_route(route_file)
        elif routes_dir:
            self.load_routes_from_directory(routes_dir)
        else:
            raise ValueError("Must provide either routes_dir or route_file")

    def load_route(self, filepath: str) -> Dict:
        with open(filepath, 'r') as f:
            route_data = json.load(f)

        route_name = Path(filepath).stem
        checkpoint_data = None
        total_distance = 0

        if 'checkpoints' in route_data:
            checkpoint_data = route_data['checkpoints']
            total_distance = route_data.get('total_distance', 0)
        elif 'scenarios' in route_data and 'custom' in route_data['scenarios']:
            if len(route_data['scenarios']['custom']) > 0:
                custom_route = route_data['scenarios']['custom'][0]
                checkpoint_data = custom_route.get('checkpoints', [])
                if checkpoint_data and len(checkpoint_data) > 1:
                    total_distance = 0
                    for i in range(1, len(checkpoint_data)):
                        dx = checkpoint_data[i]['x'] - checkpoint_data[i - 1]['x']
                        dy = checkpoint_data[i]['y'] - checkpoint_data[i - 1]['y']
                        total_distance += math.sqrt(dx * dx + dy * dy)

        if not checkpoint_data:
            raise ValueError(f"No checkpoints found in {filepath}")

        checkpoints = []
        for cp in checkpoint_data:
            checkpoints.append(RouteCheckpoint(
                x=cp['x'], y=cp['y'], z=cp['z'],
                yaw=cp.get('yaw', 0.0),
                road_id=cp.get('road_id', 0),
                lane_id=cp.get('lane_id', 0)
            ))

        route = {
            'name': route_name,
            'filepath': filepath,
            'checkpoints': checkpoints,
            'total_distance': total_distance,
        }
        self.routes.append(route)
        return route

    def load_routes_from_directory(self, dirpath: str):
        routes_path = Path(dirpath)
        if not routes_path.exists():
            raise FileNotFoundError(f"Routes directory not found: {dirpath}")

        json_files = sorted(routes_path.glob('*.json'))
        if not json_files:
            raise ValueError(f"No JSON files found in {dirpath}")

        for filepath in json_files:
            try:
                self.load_route(str(filepath))
                print(f"  ✓ Loaded: {filepath.stem}")
            except Exception as e:
                print(f"  ✗ Error loading {filepath.name}: {e}")

        print(f"\nTotal routes loaded: {len(self.routes)}")
        if len(self.routes) == 0:
            raise ValueError(f"Failed to load any routes from {dirpath}")

    def get_route(self, index: int) -> Dict:
        return self.routes[index % len(self.routes)]

    def get_route_count(self) -> int:
        return len(self.routes)


# =============================================================================
# ROUTE NAVIGATION DETECTOR (FIXED)
# =============================================================================

class RouteNavigationDetector:
    """
    Detects ROUTE-LEVEL navigation decisions from checkpoint structure.

    Only fires at decision points where the driver must make a CHOICE:
    - Intersections (road_id changes)
    - Lane merges (lane_id changes)

    Curves and sustained turns are handled by the steering output itself,
    NOT by the navigation command.

    CARLA Coordinate System:
        X = forward
        Y = right  
        Yaw = rotation around Z-axis
        
        IMPORTANT - CARLA yaw convention:
            Positive yaw = counter-clockwise = LEFT when viewed from above
            Negative yaw = clockwise = RIGHT when viewed from above
        
        So:
            yaw_diff > 0 → yaw increased → vehicle turned LEFT
            yaw_diff < 0 → yaw decreased → vehicle turned RIGHT
    """

    def __init__(self, checkpoints: List[RouteCheckpoint],
                 lookahead_meters: float = 40.0,
                 yaw_window: int = 15):
        """
        Args:
            checkpoints: Route checkpoints with road_id, lane_id, yaw
            lookahead_meters: How far ahead to report upcoming decisions
            yaw_window: Checkpoints before/after road_id change to measure
                        accumulated yaw. At ~1m spacing, 15 = 15m window.
        """
        self.checkpoints = checkpoints
        self.lookahead_meters = lookahead_meters
        self.yaw_window = yaw_window
        self.transition_indices = self._find_transitions()

    @staticmethod
    def _normalize_yaw(yaw_diff: float) -> float:
        """Normalize angle difference to [-180, 180]."""
        while yaw_diff > 180:
            yaw_diff -= 360
        while yaw_diff < -180:
            yaw_diff += 360
        return yaw_diff

    def _find_transitions(self) -> List[dict]:
        """
        Find road_id and lane_id changes — actual decision points.
        
        Deduplicates: multiple consecutive checkpoints with road_id changes
        (e.g. short intermediate road segments in intersections) are merged
        into a single transition event.
        """
        transitions = []
        visited = set()  # checkpoint indices already claimed by a transition

        for i in range(1, len(self.checkpoints)):
            if i in visited:
                continue

            prev = self.checkpoints[i - 1]
            curr = self.checkpoints[i]

            # ── Road ID change = intersection / junction ──
            if prev.road_id != curr.road_id:
                # Measure yaw over a wide window AROUND the transition point
                # to capture the full turn, not just one checkpoint step
                win_start = max(0, i - self.yaw_window)
                win_end = min(len(self.checkpoints) - 1, i + self.yaw_window)

                yaw_before = self.checkpoints[win_start].yaw
                yaw_after = self.checkpoints[win_end].yaw
                total_yaw = self._normalize_yaw(yaw_after - yaw_before)

                command = self._classify_turn(total_yaw)

                transitions.append({
                    'index': i,
                    'command': command,
                    'yaw_change': round(total_yaw, 1),
                    'from_road': prev.road_id,
                    'to_road': curr.road_id,
                })

                # Mark nearby checkpoints as visited to prevent duplicate
                # detections from short intermediate road segments
                for j in range(max(0, i - 5), min(len(self.checkpoints), i + 6)):
                    visited.add(j)

            # ── Lane ID change = merge / lane change ──
            elif prev.lane_id != curr.lane_id:
                # CARLA lane_id convention:
                #   Negative values: driving lanes (right side of road)
                #   -1 = rightmost lane, -2 = next left, -3 = further left
                #   So: lane_id increases (less negative) = moving LEFT
                #       lane_id decreases (more negative) = moving RIGHT
                if curr.lane_id > prev.lane_id:
                    command = "merge_left"
                else:
                    command = "merge_right"

                transitions.append({
                    'index': i,
                    'command': command,
                    'from_lane': prev.lane_id,
                    'to_lane': curr.lane_id,
                })
                visited.add(i)

        return transitions

    @staticmethod
    def _classify_turn(total_yaw: float) -> str:
        
       
        abs_yaw = abs(total_yaw)
        print(f"    Detected turn: total_yaw={total_yaw:.1f}° → ", end="")

        if abs_yaw > 15:
            return "go_straight"
        elif abs_yaw < 120:
            if total_yaw < 0:
                return "turn_left"
            else:
                return "turn_right"
        else:
            return "u_turn"

    def get_command(self, vehicle_location, closest_checkpoint_idx: int) -> str:
        """
        Get the next upcoming route decision based on vehicle position.

        Scans transitions ahead of the vehicle's current checkpoint index.
        Returns the first transition within lookahead_meters, or "follow_lane"
        if no decision point is upcoming.
        """
        for transition in self.transition_indices:
            if transition['index'] > closest_checkpoint_idx:
                t_cp = self.checkpoints[transition['index']]
                dist = math.sqrt(
                    (t_cp.x - vehicle_location.x) ** 2 +
                    (t_cp.y - vehicle_location.y) ** 2
                )
                if dist < self.lookahead_meters:
                    return transition['command']
                else:
                    # Next transition is beyond lookahead — nothing upcoming
                    break
        return "follow_lane"

    def get_transitions_summary(self) -> str:
        """Human-readable summary of all detected transitions."""
        if not self.transition_indices:
            return "  No decision points (straight route)"
        lines = []
        for t in self.transition_indices:
            yaw = t.get('yaw_change', None)
            if yaw is not None:
                detail = f"yaw={yaw:>+7.1f}°  road {t['from_road']}→{t['to_road']}"
            else:
                detail = f"lane {t.get('from_lane', '?')}→{t.get('to_lane', '?')}"
            lines.append(
                f"  #{t['index']:>4}: {t['command']:<14} ({detail})"
            )
        return "\n".join(lines)


# =============================================================================
# WAYPOINT EXTRACTOR FROM CHECKPOINTS
# =============================================================================

class CheckpointWaypointExtractor:
    """
    Extracts waypoints at specific ABSOLUTE distances from vehicle.
    """

    def __init__(self, distances: List[float] = None):
        self.distances = distances or WAYPOINT_DISTANCES

    def find_closest_checkpoint(self, checkpoints: List[RouteCheckpoint],
                                vehicle_location) -> int:
        min_dist = float('inf')
        closest_idx = 0
        for i, cp in enumerate(checkpoints):
            dist = math.sqrt(
                (cp.x - vehicle_location.x) ** 2 +
                (cp.y - vehicle_location.y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    def get_waypoints(self, checkpoints: List[RouteCheckpoint],
                      vehicle_location) -> List:
        closest_idx = self.find_closest_checkpoint(checkpoints, vehicle_location)

        cumulative = [0.0]
        for i in range(closest_idx + 1, len(checkpoints)):
            prev_cp = checkpoints[i - 1]
            curr_cp = checkpoints[i]
            dist = math.sqrt(
                (curr_cp.x - prev_cp.x) ** 2 +
                (curr_cp.y - prev_cp.y) ** 2
            )
            cumulative.append(cumulative[-1] + dist)

        waypoints = []
        for target_dist in self.distances:
            cp_idx = None
            for i, cum_dist in enumerate(cumulative):
                if cum_dist >= target_dist:
                    cp_idx = closest_idx + i
                    break
            if cp_idx is None or cp_idx >= len(checkpoints):
                cp_idx = len(checkpoints) - 1
            cp = checkpoints[cp_idx]
            waypoints.append([cp.x, cp.y, cp.z])
        return waypoints


# =============================================================================
# SENSOR DATA
# =============================================================================

class SensorData:
    def __init__(self):
        self.rgb_image = None
        self.seg_image = None     # semantic segmentation (H, W) uint8
        self.depth_image = None   # depth in meters (H, W) float32
        self.timestamp = 0

sensor_data = SensorData()

def process_rgb(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    sensor_data.rgb_image = array[:, :, ::-1].copy()
    sensor_data.timestamp = image.timestamp

def process_seg(image):
    """Process semantic segmentation camera. R channel = class ID."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    sensor_data.seg_image = array[:, :, 2].copy()  # R channel in BGRA

def process_depth(image):
    """Process depth camera. Convert BGRA encoding to meters."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4)).astype(np.float32)
    # CARLA depth encoding: depth = (R + G*256 + B*256*256) / (256^3 - 1) * 1000
    normalized = (array[:, :, 2] + array[:, :, 1] * 256.0 + array[:, :, 0] * 256.0 * 256.0) / (256.0**3 - 1)
    sensor_data.depth_image = (normalized * 1000.0).astype(np.float32)  # meters


# =============================================================================
# ASYNC DATA SAVER
# =============================================================================

class DataSaver:
    def __init__(self, output_dir: str, max_queue_size: int = 500):
        self.output_dir = Path(output_dir)
        self.images_dir = self.output_dir / 'images'
        self.labels_dir = self.output_dir / 'labels'
        self.raw_dir = self.output_dir / 'raw'  # For any additional raw data (e.g. numpy arrays)
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        self.raw_dir.mkdir(parents=True, exist_ok=True)

        self.queue = Queue(maxsize=max_queue_size)
        self.stop_event = Event()
        self.stats = {'queued': 0, 'saved': 0, 'dropped': 0}
        self.steer_distribution = Counter()
        self.nav_distribution = Counter()
        self._thread = Thread(target=self._save_loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.stop_event.set()
        self._thread.join(timeout=30)
        while not self.queue.empty():
            try:
                self._save_sample(self.queue.get_nowait())
            except:
                break

    def queue_sample(self, sample: dict) -> bool:
        try:
            self.queue.put_nowait(sample)
            self.stats['queued'] += 1
            self.steer_distribution[sample.get('control', {}).get('steer', 0)] += 1
            self.nav_distribution[sample.get('navigation', 'unknown')] += 1
            return True
        except Full:
            self.stats['dropped'] += 1
            return False

    def _save_loop(self):
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                sample = self.queue.get(timeout=0.5)
                self._save_sample(sample)
            except:
                continue

    def _save_sample(self, sample: dict):
        try:
            idx = self.stats['saved']
            frame_id = f"frame_{idx:06d}"

            img = sample['rgb_image']
            img_path = self.raw_dir / f"{frame_id}.npy"
            np.save(str(img_path), img)

            # Save segmentation if present
            if 'seg_image' in sample and sample['seg_image'] is not None:
                np.save(str(self.raw_dir / f"{frame_id}_seg.npy"), sample['seg_image'])

            # Save depth if present
            if 'depth_image' in sample and sample['depth_image'] is not None:
                np.save(str(self.raw_dir / f"{frame_id}_depth.npy"), sample['depth_image'])

            label = {k: v for k, v in sample.items()
                     if k not in ('rgb_image', 'seg_image', 'depth_image')}
            label['image_file'] = f"{frame_id}.npy"
            label['frame_index'] = idx

            with open(self.labels_dir / f"{frame_id}.json", 'w') as f:
                json.dump(label, f, indent=None)

            self.stats['saved'] += 1
        except Exception as e:
            print(f"\nError saving sample: {e}")

    def get_stats(self) -> dict:
        return {**self.stats, 'queue_size': self.queue.qsize()}

    def print_steer_distribution(self):
        total = sum(self.steer_distribution.values())
        if total == 0:
            print("  No steering data collected")
            return

        print(f"\n  Steering Distribution ({total} total samples):")
        print(f"  {'Angle':>8} | {'Count':>7} | {'Percent':>7} | Bar")
        print(f"  {'-'*8}-+-{'-'*7}-+-{'-'*7}-+{'-'*30}")

        for angle in sorted(DiscreteControlEncoder.STEER_ANGLES):
            count = self.steer_distribution.get(angle, 0)
            pct = count / total * 100
            bar = '█' * int(pct)
            print(f"  {angle:>6}° | {count:>7} | {pct:>6.1f}% | {bar}")

        straight = sum(self.steer_distribution.get(a, 0) for a in [-2, 0, 2])
        turn = total - straight
        print(f"\n  Straight (|angle| ≤ 2°): {straight} ({straight/total*100:.1f}%)")
        print(f"  Turning  (|angle| > 2°): {turn} ({turn/total*100:.1f}%)")

    def print_nav_distribution(self):
        total = sum(self.nav_distribution.values())
        if total == 0:
            return
        print(f"\n  Navigation Distribution ({total} samples):")
        for cmd, count in sorted(self.nav_distribution.items(), key=lambda x: -x[1]):
            pct = count / total * 100
            print(f"    {cmd:<16}: {count:>7} ({pct:>5.1f}%)")


# =============================================================================
# PURE PURSUIT CONTROLLER
# =============================================================================

class PurePursuitController:
    def __init__(self, target_speed_kmh: float = 30.0, lookahead_distance: float = 8.0):
        self.target_speed_kmh = target_speed_kmh
        self.lookahead_distance = lookahead_distance
        self.speed_kp = 0.5
        self.speed_ki = 0.05
        self.speed_kd = 0.1
        self.speed_integral = 0.0
        self.prev_speed_error = 0.0

    def compute_control(self, waypoints_3d, vehicle_transform,
                        current_speed_kmh) -> carla.VehicleControl:
        control = carla.VehicleControl()
        if not waypoints_3d:
            control.throttle = 0.0
            control.brake = 1.0
            return control

        lookahead_wp = None
        for wp in waypoints_3d:
            if isinstance(wp, (list, tuple)):
                wx, wy = wp[0], wp[1]
            else:
                wx, wy = wp.x, wp.y
            dist = math.sqrt(
                (wx - vehicle_transform.location.x) ** 2 +
                (wy - vehicle_transform.location.y) ** 2
            )
            if dist >= self.lookahead_distance:
                lookahead_wp = (wx, wy)
                break

        if lookahead_wp is None:
            wp = waypoints_3d[-1]
            if isinstance(wp, (list, tuple)):
                lookahead_wp = (wp[0], wp[1])
            else:
                lookahead_wp = (wp.x, wp.y)

        dx = lookahead_wp[0] - vehicle_transform.location.x
        dy = lookahead_wp[1] - vehicle_transform.location.y
        target_angle = math.atan2(dy, dx)
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)
        angle_diff = target_angle - vehicle_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        steer = np.clip(angle_diff / math.radians(70), -1.0, 1.0)
        control.steer = float(steer)

        speed_error = self.target_speed_kmh - current_speed_kmh
        self.speed_integral = np.clip(self.speed_integral + speed_error, -20, 20)
        speed_derivative = speed_error - self.prev_speed_error
        speed_control = (self.speed_kp * speed_error +
                         self.speed_ki * self.speed_integral +
                         self.speed_kd * speed_derivative)
        if speed_control > 0:
            control.throttle = np.clip(speed_control, 0.0, 0.7)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = np.clip(-speed_control, 0.0, 0.5)
        self.prev_speed_error = speed_error
        return control

    def reset(self):
        self.speed_integral = 0.0
        self.prev_speed_error = 0.0


class StanleyController:
    def __init__(self, target_speed_kmh: float = 30.0,
                 k_heading: float = 1.0, k_crosstrack: float = 2.5):
        self.target_speed_kmh = target_speed_kmh
        self.k_heading = k_heading
        self.k_crosstrack = k_crosstrack
        self.speed_kp = 0.5
        self.speed_ki = 0.05
        self.speed_kd = 0.1
        self.speed_integral = 0.0
        self.prev_speed_error = 0.0

    def compute_control(self, waypoints_3d, vehicle_transform,
                        current_speed_kmh) -> carla.VehicleControl:
        control = carla.VehicleControl()
        if not waypoints_3d or len(waypoints_3d) < 2:
            control.brake = 1.0
            return control

        vx = vehicle_transform.location.x
        vy = vehicle_transform.location.y
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)

        min_dist = float('inf')
        closest_idx = 0
        for i, wp in enumerate(waypoints_3d):
            if isinstance(wp, (list, tuple)):
                wx, wy = wp[0], wp[1]
            else:
                wx, wy = wp.x, wp.y
            dist = math.sqrt((wx - vx) ** 2 + (wy - vy) ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        idx = closest_idx
        next_idx = min(idx + 3, len(waypoints_3d) - 1)

        if isinstance(waypoints_3d[idx], (list, tuple)):
            cx, cy = waypoints_3d[idx][0], waypoints_3d[idx][1]
            nx, ny = waypoints_3d[next_idx][0], waypoints_3d[next_idx][1]
        else:
            cx, cy = waypoints_3d[idx].x, waypoints_3d[idx].y
            nx, ny = waypoints_3d[next_idx].x, waypoints_3d[next_idx].y

        path_yaw = math.atan2(ny - cy, nx - cx)
        heading_error = path_yaw - vehicle_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        dx_v = vx - cx
        dy_v = vy - cy
        path_dx = nx - cx
        path_dy = ny - cy
        path_len = math.sqrt(path_dx ** 2 + path_dy ** 2)
        if path_len > 0.01:
            cross_track = (dx_v * path_dy - dy_v * path_dx) / path_len
        else:
            cross_track = 0.0

        speed_ms = max(current_speed_kmh / 3.6, 0.5)
        crosstrack_term = math.atan2(self.k_crosstrack * cross_track, speed_ms)
        steer_angle = self.k_heading * heading_error + crosstrack_term
        steer = np.clip(steer_angle / math.radians(70), -1.0, 1.0)
        control.steer = float(steer)
        

        turn_severity = abs(steer)
        if turn_severity > 0.2:
            speed_target = self.target_speed_kmh * (1.0 - turn_severity * 0.6)
        else:
            speed_target = self.target_speed_kmh

        speed_error = speed_target - current_speed_kmh
        self.speed_integral = np.clip(self.speed_integral + speed_error, -20, 20)
        speed_derivative = speed_error - self.prev_speed_error
        speed_control = (self.speed_kp * speed_error +
                         self.speed_ki * self.speed_integral +
                         self.speed_kd * speed_derivative)
        if speed_control > 0:
            control.throttle = np.clip(speed_control, 0.0, 0.7)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = np.clip(-speed_control, 0.0, 0.5)
        self.prev_speed_error = speed_error
        return control

    def reset(self):
        self.speed_integral = 0.0
        self.prev_speed_error = 0.0


# =============================================================================
# TRAINING PROMPT BUILDER
# =============================================================================

class PromptBuilder:
    @staticmethod
    def build_prompt(speed_kmh: float, nav_command: str) -> str:
        return f"Speed: {int(round(speed_kmh))} km/h. Navigation: {nav_command}."

    @staticmethod
    def build_output(discrete_control: dict) -> str:
        return json.dumps(discrete_control, separators=(',', ':'))


# =============================================================================
# PROGRESS DISPLAY
# =============================================================================

def print_progress(routes_completed, total_routes, samples_saved,
                   start_time, speed_kmh, queue_size,
                   route_name, current_loop, loops_per_route,
                   nav_command, steer_angle):
    elapsed = time.time() - start_time
    percent = routes_completed / max(total_routes, 1) * 100
    bar_width = 20
    filled = int(bar_width * routes_completed / max(total_routes, 1))
    bar = '█' * filled + '░' * (bar_width - filled)
    route_display = route_name[:16] + '..' if len(route_name) > 18 else route_name
    sys.stdout.write(
        f'\r[{bar}] {percent:5.1f}% | '
        f'R:{routes_completed}/{total_routes} | '
        f'S:{samples_saved:,} | '
        f'{speed_kmh:.0f}km/h | '
        f'steer:{steer_angle:>3}° | '
        f'nav:{nav_command:<12} | '
        f'Q:{queue_size} | '
        f'{route_display} ({current_loop}/{loops_per_route})'
    )
    sys.stdout.flush()


# =============================================================================
# MAIN RECORDING FUNCTION
# =============================================================================

def record_dataset(
    routes_dir: str = None,
    route_file: str = None,
    output_dir: str = None,
    loops_per_route: int = 1,
    use_display: bool = True,
    target_speed_kmh: float = 30.0,
):
    if output_dir is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = f"./datasets/dataset_{timestamp}"

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    display = None
    clock = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('Route Recording - Discrete Controls')
        clock = pygame.time.Clock()

    print(f"\n{'=' * 70}")
    print(f"ROUTE-BASED DATA COLLECTION (Option A: Direct Control)")
    print(f"{'=' * 70}")
    print(f"Output:         {output_dir}")
    print(f"Display:        {'ON' if display else 'OFF'}")
    print(f"Target speed:   {target_speed_kmh} km/h")
    print(f"Min speed:      {MIN_SPEED_KMH} km/h (captures turns!)")
    print(f"Steer bins:     {DiscreteControlEncoder.STEER_ANGLES}")
    print(f"Throttle bins:  {DiscreteControlEncoder.THROTTLE_BINS}")
    print(f"Brake bins:     {DiscreteControlEncoder.BRAKE_BINS}")
    print(f"Loops/route:    {loops_per_route}")
    print(f"{'=' * 70}\n")

    print("Loading routes...")
    route_loader = RouteLoader(routes_dir=routes_dir, route_file=route_file)

    print("\nConnecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    waypoint_extractor = CheckpointWaypointExtractor(distances=WAYPOINT_DISTANCES)
    controller = StanleyController(target_speed_kmh=target_speed_kmh)
    control_encoder = DiscreteControlEncoder()
    prompt_builder = PromptBuilder()

    data_saver = DataSaver(output_dir=output_dir, max_queue_size=500)
    data_saver.start()

    actors = []
    vehicle = None
    camera = None
    shutdown_requested = False

    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        print("\n\nShutdown requested...")
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        current_route_idx = 0
        current_route_loop = 0
        current_route = route_loader.get_route(current_route_idx)
        first_cp = current_route['checkpoints'][0]

        spawn_transform = carla.Transform(
            carla.Location(x=first_cp.x, y=first_cp.y, z=first_cp.z + 0.5),
            carla.Rotation(yaw=first_cp.yaw)
        )
        vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
        actors.append(vehicle)
        print(f"Vehicle spawned at route: {current_route['name']}")

        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(CAMERA_CONFIG.width))
        cam_bp.set_attribute('image_size_y', str(CAMERA_CONFIG.height))
        cam_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))
        cam_transform = carla.Transform(
            carla.Location(x=CAMERA_CONFIG.x, y=CAMERA_CONFIG.y, z=CAMERA_CONFIG.z),
            carla.Rotation(pitch=CAMERA_CONFIG.pitch, yaw=CAMERA_CONFIG.yaw, roll=CAMERA_CONFIG.roll)
        )
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(process_rgb)

        # Segmentation camera (same transform as RGB)
        seg_bp = bp_lib.find('sensor.camera.semantic_segmentation')
        seg_bp.set_attribute('image_size_x', str(CAMERA_CONFIG.width))
        seg_bp.set_attribute('image_size_y', str(CAMERA_CONFIG.height))
        seg_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))
        seg_camera = world.spawn_actor(seg_bp, cam_transform, attach_to=vehicle)
        actors.append(seg_camera)
        seg_camera.listen(process_seg)

        # Depth camera (same transform as RGB)
        depth_bp = bp_lib.find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', str(CAMERA_CONFIG.width))
        depth_bp.set_attribute('image_size_y', str(CAMERA_CONFIG.height))
        depth_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))
        depth_camera = world.spawn_actor(depth_bp, cam_transform, attach_to=vehicle)
        actors.append(depth_camera)
        depth_camera.listen(process_depth)

        # Trajectory encoder for 2D projection
        traj_encoder = None
        if TRAJ_AVAILABLE:
            traj_cam_config = TrajCameraConfig(
                width=CAMERA_CONFIG.width, height=CAMERA_CONFIG.height,
                fov=CAMERA_CONFIG.fov,
                x=CAMERA_CONFIG.x, y=CAMERA_CONFIG.y, z=CAMERA_CONFIG.z,
                pitch=CAMERA_CONFIG.pitch, yaw=CAMERA_CONFIG.yaw, roll=CAMERA_CONFIG.roll,
            )
            traj_encoder = TrajectoryEncoder(traj_cam_config)

        time.sleep(1.0)
        print("Starting data collection...\n")

        nav_detector = RouteNavigationDetector(current_route['checkpoints'])
        print(f"Route transitions detected:\n{nav_detector.get_transitions_summary()}\n")

        start_time = time.time()
        frame = 0
        total_routes_to_complete = route_loader.get_route_count() * loops_per_route
        routes_completed = 0
        route_usage = {}
        route_completion_count = {}
        last_steer_angle = 0

        while routes_completed < total_routes_to_complete and not shutdown_requested:
            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        shutdown_requested = True

            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(
                velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
            )

            last_checkpoint = current_route['checkpoints'][-1]
            dist_to_end = math.sqrt(
                (transform.location.x - last_checkpoint.x) ** 2 +
                (transform.location.y - last_checkpoint.y) ** 2
            )

            if dist_to_end < ROUTE_END_THRESHOLD:
                routes_completed += 1
                current_route_loop += 1
                route_name = current_route['name']
                route_completion_count[route_name] = route_completion_count.get(route_name, 0) + 1

                stats = data_saver.get_stats()
                print(f"\n✓ Route complete: {route_name} "
                      f"(loop {current_route_loop}/{loops_per_route}) | "
                      f"Total samples: {stats['saved']:,} | "
                      f"Progress: {routes_completed}/{total_routes_to_complete}")

                if current_route_loop >= loops_per_route:
                    current_route_idx = (current_route_idx + 1) % route_loader.get_route_count()
                    current_route_loop = 0

                if routes_completed < total_routes_to_complete:
                    current_route = route_loader.get_route(current_route_idx)
                    nav_detector = RouteNavigationDetector(current_route['checkpoints'])
                    print(f"→ Starting route: {current_route['name']}")
                    print(f"  Transitions:\n{nav_detector.get_transitions_summary()}")

                first_cp = current_route['checkpoints'][0]
                new_transform = carla.Transform(
                    carla.Location(x=first_cp.x, y=first_cp.y, z=first_cp.z + 0.5),
                    carla.Rotation(yaw=first_cp.yaw)
                )
                vehicle.set_transform(new_transform)
                vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                controller.reset()
                time.sleep(0.5)
                continue

            waypoints_3d = waypoint_extractor.get_waypoints(
                current_route['checkpoints'], transform.location
            )

            control = controller.compute_control(waypoints_3d, transform, speed_kmh)
            vehicle.apply_control(control)

            closest_idx = waypoint_extractor.find_closest_checkpoint(
                current_route['checkpoints'], transform.location
            )
            nav_command = nav_detector.get_command(transform.location, closest_idx)

            discrete_control = control_encoder.encode(control)

            last_steer_angle = discrete_control['steer']
            train_nav_cmd = control_encoder.train_nav(discrete_control['steer'])

            if (sensor_data.rgb_image is not None and
                    speed_kmh >= MIN_SPEED_KMH and
                    len(waypoints_3d) >= MIN_WAYPOINTS):

                prompt = prompt_builder.build_prompt(speed_kmh, train_nav_cmd)
                output = prompt_builder.build_output(discrete_control)

                # Project 3D waypoints to 2D pixel coordinates
                trajectory_2d = None
                if traj_encoder is not None:
                    wp3d_objs = [Waypoint3D(x=w[0], y=w[1], z=w[2]) for w in waypoints_3d]
                    wp2d_objs = traj_encoder.encode(wp3d_objs, transform)
                    if wp2d_objs:
                        trajectory_2d = [[round(wp.u, 1), round(wp.v, 1)] for wp in wp2d_objs]

                sample = {
                    'rgb_image': sensor_data.rgb_image,
                    'seg_image': sensor_data.seg_image,
                    'depth_image': sensor_data.depth_image,
                    'prompt': prompt,
                    'output': output,
                    'control': discrete_control,
                    'control_raw': {
                        'steer': round(control.steer, 6),
                        'throttle': round(control.throttle, 4),
                        'brake': round(control.brake, 4)
                    },
                    'navigation (based on waypoints)': nav_command,
                    'train_nav_command': train_nav_cmd,
                    'speed_kmh': round(speed_kmh, 1),
                    'timestamp': sensor_data.timestamp,
                    'route_name': current_route['name'],
                    'vehicle_location': [
                        round(transform.location.x, 2),
                        round(transform.location.y, 2),
                        round(transform.location.z, 2)
                    ],
                    'vehicle_rotation': [
                        round(transform.rotation.pitch, 2),
                        round(transform.rotation.yaw, 2),
                        round(transform.rotation.roll, 2)
                    ],
                    'waypoints_3d': [
                        [round(w[0], 2), round(w[1], 2), round(w[2], 2)]
                        for w in waypoints_3d
                    ],
                }
                if trajectory_2d:
                    sample['trajectory_2d'] = trajectory_2d

                if data_saver.queue_sample(sample):
                    route_name = current_route['name']
                    route_usage[route_name] = route_usage.get(route_name, 0) + 1

            if display and sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(
                    sensor_data.rgb_image.swapaxes(0, 1)
                )
                display.blit(surface, (0, 0))

                font = pygame.font.SysFont('monospace', 14)

                hud_text = (f"Speed: {speed_kmh:.0f} km/h | "
                            f"Steer: {last_steer_angle}° | "
                            f"Nav: {nav_command} |"
                            f"TrainNav: {train_nav_cmd}")
                text_surface = font.render(hud_text, True, (255, 255, 0))
                display.blit(text_surface, (10, 10))

                stats = data_saver.get_stats()
                prog_text = (f"Routes: {routes_completed}/{total_routes_to_complete} | "
                             f"Samples: {stats['saved']:,}")
                text_surface = font.render(prog_text, True, (255, 255, 255))
                display.blit(text_surface, (10, 30))

                bar_center_x = WINDOW_WIDTH // 2
                bar_y = WINDOW_HEIGHT - 20
                bar_half_width = 100
                steer_pos = bar_center_x + int(last_steer_angle / 60 * bar_half_width)
                pygame.draw.line(display, (100, 100, 100),
                                 (bar_center_x - bar_half_width, bar_y),
                                 (bar_center_x + bar_half_width, bar_y), 2)
                pygame.draw.circle(display, (255, 0, 0), (steer_pos, bar_y), 6)
                pygame.draw.line(display, (0, 255, 0),
                                 (bar_center_x, bar_y - 4),
                                 (bar_center_x, bar_y + 4), 2)

                route_progress = closest_idx / max(len(current_route['checkpoints']), 1)
                bar_width = WINDOW_WIDTH - 40
                pygame.draw.rect(display, (50, 50, 50),
                                 (20, WINDOW_HEIGHT - 40, bar_width, 10))
                pygame.draw.rect(display, (0, 200, 0),
                                 (20, WINDOW_HEIGHT - 40,
                                  int(bar_width * route_progress), 10))

                pygame.display.flip()
                clock.tick(30)
            else:
                world.tick()
                time.sleep(0.033)

            if frame % 30 == 0:
                stats = data_saver.get_stats()
                print_progress(
                    routes_completed, total_routes_to_complete,
                    stats['saved'], start_time, speed_kmh,
                    stats['queue_size'], current_route['name'],
                    current_route_loop + 1, loops_per_route,
                    nav_command, last_steer_angle
                )

            frame += 1

        print(f"\n\n{'=' * 70}")
        print("DATA COLLECTION COMPLETE!")
        print(f"{'=' * 70}")

    except Exception as e:
        print(f"\n\nError: {e}")
        traceback.print_exc()

    finally:
        print("\nFinalizing data...")
        data_saver.stop()

        elapsed = time.time() - start_time
        stats = data_saver.get_stats()

        manifest = {
            'dataset_info': {
                'created': datetime.now().isoformat(),
                'format': 'option_a_direct_control',
                'total_samples': stats['saved'],
                'total_time_seconds': round(elapsed, 1),
                'samples_per_second': round(stats['saved'] / max(elapsed, 1), 2),
            },
            'config': {
                'camera': {
                    'width': CAMERA_CONFIG.width,
                    'height': CAMERA_CONFIG.height,
                    'fov': CAMERA_CONFIG.fov,
                    'position': [CAMERA_CONFIG.x, CAMERA_CONFIG.y, CAMERA_CONFIG.z],
                    'rotation': [CAMERA_CONFIG.pitch, CAMERA_CONFIG.yaw, CAMERA_CONFIG.roll],
                },
                'control_encoding': {
                    'steer_angles': DiscreteControlEncoder.STEER_ANGLES,
                    'max_steer_angle': DiscreteControlEncoder.MAX_STEER_ANGLE,
                    'throttle_bins': DiscreteControlEncoder.THROTTLE_BINS,
                    'brake_bins': DiscreteControlEncoder.BRAKE_BINS,
                },
                'waypoint_distances': WAYPOINT_DISTANCES,
                'min_speed_kmh': MIN_SPEED_KMH,
                'target_speed_kmh': target_speed_kmh,
            },
            'training_format': {
                'prompt_template': 'Speed: {speed} km/h. Navigation: {nav_command}.',
                'output_template': '{"steer": <angle_degrees>, "throttle": <bin>, "brake": <bin>}',
                'output_example': '{"steer":-15,"throttle":0.4,"brake":0.0}',
                'navigation_commands': [
                    'follow_lane', 'turn_left', 'turn_right',
                    'go_straight', 'u_turn', 'merge_left', 'merge_right'
                ],
            },
            'routes': {
                'total_routes': route_loader.get_route_count(),
                'loops_per_route': loops_per_route,
                'routes_completed': routes_completed,
                'route_usage': route_usage,
                'route_completions': route_completion_count,
            },
            'data_quality': {
                'steering_distribution': {
                    str(k): v for k, v in sorted(data_saver.steer_distribution.items())
                },
                'navigation_distribution': dict(data_saver.nav_distribution),
                'dropped_samples': stats['dropped'],
            },
        }

        manifest_path = output_path / 'manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)

        print(f"\nFinal Statistics:")
        print(f"  Routes completed: {routes_completed}/{total_routes_to_complete}")
        print(f"  Samples saved:    {stats['saved']:,}")
        print(f"  Samples dropped:  {stats['dropped']}")
        print(f"  Total time:       {elapsed / 60:.1f} minutes")
        print(f"  Average rate:     {stats['saved'] / max(elapsed, 1):.1f} samples/second")

        data_saver.print_steer_distribution()
        data_saver.print_nav_distribution()

        print(f"\nRoute Completions:")
        for rname, count in sorted(route_completion_count.items(),
                                   key=lambda x: x[1], reverse=True):
            samples = route_usage.get(rname, 0)
            print(f"  {rname}: {count}× completed, {samples} samples")

        print("\nCleaning up...")
        for actor in actors:
            if actor is not None:
                try:
                    actor.destroy()
                except:
                    pass

        if display:
            pygame.quit()

        print(f"\n{'=' * 70}")
        print(f"Dataset saved to: {output_dir}")
        print(f"Manifest: {manifest_path}")
        print(f"\nSample label format:")
        print(f'  prompt:  "Speed: 25 km/h. Navigation: turn_right."')
        print(f'  output:  {{"steer":-15,"throttle":0.4,"brake":0.0}}')
        print(f"{'=' * 70}\n")



def convert_npz_to_jpeg(data_dir: str, quality: int = 95, delete_npz: bool = False):
    """
    Convert .npy/.npz images to JPEG format.

    Run this AFTER the driving session to convert raw data.

    Args:
        data_dir: Directory containing raw/ folder with .npy/.npz files
        quality: JPEG quality (1-100)
        delete_npz: Whether to delete .npy/.npz files after conversion
    """
    from PIL import Image

    data_path = Path(data_dir)
    raw_dir = data_path / 'raw'
    images_dir = data_path / 'images'
    labels_dir = data_path / 'labels'

    if not raw_dir.exists():
        print(f"No raw/ directory found in {data_dir}")
        return

    # Look for both .npy and .npz files
    npy_files = sorted(raw_dir.glob('*.npy'))
    npz_files = sorted(raw_dir.glob('*.npz'))
    all_files = npy_files + npz_files

    if not all_files:
        print(f"No .npy or .npz files found in {raw_dir}")
        return

    print(f"Converting {len(all_files)} files to JPEG...")
    print(f"  .npy files: {len(npy_files)}")
    print(f"  .npz files: {len(npz_files)}")

    converted = 0
    errors = 0

    for file_path in all_files:
        try:
            sample_id = file_path.stem

            # Load image based on file type
            if file_path.suffix == '.npy':
                # Load .npy file directly
                image = np.load(file_path)
            else:
                # Load .npz file with key access
                data = np.load(file_path)
                image = data['image']

            # Validate image shape
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"Invalid image shape: {image.shape}, expected (H, W, 3)")

            # Ensure uint8 format
            if image.dtype != np.uint8:
                # Convert to uint8 if needed
                if image.max() <= 1.0:
                    image = (image * 255).astype(np.uint8)
                else:
                    image = image.astype(np.uint8)

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

            # Optionally delete original file
            if delete_npz:
                file_path.unlink()

            converted += 1

            if converted % 100 == 0:
                print(f"  Converted {converted}/{len(all_files)}...")

        except Exception as e:
            print(f"  Error converting {file_path}: {e}")
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
        print(f"  Deleted source files: {converted}")


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Route-Based Dataset Recording (Option A: Direct Discrete Control)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python record_dataset.py --routes ./routes
  python record_dataset.py --routes ./routes --loops 3
  python record_dataset.py --route ./routes/route_city_left_01.json --loops 5
  python record_dataset.py --routes ./routes --no-display --target-speed 20
        """
    )

    parser.add_argument('--routes', type=str, help='Directory containing route JSON files')
    parser.add_argument('--route', type=str, help='Single route JSON file')
    parser.add_argument('--loops', type=int, default=1, help='Times to complete each route (default: 1)')
    parser.add_argument('--output', type=str, default='./dataset', help='Output directory (default: auto-generated)')
    parser.add_argument('--no-display', action='store_true', help='Headless mode')
    parser.add_argument('--target-speed', type=float, default=30.0, help='Target driving speed km/h (default: 30)')

    args = parser.parse_args()

    if not args.routes and not args.route:
        parser.error("Must provide either --routes or --route")
    if args.routes and args.route:
        parser.error("Cannot use both --routes and --route")

    record_dataset(
        routes_dir=args.routes,
        route_file=args.route,
        output_dir=args.output,
        loops_per_route=args.loops,
        use_display=not args.no_display,
        target_speed_kmh=args.target_speed,
    )
    print("\nDataset recording finished.")
    print("Converting raw .npz images to JPEG format...")
    convert_npz_to_jpeg(
        data_dir='./dataset',
        quality=95,
        delete_npz=True
    )
    print("\nAll done! Your dataset is ready for HF dataset creation.")


