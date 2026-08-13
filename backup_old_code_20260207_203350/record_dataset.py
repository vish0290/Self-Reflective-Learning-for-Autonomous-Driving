#!/usr/bin/env python3
"""
Route-Based Autonomous Data Collection System

Features:
1. Loads checkpoint routes from JSON files
2. Extracts waypoints at specific distances from dense checkpoints
3. Records navigation commands (lane_keeping, turn_left, turn_right, etc.)
4. Includes decision reasoning for each frame
5. Tracks route name in dataset metadata
6. Supports variable-length routes

Directory Structure:
    core/
    ├── record_dataset.py (this file)
    ├── routes/
    │   ├── route_highway_01.json
    │   ├── route_city_intersection_left_01.json
    │   ├── route_curve_right_01.json
    │   └── ... (50+ routes)
    └── datasets/
        └── dataset_YYYYMMDD_HHMMSS/
            ├── raw/
            ├── images/
            ├── labels/
            ├── manifest.json
            └── route_manifest.json

Usage:
    # Record from all routes in routes/ folder
    python record_dataset.py --samples 10000 --routes ./routes

    # Record from specific route
    python record_dataset.py --samples 500 --route ./routes/route_highway_01.json

    # Samples per route (round-robin)
    python record_dataset.py --samples 10000 --routes ./routes --samples-per-route 200
"""

import carla
import numpy as np
import math
import time
import argparse
import signal
import sys
import json
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass

# Try to import pygame (optional for display)
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not available - running in headless mode")

# Import trajectory modules
sys.path.insert(0, str(Path(__file__).parent.parent / 'sandbox'))

from traj_planner import (
    CameraConfig, TrajectoryEncoder, TrajectoryDecoder, Waypoint3D, Waypoint2D
)
from data_collector import AsyncDataCollector
from navigation_analyzer import generate_navigation_token, calculate_trajectory_direction


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

CAMERA_CONFIG = CameraConfig(
    width=WINDOW_WIDTH,
    height=WINDOW_HEIGHT,
    fov=90,
    x=2.0,
    y=0.0,
    z=1.8,
    pitch=-15,
    yaw=0,
    roll=0
)

# Waypoint distances (meters) - sparse for model prediction
WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]

# Navigation command mapping
NAV_COMMANDS = {
    'lane_keeping': 0,
    'turn_left': 1,
    'turn_right': 2,
    'u_turn': 3,
    'merge_left': 4,
    'merge_right': 5,
    'intersection_approach': 6
}


# =============================================================================
# ROUTE CHECKPOINT LOADER
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


class RouteLoader:
    """Loads and manages route checkpoint files."""

    def __init__(self, routes_dir: str = None, route_file: str = None):
        """
        Args:
            routes_dir: Directory containing route JSON files (loads all)
            route_file: Single route file to load
        """
        self.routes = []

        if route_file:
            self.load_route(route_file)
        elif routes_dir:
            self.load_routes_from_directory(routes_dir)
        else:
            raise ValueError("Must provide either routes_dir or route_file")

    def load_route(self, filepath: str) -> Dict:
        """Load a single route file."""
        with open(filepath, 'r') as f:
            route_data = json.load(f)

        route_name = Path(filepath).stem

        # Handle different route file formats
        checkpoint_data = None
        total_distance = 0

        # Format 1: Direct checkpoints at root level
        if 'checkpoints' in route_data:
            checkpoint_data = route_data['checkpoints']
            total_distance = route_data.get('total_distance', 0)

        # Format 2: Nested in scenarios.custom[0]
        elif 'scenarios' in route_data and 'custom' in route_data['scenarios']:
            if len(route_data['scenarios']['custom']) > 0:
                custom_route = route_data['scenarios']['custom'][0]
                checkpoint_data = custom_route.get('checkpoints', [])
                # Calculate distance from checkpoints if not provided
                if checkpoint_data and len(checkpoint_data) > 1:
                    total_distance = 0
                    for i in range(1, len(checkpoint_data)):
                        dx = checkpoint_data[i]['x'] - checkpoint_data[i-1]['x']
                        dy = checkpoint_data[i]['y'] - checkpoint_data[i-1]['y']
                        total_distance += math.sqrt(dx*dx + dy*dy)

        if not checkpoint_data:
            raise ValueError(f"No checkpoints found in route file")

        # Parse checkpoints
        checkpoints = []
        for cp in checkpoint_data:
            checkpoints.append(RouteCheckpoint(
                x=cp['x'],
                y=cp['y'],
                z=cp['z'],
                yaw=cp.get('yaw', 0.0),
                road_id=cp.get('road_id', 0),
                lane_id=cp.get('lane_id', 0)
            ))

        route = {
            'name': route_name,
            'filepath': filepath,
            'checkpoints': checkpoints,
            'total_distance': total_distance,
            'metadata': route_data
        }

        self.routes.append(route)
        return route

    def load_routes_from_directory(self, dirpath: str):
        """Load all route JSON files from directory."""
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
            raise ValueError(f"Failed to load any routes from {dirpath}. Check route file format.")

    def get_route(self, index: int) -> Dict:
        """Get route by index."""
        return self.routes[index % len(self.routes)]

    def get_route_count(self) -> int:
        """Get total number of routes."""
        return len(self.routes)


# =============================================================================
# WAYPOINT EXTRACTOR FROM CHECKPOINTS
# =============================================================================

class CheckpointWaypointExtractor:
    """Extracts waypoints at specific distances from dense checkpoint array."""

    def __init__(self, distances: List[float] = WAYPOINT_DISTANCES):
        """
        Args:
            distances: Target distances for waypoint extraction (e.g., [3, 6, 9, ...])
        """
        self.distances = distances

    def find_closest_checkpoint(self, checkpoints: List[RouteCheckpoint],
                                vehicle_location: carla.Location) -> int:
        """Find checkpoint index closest to vehicle."""
        min_dist = float('inf')
        closest_idx = 0

        for i, cp in enumerate(checkpoints):
            dist = math.sqrt(
                (cp.x - vehicle_location.x)**2 +
                (cp.y - vehicle_location.y)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def calculate_cumulative_distances(self, checkpoints: List[RouteCheckpoint],
                                      start_idx: int) -> List[float]:
        """Calculate cumulative distances from start checkpoint."""
        cumulative = [0.0]

        for i in range(start_idx + 1, len(checkpoints)):
            prev_cp = checkpoints[i - 1]
            curr_cp = checkpoints[i]

            dist = math.sqrt(
                (curr_cp.x - prev_cp.x)**2 +
                (curr_cp.y - prev_cp.y)**2
            )
            cumulative.append(cumulative[-1] + dist)

        return cumulative

    def get_waypoints(self, checkpoints: List[RouteCheckpoint],
                     vehicle_location: carla.Location) -> List[Waypoint3D]:
        """
        Extract waypoints at target distances from checkpoint route.

        Args:
            checkpoints: Dense checkpoint array (~1m spacing)
            vehicle_location: Current vehicle location

        Returns:
            List of Waypoint3D at target distances
        """
        # Find closest checkpoint
        closest_idx = self.find_closest_checkpoint(checkpoints, vehicle_location)

        # Calculate cumulative distances
        cumulative_dists = self.calculate_cumulative_distances(checkpoints, closest_idx)

        # Extract waypoints at target distances
        waypoints_3d = []

        for target_dist in self.distances:
            # Find checkpoint at this distance
            cp_idx = None
            for i, cum_dist in enumerate(cumulative_dists):
                if cum_dist >= target_dist:
                    cp_idx = closest_idx + i
                    break

            # If we've reached end of route, use last checkpoint
            if cp_idx is None or cp_idx >= len(checkpoints):
                cp_idx = len(checkpoints) - 1

            cp = checkpoints[cp_idx]
            waypoints_3d.append(Waypoint3D(x=cp.x, y=cp.y, z=cp.z))

        return waypoints_3d


# =============================================================================
# DECISION REASONING GENERATOR
# =============================================================================

class DecisionReasoner:
    """Generates human-readable reasoning for navigation decisions."""

    @staticmethod
    def generate_reasoning(
        navigation_command: str,
        waypoints_3d: List[Waypoint3D],
        vehicle_state: Dict,
        route_name: str
    ) -> str:
        """
        Generate decision reasoning string.

        Args:
            navigation_command: Navigation token (e.g., 'turn_left')
            waypoints_3d: 3D waypoints
            vehicle_state: Vehicle state dict
            route_name: Name of current route

        Returns:
            Human-readable reasoning string
        """
        speed_kmh = vehicle_state.get('speed_kmh', 0)

        # Calculate trajectory curvature
        if len(waypoints_3d) >= 3:
            wp_array = [[wp.x, wp.y, wp.z] for wp in waypoints_3d]

            # Compute lateral deviation
            first_wp = np.array([waypoints_3d[0].x, waypoints_3d[0].y])
            last_wp = np.array([waypoints_3d[-1].x, waypoints_3d[-1].y])
            mid_wp = np.array([waypoints_3d[len(waypoints_3d)//2].x,
                              waypoints_3d[len(waypoints_3d)//2].y])

            # Vector from first to last
            direction_vec = last_wp - first_wp
            direction_norm = np.linalg.norm(direction_vec)

            if direction_norm > 0:
                direction_unit = direction_vec / direction_norm

                # Perpendicular distance of mid point from line
                vec_to_mid = mid_wp - first_wp
                projection = np.dot(vec_to_mid, direction_unit)
                closest_point = first_wp + projection * direction_unit
                lateral_offset = np.linalg.norm(mid_wp - closest_point)

                # Determine side
                cross = np.cross(direction_unit, vec_to_mid)
                side = "left" if cross > 0 else "right"
            else:
                lateral_offset = 0
                side = "straight"
        else:
            lateral_offset = 0
            side = "straight"

        # Generate reasoning based on command
        if navigation_command == 'lane_keeping':
            if lateral_offset < 1.0:
                reasoning = f"Following straight path on {route_name}. Trajectory ahead is straight with minimal curvature ({lateral_offset:.1f}m deviation). Maintaining lane at {speed_kmh:.0f} km/h."
            else:
                reasoning = f"Following curved path on {route_name}. Road curves slightly {side} ({lateral_offset:.1f}m lateral offset). Maintaining lane at {speed_kmh:.0f} km/h."

        elif navigation_command == 'turn_left':
            reasoning = f"Executing left turn on {route_name}. Trajectory curves left with {lateral_offset:.1f}m lateral deviation. Speed: {speed_kmh:.0f} km/h. Steering to follow curved path."

        elif navigation_command == 'turn_right':
            reasoning = f"Executing right turn on {route_name}. Trajectory curves right with {lateral_offset:.1f}m lateral deviation. Speed: {speed_kmh:.0f} km/h. Steering to follow curved path."

        elif navigation_command == 'u_turn':
            reasoning = f"Executing U-turn on {route_name}. Sharp reversal maneuver detected. Speed: {speed_kmh:.0f} km/h. Full steering required."

        elif navigation_command == 'merge_left':
            reasoning = f"Merging left on {route_name}. Gradual lane change to left with {lateral_offset:.1f}m offset. Speed: {speed_kmh:.0f} km/h."

        elif navigation_command == 'merge_right':
            reasoning = f"Merging right on {route_name}. Gradual lane change to right with {lateral_offset:.1f}m offset. Speed: {speed_kmh:.0f} km/h."

        elif navigation_command == 'intersection_approach':
            reasoning = f"Approaching intersection on {route_name}. Multiple path options detected. Preparing for turn. Speed: {speed_kmh:.0f} km/h."

        else:
            reasoning = f"Unknown navigation command '{navigation_command}' on {route_name}. Speed: {speed_kmh:.0f} km/h."

        return reasoning


# =============================================================================
# SENSOR DATA
# =============================================================================

class SensorData:
    def __init__(self):
        self.rgb_image = None
        self.timestamp = 0

sensor_data = SensorData()

def process_rgb(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    sensor_data.rgb_image = array[:, :, ::-1].copy()
    sensor_data.timestamp = image.timestamp


# =============================================================================
# PROGRESS DISPLAY
# =============================================================================

def print_progress(routes_completed, total_routes, samples_saved, start_time, speed_kmh, queue_size, route_name, current_loop, loops_per_route):
    """Print progress bar and stats."""
    elapsed = time.time() - start_time
    percent = routes_completed / total_routes * 100

    bar_width = 25
    filled = int(bar_width * routes_completed / total_routes) if total_routes > 0 else 0
    bar = '█' * filled + '░' * (bar_width - filled)

    # Truncate route name if too long
    route_display = route_name[:18] + '...' if len(route_name) > 21 else route_name

    sys.stdout.write(f'\r[{bar}] {percent:5.1f}% | '
                     f'Routes: {routes_completed}/{total_routes} | '
                     f'Samples: {samples_saved:,} | '
                     f'Speed: {speed_kmh:.0f}km/h | '
                     f'Q: {queue_size} | '
                     f'{route_display} (loop {current_loop}/{loops_per_route})')
    sys.stdout.flush()


# =============================================================================
# SIMPLE PURE PURSUIT CONTROLLER
# =============================================================================

class SimplePurePursuitController:
    """Simplified pure pursuit controller for route following."""

    def __init__(self, target_speed_kmh: float = 45.0, lookahead_distance: float = 8.0):
        self.target_speed_kmh = target_speed_kmh
        self.lookahead_distance = lookahead_distance
        self.speed_kp = 0.5
        self.speed_ki = 0.05
        self.speed_kd = 0.1
        self.speed_integral = 0.0
        self.prev_speed_error = 0.0

    def compute_control(self, waypoints_3d: List[Waypoint3D],
                       vehicle_transform: carla.Transform,
                       current_speed_kmh: float) -> carla.VehicleControl:
        """Compute control from waypoints."""
        control = carla.VehicleControl()

        if not waypoints_3d:
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            return control

        # Find lookahead point
        lookahead_wp = None
        for wp in waypoints_3d:
            dist = math.sqrt((wp.x - vehicle_transform.location.x)**2 +
                           (wp.y - vehicle_transform.location.y)**2)
            if dist >= self.lookahead_distance:
                lookahead_wp = wp
                break

        if lookahead_wp is None:
            lookahead_wp = waypoints_3d[-1]

        # Compute steering angle
        dx = lookahead_wp.x - vehicle_transform.location.x
        dy = lookahead_wp.y - vehicle_transform.location.y

        target_angle = math.atan2(dy, dx)
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)

        angle_diff = target_angle - vehicle_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Normalize to [-1, 1]
        steer = np.clip(angle_diff / math.radians(70), -1.0, 1.0)
        control.steer = float(steer)

        # Speed control (PID)
        speed_error = self.target_speed_kmh - current_speed_kmh
        self.speed_integral += speed_error
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
        """Reset controller state."""
        self.speed_integral = 0.0
        self.prev_speed_error = 0.0


# =============================================================================
# MAIN RECORDING FUNCTION
# =============================================================================

def record_dataset(
    target_samples: int = None,
    routes_dir: str = None,
    route_file: str = None,
    output_dir: str = None,
    loops_per_route: int = 1,
    use_display: bool = True,
    min_speed_kmh: float = 15.0,
    target_speed_kmh: float = 30.0,
    end_threshold_meters: float = 5.0
):
    """
    Record dataset from checkpoint routes.

    Args:
        target_samples: Total number of samples to collect (optional, for backward compat)
        routes_dir: Directory containing route JSON files
        route_file: Single route file (alternative to routes_dir)
        output_dir: Output directory (auto-generated if None)
        loops_per_route: How many times to complete each route (default: 1)
        use_display: Whether to show pygame window
        min_speed_kmh: Minimum speed to record samples
        target_speed_kmh: Target driving speed
        end_threshold_meters: Distance to last checkpoint to consider route complete
    """

    # Auto-generate output directory
    if output_dir is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = f"./datasets/dataset_{timestamp}"

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    # Initialize display
    display = None
    clock = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('Route-Based Recording - Complete Routes Mode')
        clock = pygame.time.Clock()

    # Print header
    print(f"\n{'='*70}")
    print(f"ROUTE-BASED DATA COLLECTION")
    print(f"{'='*70}")
    print(f"Output: {output_dir}")
    print(f"Display: {'ON' if display else 'OFF'}")
    print(f"Mode: Complete each route {loops_per_route} time(s)")
    print(f"Route end threshold: {end_threshold_meters}m from last checkpoint")
    print(f"{'='*70}\n")

    # Load routes
    print("Loading routes...")
    route_loader = RouteLoader(routes_dir=routes_dir, route_file=route_file)
    print(f"Loaded {route_loader.get_route_count()} routes\n")

    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # Initialize components
    waypoint_extractor = CheckpointWaypointExtractor(distances=WAYPOINT_DISTANCES)
    encoder = TrajectoryEncoder(CAMERA_CONFIG)
    decoder = TrajectoryDecoder(CAMERA_CONFIG)
    controller = SimplePurePursuitController(target_speed_kmh=target_speed_kmh)
    reasoner = DecisionReasoner()

    # Initialize data collector
    data_collector = AsyncDataCollector(
        output_dir=output_dir,
        max_queue_size=500,
        save_interval=1
    )
    data_collector.set_config(
        camera_config={
            'width': CAMERA_CONFIG.width,
            'height': CAMERA_CONFIG.height,
            'fov': CAMERA_CONFIG.fov,
            'position': [CAMERA_CONFIG.x, CAMERA_CONFIG.y, CAMERA_CONFIG.z],
            'rotation': [CAMERA_CONFIG.pitch, CAMERA_CONFIG.yaw, CAMERA_CONFIG.roll]
        },
        waypoint_distances=WAYPOINT_DISTANCES
    )
    data_collector.start()

    actors = []
    vehicle = None
    camera = None
    current_route_idx = 0
    current_route_loop = 0
    samples_on_current_route = 0
    start_time = time.time()  # Initialize early to prevent UnboundLocalError
    routes_completed = 0

    # Route tracking for manifest
    route_usage = {}
    route_completion_count = {}  # Track how many times each route was completed

    # Signal handler
    shutdown_requested = False
    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        print("\n\nShutdown requested...")
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Spawn vehicle
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        # Get first route spawn point
        current_route = route_loader.get_route(current_route_idx)
        first_checkpoint = current_route['checkpoints'][0]

        spawn_transform = carla.Transform(
            carla.Location(x=first_checkpoint.x, y=first_checkpoint.y, z=first_checkpoint.z + 0.5),
            carla.Rotation(yaw=first_checkpoint.yaw)
        )

        vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
        actors.append(vehicle)
        print(f"Vehicle spawned at route: {current_route['name']}")

        # Attach camera
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(WINDOW_WIDTH))
        cam_bp.set_attribute('image_size_y', str(WINDOW_HEIGHT))
        cam_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))

        cam_transform = carla.Transform(
            carla.Location(x=CAMERA_CONFIG.x, y=CAMERA_CONFIG.y, z=CAMERA_CONFIG.z),
            carla.Rotation(pitch=CAMERA_CONFIG.pitch, yaw=CAMERA_CONFIG.yaw, roll=CAMERA_CONFIG.roll)
        )

        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(process_rgb)

        time.sleep(1.0)
        print("Starting data collection...\n")

        start_time = time.time()  # Reset to actual start time
        frame = 0
        total_routes_to_complete = route_loader.get_route_count() * loops_per_route
        routes_completed = 0

        while routes_completed < total_routes_to_complete and not shutdown_requested:
            # Handle pygame events
            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            shutdown_requested = True

            # Get vehicle state FIRST (needed for route completion check)
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            # Check if route is complete (near end checkpoint)
            last_checkpoint = current_route['checkpoints'][-1]
            dist_to_end = math.sqrt(
                (transform.location.x - last_checkpoint.x)**2 +
                (transform.location.y - last_checkpoint.y)**2
            )

            if dist_to_end < end_threshold_meters:
                # Route completed!
                routes_completed += 1
                current_route_loop += 1

                # Track completion
                route_name = current_route['name']
                if route_name not in route_completion_count:
                    route_completion_count[route_name] = 0
                route_completion_count[route_name] += 1

                print(f"\n✓ Route complete: {current_route['name']} (loop {current_route_loop}/{loops_per_route}) | {samples_on_current_route} samples | Progress: {routes_completed}/{total_routes_to_complete}")

                # Check if we need to loop this route again or move to next
                if current_route_loop >= loops_per_route:
                    # Move to next route
                    current_route_idx = (current_route_idx + 1) % route_loader.get_route_count()
                    current_route_loop = 0

                    if routes_completed < total_routes_to_complete:
                        current_route = route_loader.get_route(current_route_idx)
                        print(f"→ Starting route: {current_route['name']}")

                # Teleport to route start
                first_cp = current_route['checkpoints'][0]
                new_transform = carla.Transform(
                    carla.Location(x=first_cp.x, y=first_cp.y, z=first_cp.z + 0.5),
                    carla.Rotation(yaw=first_cp.yaw)
                )
                vehicle.set_transform(new_transform)
                controller.reset()
                samples_on_current_route = 0
                time.sleep(0.5)
                continue

            # Extract waypoints from current route checkpoints
            waypoints_3d_original = waypoint_extractor.get_waypoints(
                current_route['checkpoints'],
                transform.location
            )

            # Encode to 2D pixels (GROUND TRUTH!)
            waypoints_2d, kept_indices = encoder.encode_with_indices(waypoints_3d_original, transform)

            # Decode for control
            if kept_indices and waypoints_3d_original:
                road_height = sum(waypoints_3d_original[i].z for i in kept_indices) / len(kept_indices)
            else:
                road_height = transform.location.z - 0.5

            waypoints_3d_decoded = decoder.decode(waypoints_2d, transform, road_height)
            waypoints_3d_kept = [waypoints_3d_original[i] for i in kept_indices] if kept_indices else []
            waypoints_3d = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d_original

            # Compute control
            control = controller.compute_control(waypoints_3d, transform, speed_kmh)
            vehicle.apply_control(control)

            # Generate navigation command
            wp_array = [[wp.x, wp.y, wp.z] for wp in waypoints_3d_kept]
            navigation_command = generate_navigation_token(wp_array, use_map_api=False)

            # Collect data if conditions met
            if sensor_data.rgb_image is not None:
                if speed_kmh >= min_speed_kmh and waypoints_2d and len(waypoints_2d) >= 5:
                    vehicle_state = {
                        'speed_kmh': speed_kmh,
                        'location': [transform.location.x, transform.location.y, transform.location.z],
                        'rotation': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
                    }

                    # Generate decision reasoning
                    reasoning = reasoner.generate_reasoning(
                        navigation_command,
                        waypoints_3d_kept,
                        vehicle_state,
                        current_route['name']
                    )

                    # Queue sample with extended metadata
                    sample_data = {
                        'rgb_image': sensor_data.rgb_image,
                        'waypoints_2d': waypoints_2d,
                        'waypoints_3d': waypoints_3d_kept,
                        'kept_indices': kept_indices,
                        'vehicle_state': vehicle_state,
                        'control': {
                            'throttle': control.throttle,
                            'brake': control.brake,
                            'steer': control.steer
                        },
                        'timestamp': sensor_data.timestamp,
                        'navigation_command': navigation_command,
                        'navigation_command_id': NAV_COMMANDS.get(navigation_command, 0),
                        'decision_reasoning': reasoning,
                        'route_name': current_route['name'],
                        'route_metadata': {
                            'total_distance': current_route['total_distance'],
                            'filepath': current_route['filepath']
                        }
                    }

                    # Modified queue_sample call to handle extended data
                    if _queue_extended_sample(data_collector, sample_data):
                        samples_on_current_route += 1

                        # Track route usage
                        route_name = current_route['name']
                        if route_name not in route_usage:
                            route_usage[route_name] = 0
                        route_usage[route_name] += 1

            # Update display
            if display and sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(
                    sensor_data.rgb_image.swapaxes(0, 1)
                )
                display.blit(surface, (0, 0))

                # Progress bar (route-based)
                progress = routes_completed / total_routes_to_complete if total_routes_to_complete > 0 else 0
                bar_width = WINDOW_WIDTH - 40
                pygame.draw.rect(display, (50, 50, 50), (20, WINDOW_HEIGHT - 30, bar_width, 20))
                pygame.draw.rect(display, (0, 200, 0), (20, WINDOW_HEIGHT - 30, int(bar_width * progress), 20))

                # Text
                font = pygame.font.SysFont('monospace', 12)
                text = font.render(f"Routes: {routes_completed}/{total_routes_to_complete} | Samples: {data_collector.stats['saved']:,} | {speed_kmh:.0f}km/h",
                                   True, (255, 255, 255))
                display.blit(text, (20, WINDOW_HEIGHT - 50))

                pygame.display.flip()
                clock.tick(30)
            else:
                world.tick()
                time.sleep(0.033)

            # Print progress
            if frame % 30 == 0:
                stats = data_collector.get_stats()
                print_progress(
                    routes_completed,
                    total_routes_to_complete,
                    stats['saved'],
                    start_time,
                    speed_kmh,
                    stats['queue_size'],
                    current_route['name'],
                    current_route_loop + 1,
                    loops_per_route
                )

            frame += 1

        print(f"\n\n{'='*70}")
        print("DATA COLLECTION COMPLETE!")
        print(f"{'='*70}")

    finally:
        # Stop collector
        print("\nFinalizing data...")
        data_collector.stop()

        # Save route manifest
        route_manifest = {
            'total_routes': route_loader.get_route_count(),
            'loops_per_route': loops_per_route,
            'total_completions': sum(route_completion_count.values()),
            'route_usage': route_usage,
            'route_completions': route_completion_count,
            'routes': [
                {
                    'name': route['name'],
                    'filepath': route['filepath'],
                    'total_distance': route['total_distance'],
                    'samples_collected': route_usage.get(route['name'], 0),
                    'times_completed': route_completion_count.get(route['name'], 0)
                }
                for route in route_loader.routes
            ]
        }

        route_manifest_path = output_path / 'route_manifest.json'
        with open(route_manifest_path, 'w') as f:
            json.dump(route_manifest, f, indent=2)

        # Print stats
        elapsed = time.time() - start_time
        stats = data_collector.get_stats()
        print(f"\nFinal Statistics:")
        print(f"  Routes completed: {routes_completed}/{total_routes_to_complete}")
        print(f"  Samples saved: {stats['saved']:,}")
        print(f"  Total time: {elapsed/60:.1f} minutes")
        print(f"  Average rate: {stats['saved']/elapsed:.1f} samples/second")

        print(f"\nRoute Completions:")
        for route_name, count in sorted(route_completion_count.items(), key=lambda x: x[1], reverse=True):
            samples = route_usage.get(route_name, 0)
            print(f"  {route_name}: {count}× completed, {samples} samples")

        # Cleanup
        print("\nCleaning up...")
        for actor in actors:
            if actor is not None:
                actor.destroy()

        if display:
            pygame.quit()

        print(f"\n{'='*70}")
        print("Dataset saved to:", output_dir)
        print(f"{'='*70}\n")


def _queue_extended_sample(collector: AsyncDataCollector, sample_data: dict) -> bool:
    """Helper to queue sample with extended metadata."""
    # The base AsyncDataCollector doesn't handle extended fields,
    # so we'll need to monkey-patch or extend the queue_sample call
    # For now, we store extended data in vehicle_state

    extended_vehicle_state = sample_data['vehicle_state'].copy()
    extended_vehicle_state['navigation_command'] = sample_data['navigation_command']
    extended_vehicle_state['navigation_command_id'] = sample_data['navigation_command_id']
    extended_vehicle_state['decision_reasoning'] = sample_data['decision_reasoning']
    extended_vehicle_state['route_name'] = sample_data['route_name']
    extended_vehicle_state['route_metadata'] = sample_data['route_metadata']

    return collector.queue_sample(
        rgb_image=sample_data['rgb_image'],
        waypoints_2d=sample_data['waypoints_2d'],
        waypoints_3d=sample_data['waypoints_3d'],
        kept_indices=sample_data['kept_indices'],
        vehicle_state=extended_vehicle_state,
        control=sample_data['control'],
        timestamp=sample_data['timestamp']
    )


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Route-Based Dataset Recording - Complete Routes Mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record each route once (complete from start to end)
  python record_dataset.py --routes ./routes

  # Record each route 3 times
  python record_dataset.py --routes ./routes --loops 3

  # Record single route multiple times
  python record_dataset.py --route ./routes/route_highway_01.json --loops 5

  # Headless mode
  python record_dataset.py --routes ./routes --loops 2 --no-display

  # Adjust route completion threshold
  python record_dataset.py --routes ./routes --end-threshold 3.0
        """
    )

    parser.add_argument('--routes', type=str,
                        help='Directory containing route JSON files')

    parser.add_argument('--route', type=str,
                        help='Single route JSON file (alternative to --routes)')

    parser.add_argument('--loops', type=int, default=1,
                        help='How many times to complete each route (default: 1)')

    parser.add_argument('--output', type=str, default=None,
                        help='Output directory (default: auto-generated with timestamp)')

    parser.add_argument('--end-threshold', type=float, default=5.0,
                        help='Distance to last checkpoint to consider route complete (default: 5.0m)')

    parser.add_argument('--no-display', action='store_true',
                        help='Run in headless mode (no pygame window)')

    parser.add_argument('--min-speed', type=float, default=15.0,
                        help='Minimum speed to record samples (default: 15.0 km/h)')
    
    parser.add_argument('--target-speed', type=float, default=30.0,
                        help='Target driving speed (default: 30.0 km/h)')

    args = parser.parse_args()

    # Validate arguments
    if not args.routes and not args.route:
        parser.error("Must provide either --routes or --route")

    if args.routes and args.route:
        parser.error("Cannot use both --routes and --route")

    # Calculate info message
    if args.routes:
        route_path = Path(args.routes)
        num_routes = len(list(route_path.glob('*.json')))
    else:
        num_routes = 1

    total_completions = num_routes * args.loops
    print(f"Will complete {total_completions} route runs ({num_routes} routes × {args.loops} loops)\n")

    record_dataset(
        routes_dir=args.routes,
        route_file=args.route,
        output_dir=args.output,
        loops_per_route=args.loops,
        use_display=not args.no_display,
        min_speed_kmh=args.min_speed,
        target_speed_kmh=args.target_speed,
        end_threshold_meters=args.end_threshold
    )
