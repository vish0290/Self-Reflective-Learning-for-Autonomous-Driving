#!/usr/bin/env python3
"""
Complete Trajectory Following System for CARLA

PIPELINE ARCHITECTURE:
======================
1. WaypointGenerator (traj_planner.py)
   - Get 3D waypoints along road from CARLA map
   - Output: List[Waypoint3D] in world coordinates

2. TrajectoryEncoder (traj_planner.py) - 3D to 2D
   - Project 3D waypoints to 2D pixel coordinates
   - Uses camera intrinsics + extrinsics
   - Output: List[Waypoint2D] in pixel coordinates
   - This is what gets saved for VLM training

3. TrajectoryDecoder (traj_planner.py) - 2D to 3D  
   - Decode 2D pixels back to 3D world coordinates
   - Uses ray-plane intersection (assumes flat road)
   - Output: List[Waypoint3D] in world coordinates
   - This simulates VLM inference output

4. PID Controller (pid.py)
   - Follow DECODED 3D trajectory
   - Pure Pursuit / Stanley algorithms
   - Output: VehicleControl (throttle, brake, steer)

Three modes:
- DATA COLLECTION: Generate ground truth trajectories for VLM training
- VLM INFERENCE: Follow VLM-predicted trajectories (VLM_ENABLED=True)
- GROUND TRUTH: Follow map-based trajectories (VLM_ENABLED=False)

VLM Pipeline: Camera Frame → VLM → 2D trajectory → Decoder → 3D → PID
The VLM sees the camera image and predicts 2D waypoints, which are
decoded to 3D world coordinates for the PID controller.
"""

import carla
import numpy as np
import pygame
import math
import time
import json
from pathlib import Path
from datetime import datetime

# Import our modules
from traj_planner import (
    CameraConfig, WaypointGenerator, TrajectoryEncoder, TrajectoryDecoder,
    Waypoint2D, Waypoint3D, waypoints_to_pixel_list, pixel_list_to_waypoints
)
from pid import PurePursuitController, ControllerConfig, VehicleControl
from data_collector import AsyncDataCollector
from vlm_inference import AsyncVLMPredictor, trajectory_to_waypoints_2d


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

# WINDOW_WIDTH = 960
# WINDOW_HEIGHT = 540

# Camera configuration
CAMERA_CONFIG = CameraConfig(
    width=WINDOW_WIDTH,
    height=WINDOW_HEIGHT,
    fov=90,
    x=2.0,      # Forward from vehicle center
    y=0.0,      # Centered
    z=1.8,      # Height
    pitch=-15,  # Tilt down to see road
    yaw=0,
    roll=0
)

# Waypoint configuration
WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]  # meters

# Controller configuration
CONTROLLER_CONFIG = ControllerConfig(
    target_speed_kmh=35.0,
    max_throttle=0.7,
    max_brake=0.5,
    max_steering=0.7,
    lookahead_distance=8.0,
    speed_kp=0.5,
    speed_ki=0.05,
    speed_kd=0.1
)

# Data collection
SAVE_INTERVAL = 5  # Save every N frames
OUTPUT_DIR = './trajectory_data'

# VLM Configuration
VLM_API_URL = "https://vishwanaths-mac-mini.woodpecker-bluegill.ts.net/v1"
# VLM_API_URL = "http://100.85.159.60:8000/v1'"
# VLM_API_URL = "https://vish2kber--vlm-inference-serve.modal.run/v1"
# VLM_MODEL_NAME = "qwen3-vl-driver_v2"  # Change to your model name
VLM_MODEL_NAME = "qwen3-vl-driver-base"  # Change to your model name
# VLM_MODEL_NAME = "VishwanathAS/Qwen3-VLA-Driver"  # Change to your model name
VLM_ENABLED = True  # Set to True to use VLM inference


# =============================================================================
# SENSOR DATA
# =============================================================================

class SensorData:
    """Storage for sensor data."""
    def __init__(self):
        self.rgb_image = None
        self.timestamp = 0

sensor_data = SensorData()


def process_rgb(image):
    """Process RGB camera image."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    sensor_data.rgb_image = array[:, :, ::-1].copy()  # BGR to RGB
    sensor_data.timestamp = image.timestamp


# =============================================================================
# VISUALIZATION
# =============================================================================

def draw_trajectory_line(surface, waypoints_2d, color, thickness=3):
    """Draw 2D trajectory as a simple line."""
    if not waypoints_2d or len(waypoints_2d) < 2:
        return
    
    points = [(int(wp.u), int(wp.v)) for wp in waypoints_2d]
    
    # Draw connected lines
    for i in range(len(points) - 1):
        pygame.draw.line(surface, color, points[i], points[i+1], thickness)


def draw_both_trajectories(surface, waypoints_gt, waypoints_vlm):
    """
    Draw both ground truth and VLM trajectories.
    Green = Ground Truth
    Blue = VLM prediction
    """
    # Ground truth - GREEN
    draw_trajectory_line(surface, waypoints_gt, (0, 255, 0), thickness=3)
    
    # VLM prediction - BLUE  
    draw_trajectory_line(surface, waypoints_vlm, (0, 150, 255), thickness=3)


def draw_2d_coordinates(surface, waypoints_2d, distances, font):
    """Draw 2D pixel coordinates next to each waypoint."""
    if not waypoints_2d:
        return
    
    for i, wp in enumerate(waypoints_2d):
        x, y = int(wp.u), int(wp.v)
        
        # Get distance for this waypoint
        dist = distances[i] if i < len(distances) else 0
        
        # Draw coordinate label with background
        label = f"({x}, {y}) {dist}m"
        
        # Offset label to avoid overlapping with the circle
        label_x = x + 12
        label_y = y - 8
        
        # Keep label within screen bounds
        if label_x > WINDOW_WIDTH - 100:
            label_x = x - 90
        if label_y < 10:
            label_y = y + 15
        
        # Draw background rectangle
        text_surface = font.render(label, True, (255, 255, 255))
        text_rect = text_surface.get_rect()
        text_rect.topleft = (label_x, label_y)
        
        bg_rect = text_rect.inflate(6, 4)
        pygame.draw.rect(surface, (0, 0, 0, 180), bg_rect)
        pygame.draw.rect(surface, (100, 100, 100), bg_rect, 1)
        
        # Draw text
        surface.blit(text_surface, text_rect)


def draw_2d_coords_panel(surface, waypoints_2d, distances, font):
    """Draw a panel showing all 2D pixel coordinates."""
    if not waypoints_2d:
        return
    
    # Panel on the right side
    panel_x = WINDOW_WIDTH - 200
    panel_y = 250
    panel_width = 190
    panel_height = 20 + len(waypoints_2d) * 22
    
    # Draw panel background
    panel = pygame.Surface((panel_width, panel_height))
    panel.set_alpha(200)
    panel.fill((0, 0, 0))
    surface.blit(panel, (panel_x, panel_y))
    
    # Title
    title = font.render("2D Pixel Coordinates:", True, (0, 255, 255))
    surface.blit(title, (panel_x + 5, panel_y + 5))
    
    # List coordinates
    y_offset = panel_y + 25
    for i, wp in enumerate(waypoints_2d):
        dist = distances[i] if i < len(distances) else 0
        text = font.render(f"{i+1}: ({int(wp.u):4d}, {int(wp.v):3d}) {dist}m", 
                          True, (200, 200, 200))
        surface.blit(text, (panel_x + 10, y_offset))
        y_offset += 20


def draw_waypoints_3d_debug(surface, waypoints_3d, vehicle_transform, font):
    """Draw 3D waypoint info for debugging."""
    if not waypoints_3d:
        return
    
    v_loc = vehicle_transform.location
    
    y_offset = 250
    text = font.render("Decoded 3D Waypoints:", True, (255, 255, 0))
    surface.blit(text, (10, y_offset))
    y_offset += 20
    
    for i, wp in enumerate(waypoints_3d[:5]):  # Show first 5
        dist = math.sqrt((wp.x - v_loc.x)**2 + (wp.y - v_loc.y)**2)
        text = font.render(f"  {i+1}: ({wp.x:.1f}, {wp.y:.1f}) d={dist:.1f}m", 
                          True, (200, 200, 200))
        surface.blit(text, (10, y_offset))
        y_offset += 18


def draw_pipeline_debug(surface, original_3d, decoded_3d, font):
    """Draw comparison between original and decoded 3D waypoints."""
    if not original_3d or not decoded_3d:
        return
    
    y_offset = 370
    text = font.render("Pipeline Accuracy (3D→2D→3D):", True, (0, 255, 255))
    surface.blit(text, (10, y_offset))
    y_offset += 20
    
    # Compare original vs decoded
    min_len = min(len(original_3d), len(decoded_3d))
    total_error = 0.0
    
    for i in range(min(min_len, 5)):
        orig = original_3d[i]
        dec = decoded_3d[i]
        error = math.sqrt((orig.x - dec.x)**2 + (orig.y - dec.y)**2)
        total_error += error
        
        color = (0, 255, 0) if error < 0.5 else ((255, 255, 0) if error < 1.0 else (255, 100, 100))
        text = font.render(f"  {i+1}: err={error:.2f}m", True, color)
        surface.blit(text, (10, y_offset))
        y_offset += 18
    
    if min_len > 0:
        avg_error = total_error / min(min_len, 5)
        color = (0, 255, 0) if avg_error < 0.5 else ((255, 255, 0) if avg_error < 1.0 else (255, 100, 100))
        text = font.render(f"  Avg: {avg_error:.2f}m", True, color)
        surface.blit(text, (10, y_offset))


def draw_info_panel(surface, info, font):
    """Draw information panel."""
    panel = pygame.Surface((300, 230))
    panel.set_alpha(200)
    panel.fill((0, 0, 0))
    surface.blit(panel, (10, 10))
    
    y = 15
    for key, value in info.items():
        if isinstance(value, float):
            text = f"{key}: {value:.2f}"
        else:
            text = f"{key}: {value}"
        text_surface = font.render(text, True, (255, 255, 255))
        surface.blit(text_surface, (20, y))
        y += 22


def draw_control_indicator(surface, control, font):
    """Draw steering/throttle/brake indicator."""
    x, y = WINDOW_WIDTH - 150, WINDOW_HEIGHT - 120
    
    # Background
    pygame.draw.rect(surface, (30, 30, 30), (x-10, y-10, 140, 110))
    
    # Steering bar
    pygame.draw.rect(surface, (50, 50, 50), (x, y, 120, 20))
    steer_x = x + 60 + int(control.steer * 55)
    pygame.draw.rect(surface, (0, 200, 200), (steer_x - 5, y, 10, 20))
    pygame.draw.line(surface, (255, 255, 255), (x + 60, y), (x + 60, y + 20), 1)
    
    # Throttle bar
    throttle_height = int(control.throttle * 50)
    pygame.draw.rect(surface, (50, 50, 50), (x, y + 30, 50, 50))
    pygame.draw.rect(surface, (0, 200, 0), (x, y + 80 - throttle_height, 50, throttle_height))
    
    # Brake bar
    brake_height = int(control.brake * 50)
    pygame.draw.rect(surface, (50, 50, 50), (x + 70, y + 30, 50, 50))
    pygame.draw.rect(surface, (200, 0, 0), (x + 70, y + 80 - brake_height, 50, brake_height))
    
    # Labels
    text = font.render("Steer", True, (150, 150, 150))
    surface.blit(text, (x + 40, y - 5))
    text = font.render("T", True, (150, 150, 150))
    surface.blit(text, (x + 20, y + 82))
    text = font.render("B", True, (150, 150, 150))
    surface.blit(text, (x + 90, y + 82))


# =============================================================================
# DATA SAVING
# =============================================================================

class DataSaver:
    """Save training data."""
    
    def __init__(self, output_dir):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'images').mkdir(exist_ok=True)
        (self.output_dir / 'labels').mkdir(exist_ok=True)
        
        self.count = 0
        self.manifest = []
    
    def save(self, rgb_image, waypoints_2d, waypoints_3d, 
             vehicle_state, control):
        """Save a training sample."""
        sample_id = f"{self.count:06d}"
        
        # Save image
        from PIL import Image
        img_path = self.output_dir / 'images' / f"{sample_id}.jpg"
        Image.fromarray(rgb_image).save(img_path, quality=95)
        
        # Create label
        label = {
            'id': sample_id,
            'timestamp': datetime.now().isoformat(),
            
            # 2D trajectory (what VLM should output)
            'trajectory_2d': waypoints_to_pixel_list(waypoints_2d),
            
            # 3D trajectory (for reference/validation)
            'trajectory_3d': [[wp.x, wp.y, wp.z] for wp in waypoints_3d],
            
            # Vehicle state (input context)
            'vehicle_state': vehicle_state,
            
            # Control output (for imitation learning)
            'control': {
                'throttle': control.throttle,
                'brake': control.brake,
                'steer': control.steer
            },
            
            # Camera config (for decoding)
            'camera': {
                'width': CAMERA_CONFIG.width,
                'height': CAMERA_CONFIG.height,
                'fov': CAMERA_CONFIG.fov,
                'position': [CAMERA_CONFIG.x, CAMERA_CONFIG.y, CAMERA_CONFIG.z],
                'rotation': [CAMERA_CONFIG.pitch, CAMERA_CONFIG.yaw, CAMERA_CONFIG.roll]
            }
        }
        
        # Save label
        label_path = self.output_dir / 'labels' / f"{sample_id}.json"
        with open(label_path, 'w') as f:
            json.dump(label, f, indent=2)
        
        self.manifest.append(sample_id)
        self.count += 1
        
        return sample_id
    
    def save_manifest(self):
        """Save manifest file."""
        manifest_path = self.output_dir / 'manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump({
                'total_samples': len(self.manifest),
                'samples': self.manifest,
                'waypoint_distances': WAYPOINT_DISTANCES,
                'camera_config': {
                    'width': CAMERA_CONFIG.width,
                    'height': CAMERA_CONFIG.height,
                    'fov': CAMERA_CONFIG.fov
                }
            }, f, indent=2)
        print(f"\nSaved manifest: {len(self.manifest)} samples")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main function."""
    pygame.init()
    font = pygame.font.SysFont('monospace', 16)
    font_small = pygame.font.SysFont('monospace', 14)
    
    display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('CARLA Trajectory Following')
    clock = pygame.time.Clock()
    
    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    world = client.load_world('Town10HD')
    bp_lib = world.get_blueprint_library()
    carla_map = world.get_map()
    
    # Initialize components
    waypoint_gen = WaypointGenerator(carla_map, distances=WAYPOINT_DISTANCES)
    encoder = TrajectoryEncoder(CAMERA_CONFIG)
    decoder = TrajectoryDecoder(CAMERA_CONFIG)
    controller = PurePursuitController(CONTROLLER_CONFIG)
    
    # Initialize VLM predictor (if enabled) - uses async/cached inference
    vlm_predictor = None
    if VLM_ENABLED:
        print(f"Initializing async VLM predictor ({VLM_API_URL})...")
        vlm_predictor = AsyncVLMPredictor(
            base_url=VLM_API_URL,
            model=VLM_MODEL_NAME
        )
        print("VLM predictor ready (async mode - predictions are cached)")
    
    # Async data collector (non-blocking!)
    data_collector = AsyncDataCollector(
        output_dir=OUTPUT_DIR,
        max_queue_size=200,
        save_interval=SAVE_INTERVAL
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
    
    actors = []
    collecting = False
    manual_control = False
    show_3d_debug = False
    show_2d_coords = False
    use_vlm = VLM_ENABLED  # Toggle VLM inference
    vlm_inference_time = 0.0

    # Navigation instruction for VLM (derived from trajectory or manual override with N key)
    nav_options = [None, "go straight", "turn left", "turn right"]  # Manual override options
    nav_index = 0
    manual_nav_override = False  # If True, use manual nav_options; if False, derive from trajectory
    current_navigation = None

    # Spawn point selection
    spawn_points = carla_map.get_spawn_points()
    spawn_index = 0

    try:
        # Spawn vehicle
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        spawn_point = spawn_points[spawn_index]
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actors.append(vehicle)
        print(f"Vehicle spawned at {spawn_point.location}")
        
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
        print("Camera attached")
        
        time.sleep(1.0)  # Wait for sensors
        
        # Main loop
        frame = 0
        running = True
        
        print("\nControls:")
        print("  SPACE - Start/Stop data collection")
        print("  M - Toggle manual control (WASD)")
        print("  V - Toggle VLM inference (vs ground truth)")
        print("  N - Toggle navigation mode: Auto (from trajectory) / Manual override")
        print("  D - Toggle 3D debug info")
        print("  P - Toggle 2D pixel coordinates")
        print("  R - Reset vehicle to current spawn point")
        print("  [ / ] - Cycle spawn points (prev/next)")
        print("  ESC - Quit")
        print(f"\nSpawn point: {spawn_index}/{len(spawn_points)-1}")
        print(f"\nVLM Mode: {'ENABLED' if use_vlm else 'DISABLED'}")
        print("\nAfter collecting, run:")
        print("  python data_collector.py --convert ./trajectory_data")
        
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        collecting = not collecting
                        if collecting:
                            data_collector.start()
                            print(f"Data collection: ON (async, saving to {OUTPUT_DIR})")
                        else:
                            data_collector.stop()
                            print(f"Data collection: OFF")
                    elif event.key == pygame.K_m:
                        manual_control = not manual_control
                        print(f"Manual control: {'ON' if manual_control else 'OFF'}")
                    elif event.key == pygame.K_v:
                        if vlm_predictor is not None:
                            use_vlm = not use_vlm
                            print(f"VLM inference: {'ON' if use_vlm else 'OFF (using ground truth)'}")
                        else:
                            print("VLM not available (check VLM_ENABLED and server)")
                    elif event.key == pygame.K_d:
                        show_3d_debug = not show_3d_debug
                    elif event.key == pygame.K_p:
                        show_2d_coords = not show_2d_coords
                        print(f"2D coordinates: {'ON' if show_2d_coords else 'OFF'}")
                    elif event.key == pygame.K_r:
                        vehicle.set_transform(spawn_point)
                        controller.reset()
                        waypoint_gen.reset_route_choice()
                        print(f"Vehicle reset to spawn {spawn_index}")
                    elif event.key == pygame.K_LEFTBRACKET:
                        spawn_index = (spawn_index - 1) % len(spawn_points)
                        spawn_point = spawn_points[spawn_index]
                        vehicle.set_transform(spawn_point)
                        controller.reset()
                        waypoint_gen.reset_route_choice()
                        print(f"Spawn point: {spawn_index}/{len(spawn_points)-1}")
                    elif event.key == pygame.K_RIGHTBRACKET:
                        spawn_index = (spawn_index + 1) % len(spawn_points)
                        spawn_point = spawn_points[spawn_index]
                        vehicle.set_transform(spawn_point)
                        controller.reset()
                        waypoint_gen.reset_route_choice()
                        print(f"Spawn point: {spawn_index}/{len(spawn_points)-1}")
                    elif event.key == pygame.K_n:
                        manual_nav_override = not manual_nav_override
                        if manual_nav_override:
                            nav_index = 1  # Start at "go straight" for manual
                            current_navigation = nav_options[nav_index]
                            print(f"Navigation: MANUAL - {current_navigation}")
                        else:
                            print("Navigation: AUTO (derived from trajectory)")
            
            # Get vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            vehicle_state = {
                'speed_kmh': speed_kmh,
                'location': [transform.location.x, transform.location.y, transform.location.z],
                'rotation': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
            }
            
            # =================================================================
            # TRAJECTORY PIPELINE:
            # 
            # VLM MODE (use_vlm=True):
            #   Camera Frame → VLM → 2D trajectory → Decoder → 3D → PID
            # 
            # GROUND TRUTH MODE (use_vlm=False):
            #   CARLA Map → 3D waypoints → Encoder → 2D → Decoder → 3D → PID
            # =================================================================
            
            # Always get ground truth for comparison and data collection
            waypoints_3d_original = waypoint_gen.get_waypoints(vehicle)
            waypoints_2d_gt, kept_indices = encoder.encode_with_indices(waypoints_3d_original, transform)
            waypoints_3d_kept = [waypoints_3d_original[i] for i in kept_indices] if kept_indices else []

            # Derive navigation from trajectory (unless manual override is enabled)
            if not manual_nav_override:
                current_navigation = waypoint_gen.get_navigation_from_waypoints(waypoints_3d_original, vehicle)
            
            # Estimate road height for decoding
            if kept_indices and waypoints_3d_original:
                road_height = sum(waypoints_3d_original[i].z for i in kept_indices) / len(kept_indices)
            else:
                road_height = transform.location.z - 0.5  # Fallback estimate
            
            # Choose trajectory source: VLM or Ground Truth
            waypoints_2d_vlm = []  # Track VLM predictions separately
            
            if use_vlm and vlm_predictor is not None and sensor_data.rgb_image is not None:
                # =============================================================
                # VLM INFERENCE MODE (ASYNC/CACHED)
                # Camera Frame → VLM → 2D trajectory → Decoder → 3D → PID
                # =============================================================

                # Submit current frame for async inference with speed and navigation context
                vlm_predictor.submit_image(
                    sensor_data.rgb_image,
                    current_speed=speed_kmh,
                    navigation=current_navigation
                )

                # Get cached trajectory (non-blocking)
                trajectory_raw, vlm_inference_time, is_new = vlm_predictor.get_trajectory()

                if trajectory_raw and len(trajectory_raw) > 0:
                    # Convert VLM output [[x, y, dist], ...] to Waypoint2D objects
                    waypoints_2d_vlm = trajectory_to_waypoints_2d(trajectory_raw)
                    waypoints_2d = waypoints_2d_vlm

                    # Decode VLM's 2D predictions to 3D world coordinates
                    waypoints_3d_decoded = decoder.decode(waypoints_2d, transform, road_height)
                    waypoints_3d = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d_original
                else:
                    # No VLM prediction yet, fall back to ground truth
                    waypoints_2d = waypoints_2d_gt
                    waypoints_3d_decoded = decoder.decode(waypoints_2d, transform, road_height)
                    waypoints_3d = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d_original
            else:
                # =============================================================
                # GROUND TRUTH MODE (for data collection or comparison)
                # CARLA Map → 3D → Encoder → 2D → Decoder → 3D → PID
                # =============================================================
                waypoints_2d = waypoints_2d_gt
                waypoints_3d_decoded = decoder.decode(waypoints_2d, transform, road_height)
                waypoints_3d = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d_original
                vlm_inference_time = 0.0

            # Compute control
            if manual_control:
                # Manual control with keyboard
                keys = pygame.key.get_pressed()
                control = VehicleControl(
                    throttle=0.5 if keys[pygame.K_w] else 0.0,
                    brake=0.5 if keys[pygame.K_s] else 0.0,
                    steer=-0.5 if keys[pygame.K_a] else (0.5 if keys[pygame.K_d] else 0.0)
                )
            else:
                # Automatic control using PID with DECODED trajectory
                control = controller.compute_control(
                    waypoints_3d, transform, speed_kmh
                )
            
            # Apply control
            vehicle.apply_control(control.to_carla_control())
            
            # Queue data for async saving (non-blocking!)
            if collecting and sensor_data.rgb_image is not None:
                if speed_kmh > 3.0 and waypoints_2d and len(waypoints_2d) >= 5:
                    data_collector.queue_sample(
                        rgb_image=sensor_data.rgb_image,
                        waypoints_2d=waypoints_2d,
                        waypoints_3d=waypoints_3d_kept,
                        kept_indices=kept_indices,
                        vehicle_state=vehicle_state,
                        control={
                            'throttle': control.throttle,
                            'brake': control.brake,
                            'steer': control.steer
                        },
                        timestamp=sensor_data.timestamp
                    )
            
            # Render
            if sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(
                    sensor_data.rgb_image.swapaxes(0, 1)
                )
                display.blit(surface, (0, 0))
                
                # Draw both trajectories: Green = GT, Blue = VLM
                draw_both_trajectories(display, waypoints_2d_gt, waypoints_2d_vlm)
                
                # Get collector stats
                collector_stats = data_collector.get_stats() if collecting else {'saved': 0, 'queue_size': 0}
                
                # Draw info
                nav_display = current_navigation if current_navigation else 'None'
                nav_mode = 'Manual' if manual_nav_override else 'Auto'
                info = {
                    'Speed': f"{speed_kmh:.1f} km/h",
                    'Throttle': f"{control.throttle:.2f}",
                    'Brake': f"{control.brake:.2f}",
                    'Steer': f"{control.steer:.2f}",
                    'Waypoints': f"{len(waypoints_2d)}/{len(waypoints_3d_original)}",
                    'Mode': 'VLM' if use_vlm else 'Ground Truth',
                    'Nav': f"{nav_display} ({nav_mode})",
                    'VLM Time': f"{vlm_inference_time*1000:.0f}ms" if use_vlm else 'N/A',
                    'Collecting': 'YES' if collecting else 'NO',
                    'Saved': collector_stats['saved'],
                }
                draw_info_panel(display, info, font)
                
                # Draw control indicator
                draw_control_indicator(display, control, font_small)
                
                # Draw 3D debug
                if show_3d_debug:
                    draw_waypoints_3d_debug(display, waypoints_3d_decoded, transform, font_small)
                    # Compare KEPT originals vs decoded (correct index matching!)
                    draw_pipeline_debug(display, waypoints_3d_kept, waypoints_3d_decoded, font_small)
                
                # Draw 2D pixel coordinates
                if show_2d_coords:
                    # Get distances for visible waypoints only
                    visible_distances = [WAYPOINT_DISTANCES[i] for i in kept_indices] if kept_indices else []
                    draw_2d_coordinates(display, waypoints_2d, visible_distances, font_small)
                    draw_2d_coords_panel(display, waypoints_2d, visible_distances, font_small)
                
                # Recording indicator
                
                # Draw control indicator
                draw_control_indicator(display, control, font_small)
                
                # Draw 3D debug
                if show_3d_debug:
                    draw_waypoints_3d_debug(display, waypoints_3d_decoded, transform, font_small)
                    # Compare KEPT originals vs decoded (correct index matching!)
                    draw_pipeline_debug(display, waypoints_3d_kept, waypoints_3d_decoded, font_small)
                
                # Draw 2D pixel coordinates
                if show_2d_coords:
                    # Get distances for visible waypoints only
                    visible_distances = [WAYPOINT_DISTANCES[i] for i in kept_indices] if kept_indices else []
                    draw_2d_coordinates(display, waypoints_2d, visible_distances, font_small)
                    draw_2d_coords_panel(display, waypoints_2d, visible_distances, font_small)
                
                # Recording indicator
                if collecting:
                    pygame.draw.circle(display, (255, 0, 0), (WINDOW_WIDTH - 30, 30), 12)
                
                # Mode indicator
                if manual_control:
                    pass
                    # mode_text = "MANUAL"
                    # mode_color = (255, 255, 0)  # Yellow for manual
                elif use_vlm:
                    mode_text = "VLM"
                    mode_color = (0, 200, 255)  # Cyan for VLM
                else:
                    mode_text = "AUTO"
                    mode_color = (0, 255, 0)
                text = font.render(mode_text, True, mode_color)
                display.blit(text, (WINDOW_WIDTH - 80, 60))
            
            pygame.display.flip()
            clock.tick(30)
            frame += 1
    
    finally:
        # Stop VLM predictor thread
        if vlm_predictor is not None:
            vlm_predictor.stop()
            print("VLM predictor stopped")
        
        # Stop data collector (waits for queue to drain)
        if collecting or data_collector.stats['saved'] > 0:
            data_collector.stop()
            print("\n" + "="*50)
            print("To convert .npz to JPEG, run:")
            print(f"  python data_collector.py --convert {OUTPUT_DIR}")
            print("="*50)
        
        # Cleanup
        print("\nCleaning up...")
        for actor in actors:
            actor.destroy()
        pygame.quit()
        print("Done!")


if __name__ == '__main__':
    main()