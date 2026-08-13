#!/usr/bin/env python3
"""
Automated Data Collection Script

Collects a specified number of training samples automatically.
Rotates spawn points for diverse data coverage.

Usage:
    python record_dataset.py --samples 10000
    python record_dataset.py --samples 10000 --output ./my_dataset
    python record_dataset.py --samples 10000 --no-display  # Headless mode
"""

import carla
import numpy as np
import math
import time
import argparse
import signal
import sys

# Import our modules
from traj_planner import (
    CameraConfig, WaypointGenerator, TrajectoryEncoder, TrajectoryDecoder,
    Waypoint3D
)
from pid import PurePursuitController, ControllerConfig, VehicleControl
from data_collector import AsyncDataCollector

# Try to import pygame (optional for display)
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not available - running in headless mode")


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

WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]

CONTROLLER_CONFIG = ControllerConfig(
    target_speed_kmh=45.0,
    max_throttle=0.7,
    max_brake=0.5,
    max_steering=0.7,
    lookahead_distance=8.0,
    speed_kp=0.5,
    speed_ki=0.05,
    speed_kd=0.1
)


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

def print_progress(current, total, start_time, speed_kmh, queue_size, spawn_idx, total_spawns):
    """Print progress bar and stats."""
    elapsed = time.time() - start_time
    percent = current / total * 100
    rate = current / elapsed if elapsed > 0 else 0
    eta = (total - current) / rate if rate > 0 else 0
    
    bar_width = 30
    filled = int(bar_width * current / total)
    bar = '█' * filled + '░' * (bar_width - filled)
    
    sys.stdout.write(f'\r[{bar}] {percent:5.1f}% | '
                     f'{current:,}/{total:,} | '
                     f'{rate:.1f}/s | '
                     f'ETA: {int(eta//60)}m{int(eta%60):02d}s | '
                     f'Speed: {speed_kmh:.0f}km/h | '
                     f'Q: {queue_size} | '
                     f'Spawn: {spawn_idx+1}/{total_spawns}')
    sys.stdout.flush()


# =============================================================================
# MAIN RECORDING FUNCTION
# =============================================================================

def record_dataset(target_samples: int,
                   output_dir: str,
                   use_display: bool = True,
                   spawn_rotation_interval: int = 200,
                   min_speed_kmh: float = 3.0,
                   random_routes: bool = True):
    """
    Record a dataset with specified number of samples.

    Args:
        target_samples: Number of samples to collect
        output_dir: Output directory
        use_display: Whether to show pygame window
        spawn_rotation_interval: Change spawn point every N samples (default: 200 for more diversity)
        min_speed_kmh: Minimum speed to record (avoids stopped frames)
        random_routes: If True, randomly select routes at intersections for diverse turns
    """
    
    # Initialize display if requested
    display = None
    clock = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption(f'Recording Dataset - Target: {target_samples:,} samples')
        clock = pygame.time.Clock()
    
    # Connect to CARLA
    print(f"\n{'='*60}")
    print(f"AUTOMATED DATA COLLECTION")
    print(f"{'='*60}")
    print(f"Target samples: {target_samples:,}")
    print(f"Output: {output_dir}")
    print(f"Display: {'ON' if display else 'OFF'}")
    print(f"{'='*60}\n")
    
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    carla_map = world.get_map()
    
    # Get all spawn points
    spawn_points = carla_map.get_spawn_points()
    np.random.shuffle(spawn_points)
    print(f"Found {len(spawn_points)} spawn points")
    
    # Initialize components
    waypoint_gen = WaypointGenerator(carla_map, distances=WAYPOINT_DISTANCES, random_route=random_routes)
    encoder = TrajectoryEncoder(CAMERA_CONFIG)
    decoder = TrajectoryDecoder(CAMERA_CONFIG)
    controller = PurePursuitController(CONTROLLER_CONFIG)

    if random_routes:
        print("Random route selection: ENABLED (diverse turns at intersections)")
    
    # Initialize async data collector
    data_collector = AsyncDataCollector(
        output_dir=output_dir,
        max_queue_size=500,
        save_interval=1  # Save every frame that meets criteria
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
    current_spawn_idx = 0
    
    # Signal handler for graceful shutdown
    shutdown_requested = False
    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        print("\n\nShutdown requested... finishing current batch")
        shutdown_requested = True
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Spawn vehicle
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        spawn_point = spawn_points[current_spawn_idx]
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actors.append(vehicle)
        print(f"Vehicle spawned at spawn point {current_spawn_idx}")
        
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
        
        time.sleep(1.0)  # Wait for sensors
        print("Starting data collection...\n")

        start_time = time.time()
        frame = 0
        stuck_counter = 0
        last_position = None
        last_respawn_at = 0  # Track last respawn sample count

        while data_collector.stats['saved'] < target_samples and not shutdown_requested:
            # Handle pygame events if display is active
            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            shutdown_requested = True
            
            # Get vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            # Check if stuck
            current_pos = (transform.location.x, transform.location.y)
            if last_position:
                dist_moved = math.sqrt((current_pos[0] - last_position[0])**2 + 
                                       (current_pos[1] - last_position[1])**2)
                if dist_moved < 0.1 and speed_kmh < 1.0:
                    stuck_counter += 1
                else:
                    stuck_counter = 0
            last_position = current_pos
            
            # Respawn if stuck for too long
            if stuck_counter > 200:  # ~6 seconds (give time to accelerate)
                print(f"\nVehicle stuck, respawning...")
                # Random spawn point for diversity
                current_spawn_idx = np.random.randint(0, len(spawn_points))
                vehicle.set_transform(spawn_points[current_spawn_idx])
                controller.reset()
                waypoint_gen.reset_route_choice()
                stuck_counter = 0
                last_respawn_at = data_collector.stats['saved']  # Reset interval counter
                time.sleep(0.5)
                continue

            # Rotate spawn point periodically for diversity
            current_saved = data_collector.stats['saved']
            if current_saved > 0 and \
               current_saved >= last_respawn_at + spawn_rotation_interval:
                # Random spawn point instead of sequential
                current_spawn_idx = np.random.randint(0, len(spawn_points))
                vehicle.set_transform(spawn_points[current_spawn_idx])
                controller.reset()
                waypoint_gen.reset_route_choice()
                last_respawn_at = current_saved
                print(f"\n[Random respawn at sample {current_saved} to point {current_spawn_idx}/{len(spawn_points)}]")
                time.sleep(0.5)
                continue
            
            # Generate trajectory
            waypoints_3d_original = waypoint_gen.get_waypoints(vehicle)
            waypoints_2d, kept_indices = encoder.encode_with_indices(waypoints_3d_original, transform)
            
            # Decode for control
            if kept_indices and waypoints_3d_original:
                road_height = sum(waypoints_3d_original[i].z for i in kept_indices) / len(kept_indices)
            else:
                road_height = transform.location.z - 0.5
            
            waypoints_3d_decoded = decoder.decode(waypoints_2d, transform, road_height)
            waypoints_3d_kept = [waypoints_3d_original[i] for i in kept_indices] if kept_indices else []
            waypoints_3d = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d_original
            
            # Compute and apply control
            control = controller.compute_control(waypoints_3d, transform, speed_kmh)
            vehicle.apply_control(control.to_carla_control())
            
            # Collect data if conditions met
            if sensor_data.rgb_image is not None:
                # Debug: show why not recording (every 2 seconds)
                if frame % 60 == 0 and speed_kmh < min_speed_kmh:
                    print(f"\n[DEBUG] Not recording: speed={speed_kmh:.1f} < min_speed={min_speed_kmh}")

                if speed_kmh >= min_speed_kmh and waypoints_2d and len(waypoints_2d) >= 5:
                    vehicle_state = {
                        'speed_kmh': speed_kmh,
                        'location': [transform.location.x, transform.location.y, transform.location.z],
                        'rotation': [transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]
                    }
                    
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
            
            # Update display
            if display and sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(
                    sensor_data.rgb_image.swapaxes(0, 1)
                )
                display.blit(surface, (0, 0))
                
                # Draw progress bar on display
                progress = data_collector.stats['saved'] / target_samples
                bar_width = WINDOW_WIDTH - 40
                pygame.draw.rect(display, (50, 50, 50), (20, WINDOW_HEIGHT - 30, bar_width, 20))
                pygame.draw.rect(display, (0, 200, 0), (20, WINDOW_HEIGHT - 30, int(bar_width * progress), 20))
                
                # Draw text
                font = pygame.font.SysFont('monospace', 14)
                text = font.render(f"{data_collector.stats['saved']:,}/{target_samples:,} samples | {speed_kmh:.0f} km/h", 
                                   True, (255, 255, 255))
                display.blit(text, (20, WINDOW_HEIGHT - 50))
                
                pygame.display.flip()
                clock.tick(30)
            else:
                # Headless mode - just tick the world
                world.tick()
                time.sleep(0.033)  # ~30 FPS
            
            # Print progress
            if frame % 30 == 0:  # Update every second
                stats = data_collector.get_stats()
                print_progress(
                    stats['saved'], 
                    target_samples, 
                    start_time, 
                    speed_kmh,
                    stats['queue_size'],
                    current_spawn_idx,
                    len(spawn_points)
                )
            
            frame += 1
        
        # Collection complete
        print(f"\n\n{'='*60}")
        print("DATA COLLECTION COMPLETE!")
        print(f"{'='*60}")
        
    finally:
        # Stop collector
        print("\nFinalizing data...")
        data_collector.stop()
        
        # Print final stats
        elapsed = time.time() - start_time
        stats = data_collector.get_stats()
        print(f"\nFinal Statistics:")
        print(f"  Samples saved: {stats['saved']:,}")
        print(f"  Samples dropped: {stats.get('dropped', 0):,}")
        print(f"  Total time: {elapsed/60:.1f} minutes")
        print(f"  Average rate: {stats['saved']/elapsed:.1f} samples/second")
        
        # Cleanup
        print("\nCleaning up...")
        for actor in actors:
            if actor is not None:
                actor.destroy()
        
        if display:
            pygame.quit()
        
        # Print next steps
        print(f"\n{'='*60}")
        print("NEXT STEPS:")
        print(f"{'='*60}")
        print(f"1. Convert .npz to JPEG:")
        print(f"   python data_collector.py --convert {output_dir}")
        print(f"\n2. Create train/val split:")
        print(f"   python data_collector.py --split {output_dir}")
        print(f"\n3. Verify dataset:")
        print(f"   python data_collector.py --verify {output_dir}")
        print(f"{'='*60}\n")


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Automated Dataset Recording')
    parser.add_argument('--samples', type=int, default=10000,
                        help='Number of samples to collect (default: 10000)')
    parser.add_argument('--output', type=str, default='./trajectory_data',
                        help='Output directory (default: ./trajectory_data)')
    parser.add_argument('--no-display', action='store_true',
                        help='Run in headless mode (no pygame window)')
    parser.add_argument('--spawn-interval', type=int, default=200,
                        help='Rotate spawn point every N samples (default: 200)')
    parser.add_argument('--min-speed', type=float, default=15.0,
                        help='Minimum speed to record samples (default: 15.0 km/h)')
    parser.add_argument('--no-random-routes', action='store_true',
                        help='Disable random route selection at intersections')

    args = parser.parse_args()

    record_dataset(
        target_samples=args.samples,
        output_dir=args.output,
        use_display=not args.no_display,
        spawn_rotation_interval=args.spawn_interval,
        min_speed_kmh=args.min_speed,
        random_routes=not args.no_random_routes
    )
