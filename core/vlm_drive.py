#!/usr/bin/env python3
"""
Unified VLM Autonomous Driving

Two modes:
1. OPEN MAP MODE: Free driving, no routes
   - VLM gets: image + speed + prompt ("follow traffic rules")
   - No navigation commands
   - Vehicle drives freely based on VLM vision

2. ROUTE MODE: Follows predefined routes
   - VLM gets: image + speed + navigation command
   - Navigation from trajectory analysis
   - Route provides guidance and completion checking

Usage:
    # Open map mode (free driving)
    python core/vlm_drive.py --mode open --speed 25

    # Route mode (follow route)
    python core/vlm_drive.py --mode route --route ./routes/custom_route.json --speed 20
"""

import argparse
import math
import signal
import time
from typing import List, Optional, Tuple

import carla
import numpy as np

# Try to import pygame (optional for display)
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not available - running in headless mode")

# Import from consolidated modules
from pid_controller import VehiclePIDController
from route_builder import RouteBuilder
from traj_planner import (
    CameraConfig, TrajectoryDecoder, TrajectoryEncoder, Waypoint3D, Waypoint2D
)
from vlm_inference import VLMTrajectoryPredictor, trajectory_to_waypoints_2d
from navigation_analyzer import generate_navigation_token


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

# VLM Endpoint Presets
VLM_ENDPOINTS = {
    'local': {
        'url': 'https://vishwanaths-mac-mini.woodpecker-bluegill.ts.net/v1',
        'model': 'unsloth/Qwen3-VL-2B-Instruct-bnb-4bit',
        'lora': 'driver',
    },
    'cloud': {
        'url': 'https://1b3565c4dfc4.woodpecker-bluegill.ts.net/v1',
        'model': 'unsloth/Qwen3-VL-2B-Instruct-bnb-4bit',
        'lora': 'driver',
    }
}

# Waypoint distances for route mode
WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]


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
# HELPER FUNCTIONS
# =============================================================================

def make_transform(location: dict, rotation: dict = None) -> carla.Transform:
    """Create a CARLA transform from location/rotation dicts or values."""
    loc = carla.Location(x=location['x'], y=location['y'], z=location['z'])
    rot = carla.Rotation(yaw=rotation.get('yaw', 0)) if rotation else carla.Rotation()
    return carla.Transform(loc, rot)


STATUS_COLORS = {
    "NEW": (50, 255, 50),
    "CACHED": (150, 200, 150),
    "FAILED": (255, 50, 50),
}

def get_status_color(status: str) -> Tuple[int, int, int]:
    """Get display color for VLM inference status."""
    return STATUS_COLORS.get(status, (255, 50, 50))


def draw_waypoints(display, waypoints_2d: List, status: str) -> None:
    """Draw waypoint trajectory on the display."""
    if not waypoints_2d or not PYGAME_AVAILABLE:
        return

    color = get_status_color(status)
    thickness = 3 if status == "NEW" else 2

    # Draw trajectory lines
    for i in range(len(waypoints_2d) - 1):
        wp1, wp2 = waypoints_2d[i], waypoints_2d[i + 1]
        pygame.draw.line(display, color, (int(wp1.u), int(wp1.v)), (int(wp2.u), int(wp2.v)), thickness)

    # Draw waypoint circles
    for i, wp in enumerate(waypoints_2d):
        is_last = i == len(waypoints_2d) - 1
        radius = 8 if is_last else 4
        pygame.draw.circle(display, color, (int(wp.u), int(wp.v)), radius)
        pygame.draw.circle(display, (255, 255, 255), (int(wp.u), int(wp.v)), radius - 2)


# =============================================================================
# MANUAL CONTROLLER
# =============================================================================

class ManualController:
    """
    Manual keyboard controller for vehicle.
    
    Controls:
        W / UP    - Accelerate
        S / DOWN  - Brake / Reverse
        A / LEFT  - Steer left
        D / RIGHT - Steer right
        SPACE     - Handbrake
        M         - Toggle manual/VLM mode
    """
    
    def __init__(self, max_steer: float = 1.0, steer_speed: float = 0.05):
        self.max_steer = max_steer
        self.steer_speed = steer_speed
        self.current_steer = 0.0
        self.steer_return_speed = 0.1  # How fast steering returns to center
        
    def get_control(self) -> 'carla.VehicleControl':
        """Get vehicle control from keyboard state."""
        if not PYGAME_AVAILABLE:
            return carla.VehicleControl()
            
        keys = pygame.key.get_pressed()
        
        # Throttle/Brake
        throttle = 0.0
        brake = 0.0
        
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            throttle = 0.7
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            brake = 0.7
            
        # Steering with smooth input
        steer_input = 0.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            steer_input = -1.0
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            steer_input = 1.0
            
        # Apply steering smoothly
        if steer_input != 0:
            self.current_steer += steer_input * self.steer_speed
            self.current_steer = max(-self.max_steer, min(self.max_steer, self.current_steer))
        else:
            # Return to center
            if abs(self.current_steer) < self.steer_return_speed:
                self.current_steer = 0.0
            elif self.current_steer > 0:
                self.current_steer -= self.steer_return_speed
            else:
                self.current_steer += self.steer_return_speed
        
        # Handbrake
        hand_brake = keys[pygame.K_SPACE]
        
        control = carla.VehicleControl(
            throttle=throttle,
            brake=brake,
            steer=self.current_steer,
            hand_brake=hand_brake,
            manual_gear_shift=False
        )
        
        return control
    
    def reset(self):
        """Reset steering to center."""
        self.current_steer = 0.0


# =============================================================================
# UNIFIED VLM DRIVER
# =============================================================================

class VLMDriver:
    """
    Unified VLM driver supporting two modes:
    - OPEN MAP: Free driving with traffic rules
    - ROUTE: Following predefined routes with navigation
    """

    def __init__(self,
                 mode: str,
                 vlm_base_url: str,
                 vlm_model: str,
                 target_speed_kmh: float = 20.0,
                 inference_hz: float = 1.0,
                 route_builder: Optional[RouteBuilder] = None,
                 open_map_prompt: str = "Follow traffic rules and drive safely",
                 lora_adapter: str = None):
        """
        Args:
            mode: 'open' or 'route'
            vlm_base_url: VLM API endpoint
            vlm_model: Base VLM model name (e.g., "unsloth/Qwen3-VL-2B-Instruct-bnb-4bit")
            target_speed_kmh: Target driving speed
            inference_hz: VLM inference frequency (Hz)
            route_builder: RouteBuilder instance (required for route mode)
            open_map_prompt: Prompt for open map mode
            lora_adapter: LoRA adapter name (e.g., "driver") - optional
        """
        self.mode = mode
        self.target_speed_kmh = target_speed_kmh
        self.inference_hz = inference_hz
        self.inference_interval = 1.0 / inference_hz
        self.open_map_prompt = open_map_prompt

        # Route mode
        self.route_builder = route_builder
        self.current_navigation = "lane_keeping"

        # Initialize VLM predictor (synchronous for simplicity)
        print(f"Initializing VLM Driver ({mode.upper()} mode)")
        print(f"  VLM: {vlm_base_url}")
        print(f"  Model: {vlm_model}")
        print(f"  Inference: {inference_hz} Hz ({self.inference_interval:.2f}s interval)")

        self.vlm_predictor = VLMTrajectoryPredictor(
            base_url=vlm_base_url,
            model=vlm_model,
            lora_adapter=lora_adapter
        )

        # Initialize components
        self.decoder = TrajectoryDecoder(CAMERA_CONFIG)
        self.encoder = TrajectoryEncoder(CAMERA_CONFIG)
        self.controller = VehiclePIDController(target_speed_kmh=target_speed_kmh)

        # State tracking
        self.last_inference_time = 0.0
        self.last_waypoints_3d = []
        self.last_waypoints_2d = []
        self.vlm_inference_count = 0
        self.vlm_total_time = 0.0
        self.vlm_failure_count = 0

    def get_waypoints(self,
                     rgb_image: np.ndarray,
                     vehicle_transform: carla.Transform,
                     current_speed_kmh: float,
                     vehicle_location: Optional[carla.Location] = None) -> Tuple[List[Waypoint3D], bool, str]:
        """
        Get waypoints from VLM.

        Returns:
            (waypoints_3d, is_new_prediction, status_message)
        """
        current_time = time.time()
        time_since_last = current_time - self.last_inference_time

        # Check if we should run new inference
        should_infer = time_since_last >= self.inference_interval

        if not should_infer:
            # Return cached waypoints
            return self.last_waypoints_3d, False, "CACHED"

        # Determine prompt based on mode
        if self.mode == 'open':
            # OPEN MAP MODE: No navigation, use custom prompt
            prompt_context = self.open_map_prompt
            nav_display = "Open Map"
        else:
            # ROUTE MODE: Generate navigation from last trajectory
            if self.last_waypoints_3d:
                waypoints_3d_list = [[wp.x, wp.y, wp.z] for wp in self.last_waypoints_3d]
                self.current_navigation = generate_navigation_token(waypoints_3d_list)
            else:
                self.current_navigation = "lane_keeping"

            prompt_context = self.current_navigation
            nav_display = self.current_navigation.replace('_', ' ').title()

        # Run VLM inference
        print(f"\n[VLM] Inference ({nav_display})...", end='', flush=True)

        trajectory_2d, inf_time = self.vlm_predictor.predict(
            rgb_image,
            current_speed=current_speed_kmh,
            navigation=prompt_context
        )

        self.last_inference_time = current_time
        self.vlm_inference_count += 1
        self.vlm_total_time += inf_time

        print(f" → {inf_time:.2f}s", flush=True)

        if not trajectory_2d:
            self.handle_failed_inference()
            print(f"[VLM] ✗ Failed! (Failures: {self.vlm_failure_count})")
            return [], True, "FAILED"

        # Convert to Waypoint2D
        waypoints_2d = trajectory_to_waypoints_2d(trajectory_2d)

        if not waypoints_2d:
            self.handle_failed_inference()
            print(f"[VLM] ✗ Invalid waypoints!")
            return [], True, "FAILED"

        # Cache 2D waypoints
        self.last_waypoints_2d = waypoints_2d

        # Decode to 3D
        road_height = vehicle_transform.location.z - 0.5
        waypoints_3d = self.decoder.decode(waypoints_2d, vehicle_transform, road_height)

        # Cache waypoints
        self.last_waypoints_3d = waypoints_3d

        print(f"[VLM] ✓ Got {len(waypoints_3d)} waypoints")
        return waypoints_3d, True, "NEW"

    def is_route_complete(self, vehicle_location: carla.Location, route) -> bool:
        """Check if route is complete (route mode only)."""
        if self.mode != 'route' or self.route_builder is None:
            return False
        return self.route_builder.is_route_complete(route, vehicle_location, threshold_meters=5.0)

    def get_stats(self) -> Tuple[float, float]:
        """Get VLM inference statistics: (avg_time, success_rate)."""
        if self.vlm_inference_count == 0:
            return 0.0, 100.0
        avg_time = self.vlm_total_time / self.vlm_inference_count
        success_rate = (1 - self.vlm_failure_count / self.vlm_inference_count) * 100
        return avg_time, success_rate

    def handle_failed_inference(self) -> None:
        """Handle a failed VLM inference by clearing cached waypoints."""
        self.vlm_failure_count += 1
        self.last_waypoints_3d = []
        self.last_waypoints_2d = []


# =============================================================================
# MAIN DRIVE FUNCTION
# =============================================================================

def run_vlm_drive(
    mode: str,
    vlm_preset: str = 'cloud',
    vlm_base_url: str = None,
    vlm_model: str = None,
    lora_adapter: str = None,
    target_speed_kmh: float = 20.0,
    inference_hz: float = 1.0,
    # Route mode options
    route_file: str = None,
    routes_dir: str = None,
    loops: int = 1,
    # Open map mode options
    open_map_prompt: str = "Follow traffic rules and drive safely",
    spawn_x: float = None,
    spawn_y: float = None,
    spawn_z: float = None,
    spawn_yaw: float = 0.0,
    duration: float = None,
    # Display options
    use_display: bool = True,
):
    """Run unified VLM driver."""

    # Resolve VLM endpoint
    if vlm_base_url is None or vlm_model is None or lora_adapter is None:
        if vlm_preset not in VLM_ENDPOINTS:
            print(f"Warning: Unknown preset '{vlm_preset}', using 'cloud'")
            vlm_preset = 'cloud'
        preset = VLM_ENDPOINTS[vlm_preset]
        vlm_base_url = vlm_base_url or preset['url']
        vlm_model = vlm_model or preset['model']
        lora_adapter = lora_adapter or preset.get('lora')

    # Initialize display
    display = None
    clock = None
    font = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption(f'VLM Drive - {mode.upper()} Mode')
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 14)

    # Print header
    print(f"\n{'='*70}")
    print(f"VLM AUTONOMOUS DRIVING - {mode.upper()} MODE")
    print(f"{'='*70}")
    print(f"Speed: {target_speed_kmh} km/h")
    print(f"VLM: {vlm_base_url}")
    print(f"Model: {vlm_model}")
    if lora_adapter:
        print(f"LoRA: {lora_adapter}")

    if mode == 'open':
        print(f"Prompt: \"{open_map_prompt}\"")
        if duration:
            print(f"Duration: {duration}s")
    else:
        print(f"Route: {route_file or routes_dir}")
        print(f"Loops: {loops}")

    print(f"{'='*70}\n")

    # Load routes (route mode only)
    route_builder = None
    if mode == 'route':
        print("Loading routes...")
        route_builder = RouteBuilder(
            routes_dir=routes_dir,
            route_file=route_file,
            waypoint_distances=WAYPOINT_DISTANCES
        )
        print(f"Loaded {route_builder.get_route_count()} routes\n")

    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # world = client.get_world()
    world = client.load_world('Town10HD')
    bp_lib = world.get_blueprint_library()

    # Initialize driver
    driver = VLMDriver(
        mode=mode,
        vlm_base_url=vlm_base_url,
        vlm_model=vlm_model,
        target_speed_kmh=target_speed_kmh,
        inference_hz=inference_hz,
        route_builder=route_builder,
        open_map_prompt=open_map_prompt,
        lora_adapter=lora_adapter
    )

    actors = []
    vehicle = None
    camera = None
    start_time = time.time()

    # Route mode state
    current_route_idx = 0
    current_route_loop = 0
    routes_completed = 0
    current_route = None

    # Manual mode state
    manual_mode = False
    manual_controller = ManualController() if PYGAME_AVAILABLE else None

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

        if mode == 'route' and route_builder:
            # Route mode: spawn at route start
            current_route = route_builder.get_route(current_route_idx)
            spawn_info = route_builder.get_spawn_transform(current_route)
            spawn_transform = make_transform(spawn_info['location'], spawn_info['rotation'])
            print(f"Spawning at route: {current_route.name}")
        elif spawn_x is not None and spawn_y is not None:
            # Open map mode: spawn at specified location
            spawn_transform = carla.Transform(
                carla.Location(x=spawn_x, y=spawn_y, z=spawn_z or 0.5),
                carla.Rotation(yaw=spawn_yaw)
            )
            print(f"Spawning at ({spawn_x:.1f}, {spawn_y:.1f}, {spawn_z or 0.5:.1f})")
        else:
            # Open map mode: spawn at map default
            spawn_points = world.get_map().get_spawn_points()
            spawn_transform = spawn_points[0] if spawn_points else carla.Transform()
            print("Spawning at map default")

        vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
        actors.append(vehicle)
        
        # Track spawn/reset position
        reset_transform = spawn_transform
        last_good_transform = spawn_transform
        last_good_transform_time = time.time()

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
        print("Starting VLM driving...\n")

        frame = 0
        total_routes = route_builder.get_route_count() * loops if mode == 'route' else 0

        # Main driving loop
        while not shutdown_requested:
            # Check termination conditions
            if mode == 'open' and duration and (time.time() - start_time) > duration:
                print(f"\n\nReached duration: {duration}s")
                break

            if mode == 'route' and routes_completed >= total_routes:
                print(f"\n\nCompleted all routes!")
                break

            # Handle pygame events
            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            shutdown_requested = True
                        elif event.key == pygame.K_m:
                            # Toggle manual mode
                            manual_mode = not manual_mode
                            if manual_controller:
                                manual_controller.reset()
                            mode_str = "MANUAL" if manual_mode else "VLM"
                            print(f"\n[MODE] Switched to {mode_str} control")
                        elif event.key == pygame.K_r:
                            # Reset vehicle position
                            print(f"\n[RESET] Resetting vehicle position...")
                            vehicle.set_transform(reset_transform)
                            vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                            vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
                            driver.controller.reset()
                            if manual_controller:
                                manual_controller.reset()
                            time.sleep(0.3)
                            print(f"[RESET] Vehicle reset to spawn point")
                        elif event.key == pygame.K_b:
                            # Backup to last good position
                            print(f"\n[BACKUP] Returning to last good position...")
                            vehicle.set_transform(last_good_transform)
                            vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                            vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
                            driver.controller.reset()
                            if manual_controller:
                                manual_controller.reset()
                            time.sleep(0.3)
                            print(f"[BACKUP] Vehicle returned to last good position")

            # Get vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            # Track last good position (when moving at reasonable speed)
            current_time_check = time.time()
            if speed_kmh > 5.0 and (current_time_check - last_good_transform_time) > 2.0:
                last_good_transform = transform
                last_good_transform_time = current_time_check

            # Route completion check (route mode only)
            if mode == 'route' and current_route and driver.is_route_complete(transform.location, current_route):
                routes_completed += 1
                current_route_loop += 1

                elapsed = time.time() - start_time
                print(f"\n✓ Route complete: {current_route.name} (loop {current_route_loop}/{loops})")
                print(f"  Time: {elapsed/60:.1f}min")

                # Move to next route or loop
                if current_route_loop >= loops:
                    current_route_idx = (current_route_idx + 1) % route_builder.get_route_count()
                    current_route_loop = 0

                    if routes_completed < total_routes:
                        current_route = route_builder.get_route(current_route_idx)
                        print(f"→ Starting route: {current_route.name}")

                # Teleport to route start
                spawn_info = route_builder.get_spawn_transform(current_route)
                new_transform = make_transform(spawn_info['location'], spawn_info['rotation'])
                vehicle.set_transform(new_transform)
                reset_transform = new_transform  # Update reset point for new route
                last_good_transform = new_transform
                driver.controller.reset()
                time.sleep(0.5)
                continue

            # Get control based on mode
            waypoints_3d = []
            status = "Waiting"

            if manual_mode and manual_controller:
                # MANUAL MODE: Get control from keyboard
                control = manual_controller.get_control()
                status = "MANUAL"
            else:
                # VLM MODE: Get waypoints and compute control
                if sensor_data.rgb_image is not None:
                    waypoints_3d, is_new, status = driver.get_waypoints(
                        sensor_data.rgb_image,
                        transform,
                        speed_kmh,
                        transform.location
                    )

                # Compute control from PID
                control = driver.controller.compute_control(waypoints_3d, transform, speed_kmh)
            
            vehicle.apply_control(control)

            # Update display
            if display and sensor_data.rgb_image is not None:
                # Draw camera feed
                surface = pygame.surfarray.make_surface(sensor_data.rgb_image.swapaxes(0, 1))
                display.blit(surface, (0, 0))

                # Draw waypoints (only in VLM mode)
                if not manual_mode and driver.last_waypoints_2d:
                    waypoints_2d_display = (
                        driver.encoder.encode_with_indices(waypoints_3d, transform)[0]
                        if waypoints_3d else driver.last_waypoints_2d
                    )
                    draw_waypoints(display, waypoints_2d_display, status)

                # Info overlay
                avg_vlm, success_rate = driver.get_stats()
                elapsed = time.time() - start_time

                # Status bar
                if manual_mode:
                    status_color = (255, 165, 0)  # Orange for manual
                    status_text = f"{mode.upper()}: MANUAL (Press M for VLM)"
                else:
                    status_color = get_status_color(status)
                    status_text = f"{mode.upper()}: {status} (Press M for Manual)"

                pygame.draw.rect(display, (0, 0, 0, 200), (0, 0, WINDOW_WIDTH, 35))
                status_font = pygame.font.SysFont('monospace', 16, bold=True)
                status_surf = status_font.render(status_text, True, status_color)
                display.blit(status_surf, (WINDOW_WIDTH // 2 - status_surf.get_width() // 2, 8))

                # Navigation indicator (route mode) or prompt (open mode) or manual controls
                if manual_mode:
                    nav_text = "W/S: Throttle  A/D: Steer  R: Reset  B: Backup"
                    nav_color = (255, 165, 0)
                else:
                    nav_text = (
                        f"Nav: {driver.current_navigation.upper().replace('_', ' ')}"
                        if mode == 'route'
                        else f"Prompt: {driver.open_map_prompt[:30]}"
                    )
                    nav_color = (255, 200, 50)
                nav_surf = font.render(nav_text, True, nav_color)
                nav_bg = pygame.Rect(WINDOW_WIDTH - nav_surf.get_width() - 20, 5, nav_surf.get_width() + 10, nav_surf.get_height() + 4)
                pygame.draw.rect(display, (0, 0, 0), nav_bg)
                pygame.draw.rect(display, nav_color, nav_bg, 2)
                display.blit(nav_surf, (WINDOW_WIDTH - nav_surf.get_width() - 15, 7))

                # Info panel - build lines with None as spacers
                mode_info = (
                    [f"Route: {current_route.name} ({current_route_loop+1}/{loops})",
                     f"Progress: {routes_completed}/{total_routes}"]
                    if mode == 'route' and current_route
                    else [f"Elapsed: {int(elapsed)}s ({elapsed/60:.1f}min)"]
                )

                info_lines = [
                    f"Speed: {speed_kmh:.1f} km/h (Target: {driver.target_speed_kmh:.0f})",
                    f"Control: S={control.steer:.2f} T={control.throttle:.2f} B={control.brake:.2f}",
                    None,
                    f"VLM: {driver.vlm_inference_count} inferences",
                    f"Success: {success_rate:.1f}%",
                    f"Avg: {avg_vlm:.2f}s",
                    f"Failures: {driver.vlm_failure_count}",
                    None,
                ] + mode_info

                y_offset = 45
                for line in info_lines:
                    if line is not None:
                        text = font.render(line, True, (255, 255, 255))
                        bg_rect = pygame.Rect(5, y_offset - 2, text.get_width() + 10, text.get_height() + 4)
                        pygame.draw.rect(display, (0, 0, 0, 180), bg_rect)
                        display.blit(text, (10, y_offset))
                    y_offset += 18

                # FPS
                fps_text = f"FPS: {clock.get_fps():.0f}"
                fps_surf = font.render(fps_text, True, (200, 200, 200))
                display.blit(fps_surf, (WINDOW_WIDTH - 80, WINDOW_HEIGHT - 25))

                pygame.display.flip()
                clock.tick(30)
            else:
                world.tick()
                time.sleep(0.033)

            # Print progress
            if frame % 30 == 0:
                avg_vlm, _ = driver.get_stats()
                elapsed = time.time() - start_time
                ctrl_mode = "MANUAL" if manual_mode else "VLM"
                if mode == 'route' and current_route:
                    print(f"\r[{ctrl_mode}] {current_route.name} | {routes_completed}/{total_routes} | Speed: {speed_kmh:.0f}km/h | VLM: {avg_vlm:.2f}s", end='')
                else:
                    print(f"\r[{ctrl_mode}] Speed: {speed_kmh:.0f}km/h | Status: {status} | VLM: {avg_vlm:.2f}s | Elapsed: {int(elapsed)}s", end='')

            frame += 1

        print(f"\n\n{'='*70}")
        print(f"VLM DRIVING COMPLETE")
        print(f"{'='*70}")

    finally:
        # Cleanup
        print("\nCleaning up...")
        for actor in actors:
            if actor is not None:
                actor.destroy()

        if display:
            pygame.quit()

        # Final stats
        elapsed = time.time() - start_time
        avg_vlm, success_rate = driver.get_stats()

        print(f"\nFinal Statistics:")
        print(f"  Total time: {elapsed/60:.1f} minutes")
        print(f"  VLM inferences: {driver.vlm_inference_count}")
        print(f"  VLM failures: {driver.vlm_failure_count}")
        print(f"  Success rate: {success_rate:.1f}%")
        print(f"  Avg VLM time: {avg_vlm:.2f}s")

        if mode == 'route':
            print(f"  Routes completed: {routes_completed}/{total_routes}")

        print(f"\n{'='*70}\n")


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Unified VLM Autonomous Driving',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Open map mode (free driving)
  python core/vlm_drive.py --mode open --speed 25 --duration 300

  # Open map mode with custom prompt
  python core/vlm_drive.py --mode open --prompt "Drive carefully and avoid obstacles"

  # Route mode (follow route)
  python core/vlm_drive.py --mode route --route ./routes/custom_route.json --speed 20

  # Route mode with multiple loops
  python core/vlm_drive.py --mode route --route ./routes/straight_1.json --loops 3

  # Open map with custom spawn
  python core/vlm_drive.py --mode open --spawn-x 100 --spawn-y 50 --spawn-yaw 90
        """
    )

    parser.add_argument('--mode', type=str, required=True, choices=['open', 'route'],
                        help='Driving mode: open (free) or route (follow routes)')

    parser.add_argument('--vlm-preset', type=str, default='cloud', choices=['local', 'cloud'],
                        help='VLM endpoint preset (default: cloud)')

    parser.add_argument('--vlm-url', type=str, help='VLM API URL (overrides preset)')
    parser.add_argument('--vlm-model', type=str, help='VLM base model name (overrides preset)')
    parser.add_argument('--lora', type=str, help='LoRA adapter name (e.g., "driver", overrides preset)')

    parser.add_argument('--speed', type=float, default=20.0,
                        help='Target speed in km/h (default: 20)')

    parser.add_argument('--inference-hz', type=float, default=1.0,
                        help='VLM inference frequency in Hz (default: 1.0)')

    # Route mode options
    parser.add_argument('--route', type=str, help='Route JSON file (route mode)')
    parser.add_argument('--routes', type=str, help='Routes directory (route mode)')
    parser.add_argument('--loops', type=int, default=1, help='Loops per route (route mode, default: 1)')

    # Open map mode options
    parser.add_argument('--prompt', type=str, default="Follow traffic rules and drive safely",
                        help='VLM prompt for open map mode')

    parser.add_argument('--spawn-x', type=float, help='Spawn X coordinate (open mode)')
    parser.add_argument('--spawn-y', type=float, help='Spawn Y coordinate (open mode)')
    parser.add_argument('--spawn-z', type=float, help='Spawn Z coordinate (open mode)')
    parser.add_argument('--spawn-yaw', type=float, default=0.0, help='Spawn yaw angle (open mode)')

    parser.add_argument('--duration', type=float, help='Max duration in seconds (open mode)')

    parser.add_argument('--no-display', action='store_true', help='Headless mode')

    args = parser.parse_args()

    # Validate arguments
    if args.mode == 'route' and not args.route and not args.routes:
        parser.error("Route mode requires --route or --routes")

    run_vlm_drive(
        mode=args.mode,
        vlm_preset=args.vlm_preset,
        vlm_base_url=args.vlm_url,
        vlm_model=args.vlm_model,
        lora_adapter=args.lora,
        target_speed_kmh=args.speed,
        inference_hz=args.inference_hz,
        route_file=args.route,
        routes_dir=args.routes,
        loops=args.loops,
        open_map_prompt=args.prompt,
        spawn_x=args.spawn_x,
        spawn_y=args.spawn_y,
        spawn_z=args.spawn_z,
        spawn_yaw=args.spawn_yaw,
        duration=args.duration,
        use_display=not args.no_display,
    )
