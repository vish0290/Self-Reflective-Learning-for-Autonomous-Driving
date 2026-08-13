#!/usr/bin/env python3
"""
VLM Control-Based Autonomous Driving

Unlike vlm_drive.py which predicts trajectories, this driver gets direct control commands
(steer, throttle, brake) from the VLM and applies them to the vehicle.

Two modes:
1. OPEN MAP MODE: Free driving with custom prompts
   - VLM gets: image + speed + prompt
   - Outputs: steer, throttle, brake

2. ROUTE MODE: Follows predefined routes with navigation
   - VLM gets: image + speed + navigation command
   - Outputs: steer, throttle, brake
   - Route provides navigation context and completion checking

Usage:
    # Open map mode (free driving)
    python core/vlm_control_drive.py --mode open --speed 25

    # Route mode (follow route)
    python core/vlm_control_drive.py --mode route --route ./routes/custom_route.json --speed 20
"""

import argparse
import math
import signal
import time
from typing import Optional, Tuple

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
from route_builder import RouteBuilder
from vlm_control_inference import VLMControlPredictor
from navigation_analyzer import generate_navigation_token
from control_navigation import get_nav_prompt
from record_dataset import DiscreteControlEncoder, StanleyController


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480
CAMERA_FOV = 90

WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]

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

# Navigation command mapping (route mode)
# NOTE: Use get_nav_prompt() for readable prompts.


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


def get_lane_following_waypoints(
    carla_map: carla.Map,
    vehicle_location: carla.Location,
    distances=None,
):
    distances = distances or WAYPOINT_DISTANCES
    base_wp = carla_map.get_waypoint(
        vehicle_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    if base_wp is None:
        return []

    waypoints_3d = []
    for dist in distances:
        next_wps = base_wp.next(dist)
        if not next_wps:
            break
        wp = next_wps[0]
        loc = wp.transform.location
        waypoints_3d.append([loc.x, loc.y, loc.z])

    return waypoints_3d


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
        self.steer_return_speed = 0.1

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
# VLM CONTROL DRIVER
# =============================================================================

class VLMControlDriver:
    """
    VLM driver that predicts direct control commands (steer, throttle, brake).

    Supports two modes:
    - OPEN MAP: Free driving with custom prompts
    - ROUTE: Following predefined routes with navigation commands
    """

    def __init__(self,
                 mode: str,
                 vlm_base_url: str,
                 vlm_model: str,
                 inference_hz: float = 2.0,
                 route_builder: Optional[RouteBuilder] = None,
                 open_map_prompt: str = "Follow traffic rules and drive safely",
                 lora_adapter: str = None):
        """
        Args:
            mode: 'open' or 'route'
            vlm_base_url: VLM API endpoint
            vlm_model: Base VLM model name (e.g., "unsloth/Qwen3-VL-2B-Instruct-bnb-4bit")
            inference_hz: VLM inference frequency (Hz)
            route_builder: RouteBuilder instance (required for route mode)
            open_map_prompt: Prompt for open map mode
            lora_adapter: LoRA adapter name (e.g., "driver") - optional
        """
        self.mode = mode
        self.inference_hz = inference_hz
        self.inference_interval = 1.0 / inference_hz
        self.open_map_prompt = open_map_prompt

        # Route mode state
        self.route_builder = route_builder
        self.current_navigation = "lane_keeping"
        self.last_vehicle_transform = None
        self.last_nav_command = "follow_lane"

        # Control-based navigation (Stanley + discrete encoder)
        self.control_encoder = DiscreteControlEncoder()
        self.stanley_controller = StanleyController(target_speed_kmh=30.0)

        # Navigation smoothing to avoid abrupt turn commands
        self.turn_persist_frames = 3
        self._turn_left_count = 0
        self._turn_right_count = 0

        # Initialize VLM predictor
        print(f"Initializing VLM Control Driver ({mode.upper()} mode)")
        print(f"  VLM: {vlm_base_url}")
        print(f"  Model: {vlm_model}")
        print(f"  Inference: {inference_hz} Hz ({self.inference_interval:.2f}s interval)")

        self.vlm_predictor = VLMControlPredictor(
            base_url=vlm_base_url,
            model=vlm_model,
            lora_adapter=lora_adapter
        )

        # State tracking
        self.last_inference_time = 0.0
        self.last_control = {"steer": 0.0, "throttle": 0.0, "brake": 0.0}
        self.vlm_inference_count = 0
        self.vlm_total_time = 0.0
        self.vlm_failure_count = 0

    def get_control(self,
                   rgb_image: np.ndarray,
                   speed_kmh: float,
                   vehicle_transform: carla.Transform,
                   nav_command: Optional[str] = None) -> Tuple[carla.VehicleControl, str]:
        """
        Get vehicle control from VLM.

        Returns:
            (vehicle_control, status_message)
        """
        current_time = time.time()
        time_since_last = current_time - self.last_inference_time

        # Check if we should run new inference
        should_infer = time_since_last >= self.inference_interval

        if not should_infer:
            # Return cached control
            control = carla.VehicleControl(
                steer=self.last_control["steer"],
                throttle=self.last_control["throttle"],
                brake=self.last_control["brake"]
            )
            return control, "CACHED"

        # Determine navigation prompt based on mode
        if self.mode == 'open':
            # OPEN MAP MODE: Use custom prompt + optional lane-following nav
            if nav_command is not None:
                self.last_nav_command = nav_command
                navigation_text = f"{self.open_map_prompt}. {get_nav_prompt(nav_command)}"
                nav_display = nav_command
            else:
                navigation_text = self.open_map_prompt
                nav_display = "Open Map"
        else:
            if nav_command is not None:
                # ROUTE MODE (preferred): Use control-based navigation command
                self.last_nav_command = nav_command
                navigation_text = get_nav_prompt(nav_command)
                nav_display = nav_command
            else:
                # ROUTE MODE (fallback): Estimate navigation from recent trajectory
                if self.last_vehicle_transform is not None:
                    delta_x = vehicle_transform.location.x - self.last_vehicle_transform.location.x
                    delta_y = vehicle_transform.location.y - self.last_vehicle_transform.location.y

                    if abs(delta_x) > 0.1 or abs(delta_y) > 0.1:
                        waypoints_3d = [
                            [self.last_vehicle_transform.location.x,
                             self.last_vehicle_transform.location.y,
                             self.last_vehicle_transform.location.z],
                            [vehicle_transform.location.x,
                             vehicle_transform.location.y,
                             vehicle_transform.location.z]
                        ]
                        self.current_navigation = generate_navigation_token(waypoints_3d)

                navigation_text = get_nav_prompt(self.current_navigation)
                nav_display = navigation_text

        # Store current transform for next iteration
        self.last_vehicle_transform = vehicle_transform

        # Run VLM inference
        print(f"\n[VLM] Inference ({nav_display})...", end='', flush=True)

        control_dict, inf_time = self.vlm_predictor.predict(
            rgb_image,
            speed_kmh=speed_kmh,
            navigation=navigation_text
        )

        self.last_inference_time = current_time
        self.vlm_inference_count += 1
        self.vlm_total_time += inf_time

        print(f" → {inf_time:.2f}s", flush=True)

        if control_dict is None:
            self.handle_failed_inference()
            print(f"[VLM] ✗ Failed! (Failures: {self.vlm_failure_count})")
            # Return last known good control
            control = carla.VehicleControl(
                steer=self.last_control["steer"],
                throttle=self.last_control["throttle"],
                brake=self.last_control["brake"]
            )
            return control, "FAILED"

        # Update cached control
        self.last_control = control_dict

        # Create CARLA control
        control = carla.VehicleControl(
            steer=control_dict["steer"],
            throttle=control_dict["throttle"],
            brake=control_dict["brake"]
        )

        print(f"[VLM] ✓ S={control_dict['steer']:+.2f} T={control_dict['throttle']:.2f} B={control_dict['brake']:.2f}")
        return control, "NEW"

    def compute_train_nav_command(self,
                                  waypoints_3d,
                                  vehicle_transform: carla.Transform,
                                  speed_kmh: float) -> str:
        if not waypoints_3d:
            return "follow_lane"

        # Use Stanley steering to generate a discrete steering label
        stanley_control = self.stanley_controller.compute_control(
            waypoints_3d,
            vehicle_transform,
            speed_kmh
        )
        discrete_control = self.control_encoder.encode(stanley_control)
        return self.control_encoder.train_nav(discrete_control["steer"])

    def smooth_nav_command(self, nav_command: Optional[str]) -> Optional[str]:
        if nav_command is None:
            return None

        if nav_command == "turn_left":
            self._turn_left_count += 1
            self._turn_right_count = 0
            if self._turn_left_count < self.turn_persist_frames:
                return "slight_left"
            return "turn_left"

        if nav_command == "turn_right":
            self._turn_right_count += 1
            self._turn_left_count = 0
            if self._turn_right_count < self.turn_persist_frames:
                return "slight_right"
            return "turn_right"

        if nav_command in ("slight_left", "slight_right", "follow_lane"):
            self._turn_left_count = 0
            self._turn_right_count = 0
            return nav_command

        return nav_command

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
        """Handle a failed VLM inference."""
        self.vlm_failure_count += 1


# =============================================================================
# MAIN DRIVE FUNCTION
# =============================================================================

def run_vlm_control_drive(
    mode: str,
    vlm_preset: str = 'cloud',
    vlm_base_url: str = None,
    vlm_model: str = None,
    lora_adapter: str = None,
    inference_hz: float = 2.0,
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
    """Run VLM control driver."""

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
        pygame.display.set_caption(f'VLM Control Drive - {mode.upper()} Mode')
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 14)

    # Print header
    print(f"\n{'='*70}")
    print(f"VLM CONTROL DRIVING - {mode.upper()} MODE")
    print(f"{'='*70}")
    print(f"VLM: {vlm_base_url}")
    print(f"Model: {vlm_model}")
    if lora_adapter:
        print(f"LoRA: {lora_adapter}")
    print(f"Inference: {inference_hz} Hz")

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
            waypoint_distances=[3, 6, 9, 12, 15, 18, 21, 24, 27, 30]
        )
        print(f"Loaded {route_builder.get_route_count()} routes\n")

    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.load_world('Town10HD')
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    # Initialize driver
    driver = VLMControlDriver(
        mode=mode,
        vlm_base_url=vlm_base_url,
        vlm_model=vlm_model,
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

        # Attach camera
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(WINDOW_WIDTH))
        cam_bp.set_attribute('image_size_y', str(WINDOW_HEIGHT))
        cam_bp.set_attribute('fov', str(CAMERA_FOV))

        cam_transform = carla.Transform(
            carla.Location(x=2.0, y=0.0, z=1.8),
            carla.Rotation(pitch=-15, yaw=0, roll=0)
        )

        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(process_rgb)

        time.sleep(1.0)
        print("Starting VLM control driving...\n")

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
                            if manual_controller:
                                manual_controller.reset()
                            time.sleep(0.3)
                            print(f"[RESET] Vehicle reset to spawn point")

            # Get vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

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
                reset_transform = new_transform
                time.sleep(0.5)
                continue

            # Get control based on mode
            status = "Waiting"
            nav_command = None

            if mode == 'route' and current_route and route_builder:
                waypoints_3d = route_builder.get_waypoints_at_distances(
                    route=current_route,
                    vehicle_location=transform.location
                )
                nav_command = driver.compute_train_nav_command(
                    waypoints_3d,
                    transform,
                    speed_kmh
                )
            elif mode == 'open':
                waypoints_3d = get_lane_following_waypoints(
                    carla_map,
                    transform.location
                )
                nav_command = driver.compute_train_nav_command(
                    waypoints_3d,
                    transform,
                    speed_kmh
                )

            nav_command = driver.smooth_nav_command(nav_command)

            if manual_mode and manual_controller:
                # MANUAL MODE: Get control from keyboard
                control = manual_controller.get_control()
                status = "MANUAL"
            else:
                # VLM MODE: Get control from VLM
                if sensor_data.rgb_image is not None:
                    control, status = driver.get_control(
                        sensor_data.rgb_image,
                        speed_kmh,
                        transform,
                        nav_command=nav_command
                    )
                else:
                    control = carla.VehicleControl()

            vehicle.apply_control(control)

            # Update display
            if display and sensor_data.rgb_image is not None:
                # Draw camera feed
                surface = pygame.surfarray.make_surface(sensor_data.rgb_image.swapaxes(0, 1))
                display.blit(surface, (0, 0))

                # Info overlay
                avg_vlm, success_rate = driver.get_stats()
                elapsed = time.time() - start_time

                # Status bar
                status_colors = {"NEW": (50, 255, 50), "CACHED": (150, 200, 150), "FAILED": (255, 50, 50), "MANUAL": (255, 165, 0)}
                status_color = status_colors.get(status, (255, 255, 255))
                status_text = f"{mode.upper()}: {status}" + (" (Press M for Manual)" if not manual_mode else " (Press M for VLM)")

                pygame.draw.rect(display, (0, 0, 0, 200), (0, 0, WINDOW_WIDTH, 35))
                status_font = pygame.font.SysFont('monospace', 16, bold=True)
                status_surf = status_font.render(status_text, True, status_color)
                display.blit(status_surf, (WINDOW_WIDTH // 2 - status_surf.get_width() // 2, 8))

                # Info panel
                mode_info = (
                    [f"Route: {current_route.name} ({current_route_loop+1}/{loops})",
                     f"Progress: {routes_completed}/{total_routes}"]
                    if mode == 'route' and current_route
                    else [f"Elapsed: {int(elapsed)}s ({elapsed/60:.1f}min)"]
                )

                info_lines = [
                    f"Speed: {speed_kmh:.1f} km/h",
                    f"Control: S={control.steer:+.2f} T={control.throttle:.2f} B={control.brake:.2f}",
                ]
                if nav_command is not None:
                    info_lines.append(f"Nav: {nav_command or driver.last_nav_command}")
                info_lines += [
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
        print(f"VLM CONTROL DRIVING COMPLETE")
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
        description='VLM Control-Based Autonomous Driving',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Open map mode (free driving)
  python core/vlm_control_drive.py --mode open --inference-hz 2

  # Open map mode with custom prompt
  python core/vlm_control_drive.py --mode open --prompt "Drive carefully and avoid obstacles"

  # Route mode (follow route)
  python core/vlm_control_drive.py --mode route --route ./routes/custom_route.json

  # Route mode with multiple loops
  python core/vlm_control_drive.py --mode route --route ./routes/straight_1.json --loops 3

  # Open map with custom spawn
  python core/vlm_control_drive.py --mode open --spawn-x 100 --spawn-y 50 --spawn-yaw 90
        """
    )

    parser.add_argument('--mode', type=str, required=True, choices=['open', 'route'],
                        help='Driving mode: open (free) or route (follow routes)')

    parser.add_argument('--vlm-preset', type=str, default='cloud', choices=['local', 'cloud'],
                        help='VLM endpoint preset (default: cloud)')

    parser.add_argument('--vlm-url', type=str, help='VLM API URL (overrides preset)')
    parser.add_argument('--vlm-model', type=str, help='VLM base model name (overrides preset)')
    parser.add_argument('--lora', type=str, help='LoRA adapter name (e.g., "driver", overrides preset)')

    parser.add_argument('--inference-hz', type=float, default=2.0,
                        help='VLM inference frequency in Hz (default: 2.0)')

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

    run_vlm_control_drive(
        mode=args.mode,
        vlm_preset=args.vlm_preset,
        vlm_base_url=args.vlm_url,
        vlm_model=args.vlm_model,
        lora_adapter=args.lora,
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
