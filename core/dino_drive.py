#!/usr/bin/env python3
"""
DINOv2 Autonomous Driving in CARLA

Runs the DINOv2 trajectory prediction model in real-time:
  Camera -> DINOv2 -> MLP -> 10 pixel waypoints -> TrajectoryDecoder -> PID -> drive

Usage:
    python core/dino_drive.py --checkpoint ./dino_driver_checkpoints/best_heads.pt
    python core/dino_drive.py --checkpoint best_heads.pt --speed 25
"""

import argparse
import math
import signal
import time

import carla
import numpy as np
import torch

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

from pid_controller import VehiclePIDController
from traj_planner import (
    CameraConfig, TrajectoryDecoder, TrajectoryEncoder,
    Waypoint2D, Waypoint3D,
)
from control_navigation import SteeringNavigationAnalyzer
from dino_driver import DINODriver, NAV_COMMANDS, IMG_W, IMG_H

# =============================================================================
# CONFIG
# =============================================================================

CAMERA_CONFIG = CameraConfig(
    width=IMG_W, height=IMG_H, fov=90,
    x=2.0, y=0.0, z=1.8,
    pitch=-15, yaw=0, roll=0,
)


# =============================================================================
# SENSOR
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
# MAIN
# =============================================================================

def run_dino_drive(
    checkpoint: str,
    target_speed_kmh: float = 20.0,
    use_display: bool = True,
    spawn_x: float = None,
    spawn_y: float = None,
    spawn_z: float = None,
    spawn_yaw: float = 0.0,
    duration: float = None,
):
    # Device
    device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    print(f"Device: {device}")

    # Load model
    print("Loading DINOv2 model...")
    model = DINODriver().to(device)
    model.load_heads(checkpoint, map_location=device)
    model.eval()
    print(f"Loaded checkpoint: {checkpoint}")

    # Components
    decoder = TrajectoryDecoder(CAMERA_CONFIG)
    encoder = TrajectoryEncoder(CAMERA_CONFIG)
    controller = VehiclePIDController(target_speed_kmh=target_speed_kmh)
    nav_analyzer = SteeringNavigationAnalyzer()

    # Display
    display = None
    clock = None
    font = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((IMG_W, IMG_H))
        pygame.display.set_caption('DINOv2 Drive')
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 14)

    print(f"\n{'='*60}")
    print(f"DINO AUTONOMOUS DRIVING")
    print(f"{'='*60}")
    print(f"Speed: {target_speed_kmh} km/h")
    print(f"{'='*60}\n")

    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    actors = []
    shutdown_requested = False
    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Spawn vehicle
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        if spawn_x is not None and spawn_y is not None:
            spawn_transform = carla.Transform(
                carla.Location(x=spawn_x, y=spawn_y, z=spawn_z or 0.5),
                carla.Rotation(yaw=spawn_yaw),
            )
        else:
            spawn_points = world.get_map().get_spawn_points()
            spawn_transform = spawn_points[0] if spawn_points else carla.Transform()

        vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
        actors.append(vehicle)

        # Camera
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(IMG_W))
        cam_bp.set_attribute('image_size_y', str(IMG_H))
        cam_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))
        cam_transform = carla.Transform(
            carla.Location(x=CAMERA_CONFIG.x, y=CAMERA_CONFIG.y, z=CAMERA_CONFIG.z),
            carla.Rotation(pitch=CAMERA_CONFIG.pitch),
        )
        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
        actors.append(camera)
        camera.listen(process_rgb)

        time.sleep(1.0)
        print("Driving...\n")

        start_time = time.time()
        frame = 0
        nav_cmd_idx = 0  # follow_lane
        prev_steer = 0.0
        inference_times = []

        while not shutdown_requested:
            if duration and (time.time() - start_time) > duration:
                break

            # Pygame events
            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                        vehicle.set_transform(spawn_transform)
                        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                        controller.reset()
                        time.sleep(0.3)

            # Vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            # Nav from previous steer
            nav_str = nav_analyzer.from_steer_value(prev_steer)
            nav_cmd_idx = NAV_COMMANDS.get(nav_str, 0)

            waypoints_3d = []
            waypoints_2d = []

            if sensor_data.rgb_image is not None:
                # Prepare input
                img_tensor = torch.from_numpy(sensor_data.rgb_image).float().permute(2, 0, 1) / 255.0
                img_tensor = img_tensor.unsqueeze(0).to(device)
                speed_tensor = torch.tensor([[speed_kmh]], dtype=torch.float32).to(device)
                nav_tensor = torch.tensor([nav_cmd_idx], dtype=torch.long).to(device)

                # Inference
                t0 = time.time()
                with torch.no_grad():
                    pixels = model.predict_pixels(img_tensor, speed_tensor, nav_tensor)
                inf_time = time.time() - t0
                inference_times.append(inf_time)

                # Convert to Waypoint2D
                px = pixels[0].cpu().numpy()
                waypoints_2d = [Waypoint2D(u=float(px[i, 0]), v=float(px[i, 1]))
                                for i in range(px.shape[0])]

                # Decode to 3D
                road_height = transform.location.z - 0.5
                waypoints_3d = decoder.decode(waypoints_2d, transform, road_height)

            # PID control
            control = controller.compute_control(waypoints_3d, transform, speed_kmh)
            vehicle.apply_control(control)
            prev_steer = control.steer

            # Display
            if display and sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(sensor_data.rgb_image.swapaxes(0, 1))
                display.blit(surface, (0, 0))

                # Draw waypoints
                if waypoints_2d:
                    color = (50, 255, 50)
                    for i in range(len(waypoints_2d) - 1):
                        w1, w2 = waypoints_2d[i], waypoints_2d[i+1]
                        pygame.draw.line(display, color,
                                        (int(w1.u), int(w1.v)),
                                        (int(w2.u), int(w2.v)), 3)
                    for wp in waypoints_2d:
                        pygame.draw.circle(display, color, (int(wp.u), int(wp.v)), 4)
                        pygame.draw.circle(display, (255, 255, 255), (int(wp.u), int(wp.v)), 2)

                # HUD
                avg_inf = np.mean(inference_times[-30:]) if inference_times else 0
                fps = clock.get_fps()
                lines = [
                    f"Speed: {speed_kmh:.0f} km/h | FPS: {fps:.0f}",
                    f"Steer: {control.steer:.2f} | Nav: {nav_str}",
                    f"Inference: {avg_inf*1000:.0f}ms | WPs: {len(waypoints_3d)}",
                ]
                for i, line in enumerate(lines):
                    text = font.render(line, True, (255, 255, 0))
                    bg = pygame.Rect(5, 5 + i * 18, text.get_width() + 10, 18)
                    pygame.draw.rect(display, (0, 0, 0), bg)
                    display.blit(text, (10, 7 + i * 18))

                pygame.display.flip()
                clock.tick(60)  # DINOv2 is fast enough for 60fps render
            else:
                world.tick()
                time.sleep(0.016)

            if frame % 60 == 0:
                avg_inf = np.mean(inference_times[-60:]) * 1000 if inference_times else 0
                print(f"\rSpeed: {speed_kmh:.0f}km/h | Nav: {nav_str:<15} | "
                      f"Inf: {avg_inf:.0f}ms | WPs: {len(waypoints_3d)}", end='')

            frame += 1

    finally:
        print("\n\nCleaning up...")
        for actor in actors:
            actor.destroy()
        if display:
            pygame.quit()

        elapsed = time.time() - start_time
        avg_inf = np.mean(inference_times) * 1000 if inference_times else 0
        print(f"\nSession: {elapsed/60:.1f}min, {len(inference_times)} inferences, avg {avg_inf:.0f}ms")


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='DINOv2 Autonomous Driving')
    parser.add_argument('--checkpoint', type=str, required=True,
                        help='Path to trained heads checkpoint (.pt)')
    parser.add_argument('--speed', type=float, default=20.0, help='Target speed km/h')
    parser.add_argument('--spawn-x', type=float, default=None)
    parser.add_argument('--spawn-y', type=float, default=None)
    parser.add_argument('--spawn-z', type=float, default=None)
    parser.add_argument('--spawn-yaw', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=None, help='Max duration seconds')
    parser.add_argument('--no-display', action='store_true')
    args = parser.parse_args()

    run_dino_drive(
        checkpoint=args.checkpoint,
        target_speed_kmh=args.speed,
        use_display=not args.no_display,
        spawn_x=args.spawn_x,
        spawn_y=args.spawn_y,
        spawn_z=args.spawn_z,
        spawn_yaw=args.spawn_yaw,
        duration=args.duration,
    )
