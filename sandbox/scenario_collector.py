#!/usr/bin/env python3
"""
Scenario-Based Data Collection with Active Routing

Actively drives the vehicle to specific scenarios (intersections, turns)
and forces the desired maneuver.

Scenarios:
- right_turn: Right turn at intersection
- left_turn: Left turn at intersection
- sharp_right: Sharp right turn (>60°) at intersection
- sharp_left: Sharp left turn (>60°) at intersection
- lane_keeping: Straight driving

Usage:
    python scenario_collector.py --scenario right_turn --count 10
    python scenario_collector.py --all --count 10
    python scenario_collector.py --scenarios right_turn:10,left_turn:10,lane_keeping:20
"""

import carla
import numpy as np
import math
import time
import argparse
import signal
import sys
import json
import random
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from enum import Enum

from traj_planner import (
    CameraConfig, TrajectoryEncoder, TrajectoryDecoder,
    Waypoint3D
)
from pid import PurePursuitController, ControllerConfig
from data_collector import AsyncDataCollector

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================

class TurnDirection(Enum):
    LEFT = "left"
    RIGHT = "right"
    STRAIGHT = "straight"
    SHARP_LEFT = "sharp_left"
    SHARP_RIGHT = "sharp_right"


@dataclass
class ScenarioConfig:
    name: str
    turn_direction: TurnDirection
    requires_intersection: bool
    description: str


SCENARIOS = {
    'lane_keeping': ScenarioConfig(
        name='lane_keeping',
        turn_direction=TurnDirection.STRAIGHT,
        requires_intersection=False,
        description='Straight driving'
    ),
    'right_turn': ScenarioConfig(
        name='right_turn',
        turn_direction=TurnDirection.RIGHT,
        requires_intersection=True,
        description='Right turn at intersection'
    ),
    'left_turn': ScenarioConfig(
        name='left_turn',
        turn_direction=TurnDirection.LEFT,
        requires_intersection=True,
        description='Left turn at intersection'
    ),
    'sharp_right': ScenarioConfig(
        name='sharp_right',
        turn_direction=TurnDirection.SHARP_RIGHT,
        requires_intersection=True,
        description='Sharp right turn (>60°)'
    ),
    'sharp_left': ScenarioConfig(
        name='sharp_left',
        turn_direction=TurnDirection.SHARP_LEFT,
        requires_intersection=True,
        description='Sharp left turn (>60°)'
    ),
}


# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

CAMERA_CONFIG = CameraConfig(
    width=WINDOW_WIDTH,
    height=WINDOW_HEIGHT,
    fov=90,
    x=2.0, y=0.0, z=1.8,
    pitch=-15, yaw=0, roll=0
)

WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]

CONTROLLER_CONFIG = ControllerConfig(
    target_speed_kmh=30.0,  # Slower for turns
    max_throttle=0.5,
    max_brake=0.6,
    max_steering=0.9,
    lookahead_distance=5.0,
    speed_kp=0.5,
    speed_ki=0.05,
    speed_kd=0.1
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
# SCENARIO-AWARE WAYPOINT GENERATOR
# =============================================================================

class ScenarioWaypointGenerator:
    """
    Generates waypoints that FORCE a specific turn direction at intersections.
    """

    def __init__(self, carla_map, distances: List[float] = None):
        self.map = carla_map
        self.distances = distances or WAYPOINT_DISTANCES
        self._target_direction: Optional[TurnDirection] = None
        self._in_junction = False
        self._junction_entered_at = None

    def set_target_direction(self, direction: TurnDirection):
        """Set the desired turn direction for next intersection."""
        self._target_direction = direction

    def get_waypoints(self, vehicle) -> Tuple[List[Waypoint3D], bool, float]:
        """
        Get waypoints that steer toward the target direction.

        Returns:
            Tuple of (waypoints, is_at_intersection, angle_change)
        """
        vehicle_location = vehicle.get_transform().location
        vehicle_yaw = vehicle.get_transform().rotation.yaw
        current_wp = self.map.get_waypoint(vehicle_location)

        if current_wp is None:
            return [], False, 0.0

        waypoints = []
        tracking_wp = current_wp
        at_intersection = current_wp.is_junction

        for i, distance in enumerate(self.distances):
            next_wps = tracking_wp.next(distance if i == 0 else (self.distances[i] - self.distances[i-1]))

            if not next_wps:
                if waypoints:
                    waypoints.append(waypoints[-1])
                continue

            # At intersection with multiple options
            if len(next_wps) > 1:
                at_intersection = True
                wp = self._select_waypoint_for_direction(
                    next_wps, vehicle_yaw, tracking_wp
                )
            else:
                wp = next_wps[0]

            waypoints.append(Waypoint3D(
                x=wp.transform.location.x,
                y=wp.transform.location.y,
                z=wp.transform.location.z
            ))
            tracking_wp = wp

        # Calculate angle change
        angle_change = self._calculate_angle_change(waypoints) if len(waypoints) >= 3 else 0.0

        return waypoints, at_intersection, angle_change

    def _select_waypoint_for_direction(self, waypoints, vehicle_yaw, current_wp) -> 'carla.Waypoint':
        """Select waypoint that best matches target direction."""
        if self._target_direction is None or self._target_direction == TurnDirection.STRAIGHT:
            # Find straightest path
            best_wp = waypoints[0]
            min_angle = float('inf')
            for wp in waypoints:
                angle_diff = abs(self._normalize_angle(wp.transform.rotation.yaw - vehicle_yaw))
                if angle_diff < min_angle:
                    min_angle = angle_diff
                    best_wp = wp
            return best_wp

        # Calculate angle to each option
        options = []
        for wp in waypoints:
            angle_diff = self._normalize_angle(wp.transform.rotation.yaw - vehicle_yaw)
            options.append((wp, angle_diff))

        # Sort by angle (negative = right, positive = left)
        options.sort(key=lambda x: x[1])

        if self._target_direction == TurnDirection.RIGHT:
            # Pick rightmost (most negative angle)
            return options[0][0]
        elif self._target_direction == TurnDirection.SHARP_RIGHT:
            # Pick rightmost with angle > 45°
            for wp, angle in options:
                if angle < -45:
                    return wp
            return options[0][0]  # Fallback to rightmost
        elif self._target_direction == TurnDirection.LEFT:
            # Pick leftmost (most positive angle)
            return options[-1][0]
        elif self._target_direction == TurnDirection.SHARP_LEFT:
            # Pick leftmost with angle > 45°
            for wp, angle in reversed(options):
                if angle > 45:
                    return wp
            return options[-1][0]  # Fallback to leftmost

        return waypoints[0]

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180]."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _calculate_angle_change(self, waypoints: List[Waypoint3D]) -> float:
        """Calculate trajectory angle change in degrees."""
        if len(waypoints) < 3:
            return 0.0

        wp0 = np.array([waypoints[0].x, waypoints[0].y])
        wp_mid = np.array([waypoints[len(waypoints)//2].x, waypoints[len(waypoints)//2].y])
        wp_last = np.array([waypoints[-1].x, waypoints[-1].y])

        vec1 = wp_mid - wp0
        vec2 = wp_last - wp_mid

        if np.linalg.norm(vec1) < 0.1 or np.linalg.norm(vec2) < 0.1:
            return 0.0

        angle1 = np.arctan2(vec1[1], vec1[0])
        angle2 = np.arctan2(vec2[1], vec2[0])

        angle_change = np.degrees(angle2 - angle1)
        return self._normalize_angle(angle_change)

    def reset(self):
        """Reset state."""
        self._in_junction = False
        self._junction_entered_at = None


# =============================================================================
# JUNCTION FINDER
# =============================================================================

def find_spawn_near_junction(carla_map, spawn_points, target_direction: TurnDirection) -> Optional[carla.Transform]:
    """
    Find a spawn point that leads to a junction with the desired turn option.
    """
    random.shuffle(spawn_points)

    for spawn in spawn_points:
        wp = carla_map.get_waypoint(spawn.location)
        if wp is None:
            continue

        # Look ahead for a junction
        current = wp
        distance_traveled = 0
        max_search_distance = 100  # meters

        while distance_traveled < max_search_distance:
            next_wps = current.next(5.0)
            if not next_wps:
                break

            # Check if any path leads to junction with multiple options
            for next_wp in next_wps:
                if next_wp.is_junction:
                    # Check if junction has the turn we want
                    junction_exits = next_wp.next(5.0)
                    if junction_exits and len(junction_exits) > 1:
                        # Check if desired direction is available
                        if _has_turn_direction(junction_exits, current.transform.rotation.yaw, target_direction):
                            return spawn

            current = next_wps[0]
            distance_traveled += 5.0

    # Fallback: return random spawn
    return random.choice(spawn_points) if spawn_points else None


def _has_turn_direction(waypoints, current_yaw: float, target: TurnDirection) -> bool:
    """Check if any waypoint matches the target turn direction."""
    for wp in waypoints:
        angle_diff = wp.transform.rotation.yaw - current_yaw
        # Normalize
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        if target == TurnDirection.LEFT and angle_diff > 30:
            return True
        elif target == TurnDirection.RIGHT and angle_diff < -30:
            return True
        elif target == TurnDirection.SHARP_LEFT and angle_diff > 60:
            return True
        elif target == TurnDirection.SHARP_RIGHT and angle_diff < -60:
            return True
        elif target == TurnDirection.STRAIGHT and abs(angle_diff) < 20:
            return True

    return False


# =============================================================================
# CLASSIFY RECORDED SCENARIO
# =============================================================================

def classify_turn(angle_change: float, at_intersection: bool) -> str:
    """Classify the turn based on angle and intersection status."""
    if not at_intersection:
        return 'lane_keeping'

    if angle_change > 60:
        return 'sharp_left'
    elif angle_change > 25:
        return 'left_turn'
    elif angle_change < -60:
        return 'sharp_right'
    elif angle_change < -25:
        return 'right_turn'
    else:
        return 'lane_keeping'


# =============================================================================
# MAIN COLLECTION
# =============================================================================

def collect_scenarios(
    target_scenarios: Dict[str, int],
    output_dir: str,
    use_display: bool = True,
    min_speed_kmh: float = 8.0,
    max_time_minutes: float = 60.0
):
    """
    Collect specific driving scenarios by actively routing to them.
    """

    # Initialize display
    display = None
    clock = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('Scenario Collection')
        clock = pygame.time.Clock()

    print(f"\n{'='*60}")
    print("SCENARIO-BASED DATA COLLECTION")
    print(f"{'='*60}")
    print(f"Output: {output_dir}")
    print(f"\nTarget scenarios:")
    total_needed = 0
    for name, count in target_scenarios.items():
        config = SCENARIOS.get(name)
        if config:
            print(f"  - {name}: {count} samples ({config.description})")
            total_needed += count
    print(f"\nTotal: {total_needed} samples")
    print(f"{'='*60}\n")

    # Connect to CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    carla_map = world.get_map()

    spawn_points = carla_map.get_spawn_points()
    print(f"Found {len(spawn_points)} spawn points")

    # Initialize components
    waypoint_gen = ScenarioWaypointGenerator(carla_map, distances=WAYPOINT_DISTANCES)
    encoder = TrajectoryEncoder(CAMERA_CONFIG)
    decoder = TrajectoryDecoder(CAMERA_CONFIG)
    controller = PurePursuitController(CONTROLLER_CONFIG)

    # Data collector
    data_collector = AsyncDataCollector(
        output_dir=output_dir,
        max_queue_size=200,
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

    # Track counts
    scenario_counts = {name: 0 for name in target_scenarios}

    actors = []
    vehicle = None
    camera = None

    # Current target scenario
    current_target = None
    frames_at_current_target = 0

    # Signal handler
    shutdown_requested = False
    def signal_handler(sig, frame):
        nonlocal shutdown_requested
        print("\n\nShutdown requested...")
        shutdown_requested = True
    signal.signal(signal.SIGINT, signal_handler)

    def get_next_needed_scenario() -> Optional[str]:
        """Get a scenario that still needs samples."""
        needed = [name for name, target in target_scenarios.items()
                  if scenario_counts.get(name, 0) < target]
        if not needed:
            return None
        return random.choice(needed)

    def all_targets_met():
        return all(scenario_counts.get(name, 0) >= count
                   for name, count in target_scenarios.items())

    try:
        # Initial spawn
        current_target = get_next_needed_scenario()
        if current_target:
            config = SCENARIOS[current_target]
            waypoint_gen.set_target_direction(config.turn_direction)

            # Find good spawn point
            spawn = find_spawn_near_junction(carla_map, spawn_points, config.turn_direction)
        else:
            spawn = random.choice(spawn_points)

        # Spawn vehicle
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn)
        actors.append(vehicle)
        print(f"Vehicle spawned, targeting: {current_target}")

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
        print("Starting collection...\n")

        start_time = time.time()
        frame = 0
        stuck_counter = 0
        last_position = None
        last_recorded_scenario = None

        while not all_targets_met() and not shutdown_requested:
            elapsed = time.time() - start_time
            if elapsed > max_time_minutes * 60:
                print(f"\n\nTime limit reached")
                break

            # Handle events
            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown_requested = True
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        shutdown_requested = True

            # Get vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

            # Check stuck
            current_pos = (transform.location.x, transform.location.y)
            if last_position:
                dist = math.sqrt((current_pos[0] - last_position[0])**2 +
                                 (current_pos[1] - last_position[1])**2)
                if dist < 0.1 and speed_kmh < 1.0:
                    stuck_counter += 1
                else:
                    stuck_counter = 0
            last_position = current_pos

            # Respawn if stuck or been at same target too long
            frames_at_current_target += 1
            need_respawn = stuck_counter > 150 or frames_at_current_target > 1000

            if need_respawn:
                current_target = get_next_needed_scenario()
                if current_target is None:
                    break

                config = SCENARIOS[current_target]
                waypoint_gen.set_target_direction(config.turn_direction)
                waypoint_gen.reset()

                spawn = find_spawn_near_junction(carla_map, spawn_points, config.turn_direction)
                if spawn:
                    vehicle.set_transform(spawn)

                controller.reset()
                stuck_counter = 0
                frames_at_current_target = 0
                print(f"\n→ Respawned, now targeting: {current_target}")
                time.sleep(0.5)
                continue

            # Generate waypoints with forced direction
            waypoints_3d, at_intersection, angle_change = waypoint_gen.get_waypoints(vehicle)

            if not waypoints_3d:
                world.tick()
                continue

            # Encode to 2D
            waypoints_2d, kept_indices = encoder.encode_with_indices(waypoints_3d, transform)

            # Decode for control
            if kept_indices and waypoints_3d:
                road_height = sum(waypoints_3d[i].z for i in kept_indices) / len(kept_indices)
            else:
                road_height = transform.location.z - 0.5

            waypoints_3d_decoded = decoder.decode(waypoints_2d, transform, road_height)
            waypoints_3d_kept = [waypoints_3d[i] for i in kept_indices] if kept_indices else []
            control_waypoints = waypoints_3d_decoded if waypoints_3d_decoded else waypoints_3d

            # Control
            control = controller.compute_control(control_waypoints, transform, speed_kmh)
            vehicle.apply_control(control.to_carla_control())

            # Classify what we're actually recording
            recorded_scenario = classify_turn(angle_change, at_intersection)

            # Record if conditions met and we need this scenario
            if sensor_data.rgb_image is not None:
                should_record = (
                    speed_kmh >= min_speed_kmh and
                    waypoints_2d and len(waypoints_2d) >= 5 and
                    recorded_scenario in target_scenarios and
                    scenario_counts[recorded_scenario] < target_scenarios[recorded_scenario]
                )

                if should_record:
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
                        timestamp=sensor_data.timestamp,
                        extra_metadata={
                            'scenario': recorded_scenario,
                            'angle_change': angle_change,
                            'at_intersection': at_intersection,
                            'target_scenario': current_target
                        }
                    )

                    scenario_counts[recorded_scenario] += 1

                    # Switch target if we got enough of current
                    if scenario_counts.get(current_target, 0) >= target_scenarios.get(current_target, 0):
                        current_target = get_next_needed_scenario()
                        if current_target:
                            config = SCENARIOS[current_target]
                            waypoint_gen.set_target_direction(config.turn_direction)
                            print(f"\n✓ {recorded_scenario} complete! Now targeting: {current_target}")

            # Display
            if display and sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(sensor_data.rgb_image.swapaxes(0, 1))
                display.blit(surface, (0, 0))

                # Status overlay
                font = pygame.font.SysFont('monospace', 14)

                y = 10
                for name, target in target_scenarios.items():
                    count = scenario_counts.get(name, 0)
                    color = (0, 255, 0) if count >= target else (255, 255, 255)
                    text = font.render(f"{name}: {count}/{target}", True, color)
                    display.blit(text, (10, y))
                    y += 18

                # Current info
                target_text = font.render(f"Target: {current_target}", True, (255, 255, 0))
                display.blit(target_text, (10, WINDOW_HEIGHT - 70))

                angle_text = font.render(f"Angle: {angle_change:+.1f}° | Int: {'Y' if at_intersection else 'N'}",
                                        True, (0, 255, 255))
                display.blit(angle_text, (10, WINDOW_HEIGHT - 50))

                actual_text = font.render(f"Recording: {recorded_scenario}", True, (255, 200, 0))
                display.blit(actual_text, (10, WINDOW_HEIGHT - 30))

                pygame.display.flip()
                clock.tick(30)
            else:
                world.tick()
                time.sleep(0.033)

            frame += 1

        print(f"\n\n{'='*60}")
        print("COLLECTION COMPLETE!")
        print(f"{'='*60}")

    finally:
        print("\nFinalizing...")
        data_collector.stop()

        elapsed = time.time() - start_time
        print(f"\nResults:")
        for name, target in target_scenarios.items():
            count = scenario_counts.get(name, 0)
            status = "✓" if count >= target else "✗"
            print(f"  {status} {name}: {count}/{target}")

        print(f"\nTime: {elapsed/60:.1f} min")

        for actor in actors:
            if actor:
                actor.destroy()

        if display:
            pygame.quit()

        # Save summary
        summary_path = Path(output_dir) / 'scenario_summary.json'
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        with open(summary_path, 'w') as f:
            json.dump({
                'targets': target_scenarios,
                'collected': scenario_counts,
                'elapsed_minutes': elapsed / 60
            }, f, indent=2)
        print(f"\nSummary: {summary_path}")


# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Scenario-Based Data Collection',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scenario_collector.py --scenario right_turn --count 10
    python scenario_collector.py --all --count 10
    python scenario_collector.py --scenarios right_turn:10,left_turn:10,lane_keeping:20

Scenarios:
    lane_keeping  - Straight driving
    right_turn    - Right turn at intersection
    left_turn     - Left turn at intersection
    sharp_right   - Sharp right (>60°) at intersection
    sharp_left    - Sharp left (>60°) at intersection
        """
    )

    parser.add_argument('--scenario', type=str, help='Single scenario')
    parser.add_argument('--scenarios', type=str, help='Multiple: name:count,name:count')
    parser.add_argument('--all', action='store_true', help='All scenarios')
    parser.add_argument('--count', type=int, default=10, help='Samples per scenario')
    parser.add_argument('--output', type=str, default='./scenario_data', help='Output dir')
    parser.add_argument('--no-display', action='store_true', help='Headless mode')
    parser.add_argument('--min-speed', type=float, default=8.0, help='Min speed (km/h)')
    parser.add_argument('--max-time', type=float, default=60.0, help='Max time (minutes)')

    args = parser.parse_args()

    # Build targets
    target_scenarios = {}

    if args.all:
        target_scenarios = {name: args.count for name in SCENARIOS}
    elif args.scenario:
        if args.scenario not in SCENARIOS:
            print(f"Unknown: {args.scenario}")
            print(f"Available: {list(SCENARIOS.keys())}")
            sys.exit(1)
        target_scenarios = {args.scenario: args.count}
    elif args.scenarios:
        for item in args.scenarios.split(','):
            if ':' in item:
                name, count = item.split(':')
                target_scenarios[name.strip()] = int(count)
            else:
                target_scenarios[item.strip()] = args.count
        for name in target_scenarios:
            if name not in SCENARIOS:
                print(f"Unknown: {name}")
                sys.exit(1)
    else:
        print("Specify --scenario, --scenarios, or --all")
        parser.print_help()
        sys.exit(1)

    collect_scenarios(
        target_scenarios=target_scenarios,
        output_dir=args.output,
        use_display=not args.no_display,
        min_speed_kmh=args.min_speed,
        max_time_minutes=args.max_time
    )
