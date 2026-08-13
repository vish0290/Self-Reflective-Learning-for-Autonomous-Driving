#!/usr/bin/env python3
"""
CARLA Route Navigator with Pygame Interface

Features:
- Select start and destination spawn points via pygame UI
- Simple lane following controller
- Real-time camera feed rendering
- Route visualization on mini-map
"""

import carla
import pygame
import numpy as np
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    # Display settings
    window_width: int = 1280
    window_height: int = 720
    camera_width: int = 960
    camera_height: int = 540

    # Minimap settings
    minimap_width: int = 300
    minimap_height: int = 300
    minimap_scale: float = 3.0  # meters per pixel

    # CARLA settings
    host: str = 'localhost'
    port: int = 2000
    town: str = 'Town10HD'

    # Camera settings
    camera_fov: float = 90.0
    camera_x: float = -5.0  # Behind vehicle for 3rd person
    camera_z: float = 3.0   # Above vehicle
    camera_pitch: float = -15.0

    # Controller settings
    target_speed_kmh: float = 30.0
    lookahead_distance: float = 5.0
    min_lookahead: float = 2.0
    max_lookahead: float = 15.0


class AppState(Enum):
    SELECT_START = 1
    SELECT_DESTINATION = 2
    NAVIGATING = 3
    ARRIVED = 4


# =============================================================================
# COLORS
# =============================================================================

class Colors:
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GRAY = (128, 128, 128)
    DARK_GRAY = (40, 40, 40)
    RED = (255, 80, 80)
    GREEN = (80, 255, 80)
    BLUE = (80, 150, 255)
    YELLOW = (255, 255, 80)
    ORANGE = (255, 165, 0)
    CYAN = (80, 255, 255)
    PURPLE = (180, 100, 255)


# =============================================================================
# SIMPLE LANE FOLLOWING CONTROLLER
# =============================================================================

class LaneFollowingController:
    """
    Simple lane following controller that uses CARLA waypoint directions.
    Much more accurate than Pure Pursuit as it uses the road's actual direction.
    """

    def __init__(self, config: Config):
        self.config = config

    def compute_control(self, route_waypoints: List, current_waypoint_idx: int,
                        vehicle_transform, current_speed_kmh: float) -> Tuple[carla.VehicleControl, int]:
        """
        Compute vehicle control to follow waypoints using simple angle-based steering.

        Returns:
            Tuple of (VehicleControl, new_waypoint_idx)
        """
        control = carla.VehicleControl()

        if not route_waypoints or current_waypoint_idx >= len(route_waypoints):
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            return control, current_waypoint_idx

        # Get vehicle state
        v_loc = vehicle_transform.location
        v_yaw = vehicle_transform.rotation.yaw

        # Find the closest waypoint ahead of the vehicle
        new_idx = self._find_next_waypoint(route_waypoints, current_waypoint_idx, v_loc)

        if new_idx >= len(route_waypoints):
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            return control, new_idx

        # Get target waypoint (look a few waypoints ahead for smoother steering)
        lookahead_count = max(3, int(current_speed_kmh / 10))  # More lookahead at higher speeds
        target_idx = min(new_idx + lookahead_count, len(route_waypoints) - 1)
        target_wp = route_waypoints[target_idx]
        target_loc = target_wp.transform.location

        # Calculate steering based on angle to target
        steer = self._compute_steering(v_loc, v_yaw, target_loc)

        # Calculate throttle/brake based on speed error
        throttle, brake = self._compute_speed_control(current_speed_kmh, abs(steer))

        control.throttle = float(np.clip(throttle, 0.0, 1.0))
        control.brake = float(np.clip(brake, 0.0, 1.0))
        control.steer = float(np.clip(steer, -1.0, 1.0))

        return control, new_idx

    def _find_next_waypoint(self, waypoints: List, start_idx: int, vehicle_loc) -> int:
        """Find the next waypoint the vehicle should target."""
        # Search around current index for closest waypoint
        min_dist = float('inf')
        closest_idx = start_idx

        search_start = max(0, start_idx - 2)
        search_end = min(len(waypoints), start_idx + 30)

        for i in range(search_start, search_end):
            wp_loc = waypoints[i].transform.location
            dist = vehicle_loc.distance(wp_loc)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Move past waypoints that are behind us or very close
        while closest_idx < len(waypoints) - 1:
            wp_loc = waypoints[closest_idx].transform.location
            dist = vehicle_loc.distance(wp_loc)

            # If waypoint is close enough, move to next one
            if dist < 2.0:
                closest_idx += 1
            else:
                break

        return closest_idx

    def _compute_steering(self, vehicle_loc, vehicle_yaw: float, target_loc) -> float:
        """
        Compute steering angle based on angle difference to target.
        Uses simple trigonometry - angle to target vs vehicle heading.
        """
        # Vector from vehicle to target
        dx = target_loc.x - vehicle_loc.x
        dy = target_loc.y - vehicle_loc.y

        # Angle to target in world coordinates
        target_angle = math.degrees(math.atan2(dy, dx))

        # Angle difference (how much we need to turn)
        angle_diff = target_angle - vehicle_yaw

        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # Convert to steering value [-1, 1]
        # A 45 degree error = full steering
        # Negative because CARLA steering is inverted
        steer = angle_diff / 45.0
        steer = np.clip(steer, -1.0, 1.0)

        return float(steer)

    def _compute_speed_control(self, current_speed_kmh: float, steer_magnitude: float) -> Tuple[float, float]:
        """
        Simple speed control - reduce speed in turns.
        """
        target_speed = self.config.target_speed_kmh

        # Reduce target speed based on steering (sharper turn = slower)
        if steer_magnitude > 0.1:
            # Scale down speed: at full steering, target 40% of normal speed
            speed_factor = 1.0 - (steer_magnitude * 0.6)
            target_speed *= speed_factor

        speed_error = target_speed - current_speed_kmh

        if speed_error > 0:
            # Need to accelerate
            # Proportional control with some minimum throttle
            throttle = min(0.7, max(0.3, speed_error / 20.0))
            brake = 0.0
        else:
            # Need to slow down
            throttle = 0.0
            # Proportional braking
            brake = min(0.8, abs(speed_error) / 15.0)

        return float(throttle), float(brake)

    def reset(self):
        """Reset controller state (no state to reset in this simple controller)."""
        pass


# =============================================================================
# ROUTE NAVIGATOR
# =============================================================================

class RouteNavigator:
    def __init__(self, config: Config = None):
        self.config = config or Config()

        # CARLA
        self.client = None
        self.world = None
        self.map = None
        self.vehicle = None
        self.camera = None
        self.spectator = None

        # Spawn points
        self.spawn_points = []
        self.start_idx = None
        self.dest_idx = None

        # Route
        self.route_waypoints = []
        self.current_waypoint_idx = 0

        # Controller
        self.controller = LaneFollowingController(self.config)

        # Pygame
        self.display = None
        self.clock = None
        self.font = None
        self.font_large = None

        # Camera image
        self.camera_image = None

        # State
        self.state = AppState.SELECT_START
        self.running = True

        # UI state
        self.scroll_offset = 0
        self.hovered_spawn = None
        self.items_per_page = 15

        # Control info for display
        self.current_control = carla.VehicleControl()

    def connect(self):
        """Connect to CARLA server."""
        print(f"Connecting to CARLA at {self.config.host}:{self.config.port}...")
        self.client = carla.Client(self.config.host, self.config.port)
        self.client.set_timeout(10.0)

        print(f"Loading world: {self.config.town}")
        self.world = self.client.load_world(self.config.town)
        self.map = self.world.get_map()

        # Get spawn points
        self.spawn_points = self.map.get_spawn_points()
        print(f"Found {len(self.spawn_points)} spawn points")

        # Set synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / 30.0
        self.world.apply_settings(settings)

        self.spectator = self.world.get_spectator()

    def init_pygame(self):
        """Initialize pygame."""
        pygame.init()
        self.display = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height)
        )
        pygame.display.set_caption("CARLA Route Navigator - Lane Following")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('monospace', 16)
        self.font_large = pygame.font.SysFont('monospace', 24, bold=True)

    def spawn_vehicle(self, spawn_point):
        """Spawn vehicle at given spawn point."""
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned vehicle at spawn point")

        # Attach camera (3rd person view)
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.config.camera_width))
        camera_bp.set_attribute('image_size_y', str(self.config.camera_height))
        camera_bp.set_attribute('fov', str(self.config.camera_fov))

        camera_transform = carla.Transform(
            carla.Location(x=self.config.camera_x, z=self.config.camera_z),
            carla.Rotation(pitch=self.config.camera_pitch)
        )

        self.camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self.vehicle
        )
        self.camera.listen(self._process_camera)

    def _process_camera(self, image):
        """Process camera image."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        self.camera_image = array[:, :, :3][:, :, ::-1].copy()  # BGRA -> RGB

    def compute_route(self):
        """Compute route from start to destination using simple waypoint following."""
        if self.start_idx is None or self.dest_idx is None:
            return

        start_loc = self.spawn_points[self.start_idx].location
        dest_loc = self.spawn_points[self.dest_idx].location

        self.route_waypoints = self._compute_simple_route(start_loc, dest_loc)
        self.current_waypoint_idx = 0

        print(f"Route computed: {len(self.route_waypoints)} waypoints")

    def _compute_simple_route(self, start_loc, dest_loc, max_waypoints=2000):
        """
        Simple route computation using waypoint.next().
        Greedily follows road topology toward destination.
        """
        waypoints = []
        current_wp = self.map.get_waypoint(start_loc)

        if current_wp is None:
            return waypoints

        waypoints.append(current_wp)
        dest_distance = current_wp.transform.location.distance(dest_loc)

        for _ in range(max_waypoints):
            next_wps = current_wp.next(2.0)  # 2m steps

            if not next_wps:
                break

            # Choose waypoint that gets us closer to destination
            best_wp = None
            best_distance = float('inf')

            for wp in next_wps:
                dist = wp.transform.location.distance(dest_loc)
                if dist < best_distance:
                    best_distance = dist
                    best_wp = wp

            if best_wp is None:
                break

            waypoints.append(best_wp)
            current_wp = best_wp

            # Check if we've arrived
            if best_distance < 5.0:
                break

            # Check if we're getting much further (might be stuck in loop)
            if best_distance > dest_distance + 200.0:
                print("Warning: Route seems to be diverging, stopping")
                break

            dest_distance = min(dest_distance, best_distance)

        return waypoints

    def start_navigation(self):
        """Start navigation with Pure Pursuit controller."""
        if self.vehicle is None:
            return

        self.controller.reset()
        self.current_waypoint_idx = 0
        self.state = AppState.NAVIGATING
        print("Navigation started - Lane following controller active")

    def update_control(self):
        """Update vehicle control using Pure Pursuit."""
        if self.vehicle is None or self.state != AppState.NAVIGATING:
            return

        # Get current speed
        velocity = self.vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        # Compute control
        control, new_idx = self.controller.compute_control(
            self.route_waypoints,
            self.current_waypoint_idx,
            self.vehicle.get_transform(),
            speed_kmh
        )

        self.current_waypoint_idx = new_idx
        self.current_control = control

        # Apply control
        self.vehicle.apply_control(control)

    def check_arrival(self):
        """Check if vehicle has arrived at destination."""
        if self.vehicle is None or self.dest_idx is None:
            return False

        vehicle_loc = self.vehicle.get_location()
        dest_loc = self.spawn_points[self.dest_idx].location

        distance = vehicle_loc.distance(dest_loc)

        # Also check if we've reached the end of waypoints
        if self.current_waypoint_idx >= len(self.route_waypoints) - 5:
            return True

        return distance < 10.0

    def cleanup(self):
        """Clean up CARLA actors."""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
        if self.vehicle:
            self.vehicle.destroy()

        if self.world:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)

    def reset(self):
        """Reset for new route selection."""
        self.cleanup()
        self.vehicle = None
        self.camera = None
        self.camera_image = None
        self.route_waypoints = []
        self.current_waypoint_idx = 0
        self.start_idx = None
        self.dest_idx = None
        self.state = AppState.SELECT_START
        self.scroll_offset = 0
        self.controller.reset()

    # =========================================================================
    # RENDERING
    # =========================================================================

    def render(self):
        """Main render function."""
        self.display.fill(Colors.DARK_GRAY)

        if self.state in [AppState.SELECT_START, AppState.SELECT_DESTINATION]:
            self._render_spawn_selection()
        elif self.state in [AppState.NAVIGATING, AppState.ARRIVED]:
            self._render_navigation()

        pygame.display.flip()

    def _render_spawn_selection(self):
        """Render spawn point selection UI."""
        if self.state == AppState.SELECT_START:
            title = "SELECT START POINT"
            color = Colors.GREEN
        else:
            title = "SELECT DESTINATION"
            color = Colors.RED

        title_surface = self.font_large.render(title, True, color)
        self.display.blit(title_surface, (20, 20))

        instructions = [
            "UP/DOWN or SCROLL: Navigate list",
            "ENTER or CLICK: Select spawn point",
            "ESC: Cancel / Reset",
            "Q: Quit"
        ]
        for i, text in enumerate(instructions):
            surf = self.font.render(text, True, Colors.GRAY)
            self.display.blit(surf, (20, 60 + i * 20))

        list_x = 20
        list_y = 160
        item_height = 30

        visible_items = min(self.items_per_page, len(self.spawn_points))
        max_scroll = max(0, len(self.spawn_points) - visible_items)
        self.scroll_offset = min(self.scroll_offset, max_scroll)

        list_rect = pygame.Rect(list_x - 5, list_y - 5, 400, visible_items * item_height + 10)
        pygame.draw.rect(self.display, Colors.BLACK, list_rect)
        pygame.draw.rect(self.display, Colors.GRAY, list_rect, 1)

        mouse_pos = pygame.mouse.get_pos()
        self.hovered_spawn = None

        for i in range(visible_items):
            idx = i + self.scroll_offset
            if idx >= len(self.spawn_points):
                break

            sp = self.spawn_points[idx]
            y = list_y + i * item_height

            item_rect = pygame.Rect(list_x, y, 390, item_height - 2)
            is_hovered = item_rect.collidepoint(mouse_pos)

            if is_hovered:
                self.hovered_spawn = idx
                pygame.draw.rect(self.display, Colors.BLUE, item_rect)

            if idx == self.start_idx:
                pygame.draw.rect(self.display, Colors.GREEN, item_rect, 2)
            elif idx == self.dest_idx:
                pygame.draw.rect(self.display, Colors.RED, item_rect, 2)

            text = f"#{idx:3d}  X:{sp.location.x:7.1f}  Y:{sp.location.y:7.1f}  Z:{sp.location.z:5.1f}"
            text_color = Colors.WHITE if is_hovered else Colors.CYAN
            surf = self.font.render(text, True, text_color)
            self.display.blit(surf, (list_x + 5, y + 5))

        if len(self.spawn_points) > visible_items:
            scrollbar_x = list_x + 395
            scrollbar_height = (visible_items / len(self.spawn_points)) * (visible_items * item_height)
            scrollbar_y = list_y + (self.scroll_offset / len(self.spawn_points)) * (visible_items * item_height)
            pygame.draw.rect(self.display, Colors.GRAY,
                           (scrollbar_x, scrollbar_y, 8, scrollbar_height))

        self._render_minimap_preview()

        info_x = 450
        info_y = 160

        if self.start_idx is not None:
            sp = self.spawn_points[self.start_idx]
            surf = self.font.render(f"START: #{self.start_idx}", True, Colors.GREEN)
            self.display.blit(surf, (info_x, info_y))
            surf = self.font.render(f"  Location: ({sp.location.x:.1f}, {sp.location.y:.1f})", True, Colors.WHITE)
            self.display.blit(surf, (info_x, info_y + 20))

        if self.dest_idx is not None:
            sp = self.spawn_points[self.dest_idx]
            surf = self.font.render(f"DESTINATION: #{self.dest_idx}", True, Colors.RED)
            self.display.blit(surf, (info_x, info_y + 60))
            surf = self.font.render(f"  Location: ({sp.location.x:.1f}, {sp.location.y:.1f})", True, Colors.WHITE)
            self.display.blit(surf, (info_x, info_y + 80))

        if self.start_idx is not None and self.dest_idx is not None:
            btn_rect = pygame.Rect(info_x, info_y + 140, 200, 50)
            pygame.draw.rect(self.display, Colors.GREEN, btn_rect)
            pygame.draw.rect(self.display, Colors.WHITE, btn_rect, 2)

            btn_text = self.font_large.render("START ROUTE", True, Colors.BLACK)
            text_rect = btn_text.get_rect(center=btn_rect.center)
            self.display.blit(btn_text, text_rect)

            if pygame.mouse.get_pressed()[0] and btn_rect.collidepoint(mouse_pos):
                self._begin_route()

    def _render_minimap_preview(self):
        """Render minimap showing spawn points."""
        map_x = self.config.window_width - self.config.minimap_width - 20
        map_y = 160

        map_rect = pygame.Rect(map_x, map_y, self.config.minimap_width, self.config.minimap_height)
        pygame.draw.rect(self.display, Colors.BLACK, map_rect)
        pygame.draw.rect(self.display, Colors.GRAY, map_rect, 1)

        if not self.spawn_points:
            return

        xs = [sp.location.x for sp in self.spawn_points]
        ys = [sp.location.y for sp in self.spawn_points]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        range_x = max_x - min_x or 1
        range_y = max_y - min_y or 1

        scale = min(
            (self.config.minimap_width - 20) / range_x,
            (self.config.minimap_height - 20) / range_y
        )

        def world_to_map(x, y):
            px = map_x + 10 + (x - min_x) * scale
            py = map_y + 10 + (y - min_y) * scale
            return int(px), int(py)

        for i, sp in enumerate(self.spawn_points):
            px, py = world_to_map(sp.location.x, sp.location.y)

            if i == self.start_idx:
                pygame.draw.circle(self.display, Colors.GREEN, (px, py), 6)
            elif i == self.dest_idx:
                pygame.draw.circle(self.display, Colors.RED, (px, py), 6)
            elif i == self.hovered_spawn:
                pygame.draw.circle(self.display, Colors.YELLOW, (px, py), 5)
            else:
                pygame.draw.circle(self.display, Colors.GRAY, (px, py), 2)

        if len(self.route_waypoints) > 1:
            points = []
            for wp in self.route_waypoints[::5]:
                px, py = world_to_map(wp.transform.location.x, wp.transform.location.y)
                points.append((px, py))
            if len(points) > 1:
                pygame.draw.lines(self.display, Colors.ORANGE, False, points, 2)

    def _render_navigation(self):
        """Render navigation view with camera feed."""
        if self.camera_image is not None:
            surface = pygame.surfarray.make_surface(self.camera_image.swapaxes(0, 1))
            surface = pygame.transform.scale(surface, (self.config.camera_width, self.config.camera_height))
            self.display.blit(surface, (0, 0))

        self._render_info_panel()
        self._render_control_bars()
        self._render_navigation_minimap()

        if self.state == AppState.ARRIVED:
            overlay = pygame.Surface((self.config.window_width, 100))
            overlay.fill(Colors.GREEN)
            overlay.set_alpha(200)
            self.display.blit(overlay, (0, self.config.window_height // 2 - 50))

            text = self.font_large.render("ARRIVED AT DESTINATION!", True, Colors.BLACK)
            text_rect = text.get_rect(center=(self.config.window_width // 2, self.config.window_height // 2 - 20))
            self.display.blit(text, text_rect)

            text2 = self.font.render("Press R to select new route, Q to quit", True, Colors.BLACK)
            text2_rect = text2.get_rect(center=(self.config.window_width // 2, self.config.window_height // 2 + 20))
            self.display.blit(text2, text2_rect)

    def _render_info_panel(self):
        """Render info panel during navigation."""
        panel_x = self.config.camera_width + 10
        panel_y = 10

        if self.vehicle is None:
            return

        velocity = self.vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        location = self.vehicle.get_location()

        if self.dest_idx is not None:
            dest_loc = self.spawn_points[self.dest_idx].location
            distance = location.distance(dest_loc)
        else:
            distance = 0

        progress = (self.current_waypoint_idx / max(len(self.route_waypoints), 1)) * 100

        info_lines = [
            ("STATUS", "NAVIGATING" if self.state == AppState.NAVIGATING else "ARRIVED", Colors.GREEN),
            ("", "", Colors.WHITE),
            ("Speed", f"{speed_kmh:.1f} km/h", Colors.CYAN),
            ("Target", f"{self.config.target_speed_kmh:.0f} km/h", Colors.GRAY),
            ("", "", Colors.WHITE),
            ("Start", f"#{self.start_idx}", Colors.GREEN),
            ("Destination", f"#{self.dest_idx}", Colors.RED),
            ("Distance", f"{distance:.1f} m", Colors.YELLOW),
            ("", "", Colors.WHITE),
            ("Progress", f"{progress:.1f}%", Colors.ORANGE),
            ("Waypoint", f"{self.current_waypoint_idx}/{len(self.route_waypoints)}", Colors.ORANGE),
        ]

        for i, (label, value, color) in enumerate(info_lines):
            if label:
                text = f"{label}: {value}"
            else:
                text = ""
            surf = self.font.render(text, True, color)
            self.display.blit(surf, (panel_x, panel_y + i * 20))

        controls_y = panel_y + len(info_lines) * 20 + 10
        controls = [
            "CONTROLS:",
            "R - New route",
            "+/- - Adjust speed",
            "ESC - Stop",
            "Q - Quit"
        ]
        for i, text in enumerate(controls):
            color = Colors.GRAY if i > 0 else Colors.WHITE
            surf = self.font.render(text, True, color)
            self.display.blit(surf, (panel_x, controls_y + i * 18))

    def _render_control_bars(self):
        """Render steering, throttle, brake bars."""
        bar_x = self.config.camera_width + 10
        bar_y = 380
        bar_width = 280
        bar_height = 20

        # Steering bar
        surf = self.font.render("Steering", True, Colors.WHITE)
        self.display.blit(surf, (bar_x, bar_y))

        steer_rect = pygame.Rect(bar_x, bar_y + 18, bar_width, bar_height)
        pygame.draw.rect(self.display, Colors.DARK_GRAY, steer_rect)

        center_x = bar_x + bar_width // 2
        pygame.draw.line(self.display, Colors.WHITE, (center_x, bar_y + 18), (center_x, bar_y + 18 + bar_height), 2)

        steer_val = -self.current_control.steer
        steer_x = center_x + int(steer_val * (bar_width // 2))
        steer_color = Colors.RED if abs(steer_val) > 0.5 else Colors.CYAN
        pygame.draw.rect(self.display, steer_color, (min(center_x, steer_x), bar_y + 20, abs(steer_x - center_x), bar_height - 4))

        # Throttle bar
        bar_y += 50
        surf = self.font.render("Throttle", True, Colors.WHITE)
        self.display.blit(surf, (bar_x, bar_y))

        throttle_rect = pygame.Rect(bar_x, bar_y + 18, bar_width, bar_height)
        pygame.draw.rect(self.display, Colors.DARK_GRAY, throttle_rect)

        throttle_width = int(self.current_control.throttle * bar_width)
        pygame.draw.rect(self.display, Colors.GREEN, (bar_x, bar_y + 20, throttle_width, bar_height - 4))

        # Brake bar
        bar_y += 50
        surf = self.font.render("Brake", True, Colors.WHITE)
        self.display.blit(surf, (bar_x, bar_y))

        brake_rect = pygame.Rect(bar_x, bar_y + 18, bar_width, bar_height)
        pygame.draw.rect(self.display, Colors.DARK_GRAY, brake_rect)

        brake_width = int(self.current_control.brake * bar_width)
        pygame.draw.rect(self.display, Colors.RED, (bar_x, bar_y + 20, brake_width, bar_height - 4))

    def _render_navigation_minimap(self):
        """Render minimap during navigation."""
        map_x = self.config.camera_width + 10
        map_y = self.config.window_height - self.config.minimap_height - 10

        map_rect = pygame.Rect(map_x, map_y, self.config.minimap_width, self.config.minimap_height)
        pygame.draw.rect(self.display, Colors.BLACK, map_rect)
        pygame.draw.rect(self.display, Colors.GRAY, map_rect, 1)

        if self.vehicle is None:
            return

        vehicle_loc = self.vehicle.get_location()
        center_x = vehicle_loc.x
        center_y = vehicle_loc.y

        scale = 1.0 / self.config.minimap_scale

        def world_to_map(x, y):
            dx = (x - center_x) * scale
            dy = (y - center_y) * scale
            px = map_x + self.config.minimap_width // 2 + dx
            py = map_y + self.config.minimap_height // 2 + dy
            return int(px), int(py)

        # Draw route with progress coloring
        if len(self.route_waypoints) > 1:
            for i, wp in enumerate(self.route_waypoints):
                px, py = world_to_map(wp.transform.location.x, wp.transform.location.y)
                if map_x <= px <= map_x + self.config.minimap_width and \
                   map_y <= py <= map_y + self.config.minimap_height:
                    color = Colors.GRAY if i < self.current_waypoint_idx else Colors.ORANGE
                    pygame.draw.circle(self.display, color, (px, py), 2)

        # Draw destination
        if self.dest_idx is not None:
            dest = self.spawn_points[self.dest_idx].location
            px, py = world_to_map(dest.x, dest.y)
            pygame.draw.circle(self.display, Colors.RED, (px, py), 8)
            pygame.draw.circle(self.display, Colors.WHITE, (px, py), 8, 2)

        # Draw vehicle
        vx, vy = world_to_map(vehicle_loc.x, vehicle_loc.y)
        yaw = math.radians(self.vehicle.get_transform().rotation.yaw)
        arrow_len = 15
        ax = vx + arrow_len * math.cos(yaw)
        ay = vy + arrow_len * math.sin(yaw)

        pygame.draw.circle(self.display, Colors.GREEN, (vx, vy), 6)
        pygame.draw.line(self.display, Colors.GREEN, (vx, vy), (int(ax), int(ay)), 3)

    def _begin_route(self):
        """Begin the route navigation."""
        print(f"Starting route from #{self.start_idx} to #{self.dest_idx}")

        self.spawn_vehicle(self.spawn_points[self.start_idx])
        self.world.tick()

        self.compute_route()
        self.start_navigation()

    # =========================================================================
    # EVENT HANDLING
    # =========================================================================

    def handle_events(self):
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                self._handle_keydown(event)

            elif event.type == pygame.MOUSEWHEEL:
                self.scroll_offset -= event.y * 3
                self.scroll_offset = max(0, self.scroll_offset)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self._handle_click(event.pos)

    def _handle_keydown(self, event):
        """Handle key press."""
        if event.key == pygame.K_q:
            self.running = False

        elif event.key == pygame.K_ESCAPE:
            if self.state in [AppState.NAVIGATING, AppState.ARRIVED]:
                self.reset()
            elif self.state == AppState.SELECT_DESTINATION:
                self.state = AppState.SELECT_START
                self.dest_idx = None

        elif event.key == pygame.K_r:
            self.reset()

        elif event.key == pygame.K_UP:
            if self.state in [AppState.SELECT_START, AppState.SELECT_DESTINATION]:
                self.scroll_offset = max(0, self.scroll_offset - 1)

        elif event.key == pygame.K_DOWN:
            if self.state in [AppState.SELECT_START, AppState.SELECT_DESTINATION]:
                self.scroll_offset += 1

        elif event.key == pygame.K_RETURN:
            if self.hovered_spawn is not None:
                self._select_spawn(self.hovered_spawn)

        elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
            self.config.target_speed_kmh = min(60, self.config.target_speed_kmh + 5)
            print(f"Target speed: {self.config.target_speed_kmh} km/h")

        elif event.key == pygame.K_MINUS:
            self.config.target_speed_kmh = max(10, self.config.target_speed_kmh - 5)
            print(f"Target speed: {self.config.target_speed_kmh} km/h")

    def _handle_click(self, pos):
        """Handle mouse click."""
        if self.state in [AppState.SELECT_START, AppState.SELECT_DESTINATION]:
            if self.hovered_spawn is not None:
                self._select_spawn(self.hovered_spawn)

    def _select_spawn(self, idx):
        """Select a spawn point."""
        if self.state == AppState.SELECT_START:
            self.start_idx = idx
            self.state = AppState.SELECT_DESTINATION
            print(f"Selected start: #{idx}")

        elif self.state == AppState.SELECT_DESTINATION:
            if idx != self.start_idx:
                self.dest_idx = idx
                print(f"Selected destination: #{idx}")
                self._begin_route()

    # =========================================================================
    # MAIN LOOP
    # =========================================================================

    def run(self):
        """Main loop."""
        try:
            self.connect()
            self.init_pygame()

            while self.running:
                self.handle_events()

                # Update control if navigating
                if self.state == AppState.NAVIGATING:
                    self.update_control()

                    if self.check_arrival():
                        self.state = AppState.ARRIVED
                        if self.vehicle:
                            control = carla.VehicleControl()
                            control.brake = 1.0
                            self.vehicle.apply_control(control)
                        print("Arrived at destination!")

                self.world.tick()
                self.render()
                self.clock.tick(30)

        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
            pygame.quit()


# =============================================================================
# MAIN
# =============================================================================

def main():
    config = Config()

    if len(sys.argv) > 1:
        config.town = sys.argv[1]

    navigator = RouteNavigator(config)
    navigator.run()


if __name__ == '__main__':
    main()
