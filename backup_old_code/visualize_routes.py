#!/usr/bin/env python3
"""
Route Visualization Tool

Loads all routes from ./routes/ directory and displays them in pygame.
Shows route paths with different colors for each route type.
"""

import json
import math
from pathlib import Path
from typing import List, Dict, Tuple
import pygame
import sys

# =============================================================================
# CONFIGURATION
# =============================================================================

WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
BACKGROUND_COLOR = (30, 30, 40)
GRID_COLOR = (50, 50, 60)
TEXT_COLOR = (200, 200, 200)

# Route colors by type (based on filename prefix)
ROUTE_COLORS = {
    'left': (255, 100, 100),      # Red
    'right': (100, 255, 100),     # Green
    'straight': (100, 100, 255),  # Blue
    'loop': (255, 255, 100),      # Yellow
    'mix': (255, 100, 255),       # Magenta
    'default': (150, 150, 150)    # Gray
}

CHECKPOINT_RADIUS = 3
START_MARKER_RADIUS = 8
END_MARKER_RADIUS = 6

# Single track view options
SHOW_DIRECTION_ARROWS = True
SHOW_CHECKPOINT_NUMBERS = True
SHOW_DISTANCE_MARKERS = True
ARROW_SPACING = 5  # Draw arrow every N checkpoints
DISTANCE_MARKER_INTERVAL = 10.0  # meters

# =============================================================================
# ROUTE LOADER
# =============================================================================

class RouteVisualizer:
    def __init__(self, routes_dir: str):
        self.routes_dir = Path(routes_dir)
        self.routes = []
        self.load_all_routes()

        # Calculate bounds for scaling
        self.calculate_bounds()

    def load_all_routes(self):
        """Load all route JSON files from directory."""
        json_files = sorted(self.routes_dir.glob('*.json'))

        if not json_files:
            raise ValueError(f"No JSON files found in {self.routes_dir}")

        print(f"Loading routes from {self.routes_dir}...")
        for filepath in json_files:
            try:
                route = self.load_route(filepath)
                self.routes.append(route)
                print(f"  ✓ Loaded: {filepath.stem} ({len(route['checkpoints'])} checkpoints)")
            except Exception as e:
                print(f"  ✗ Error loading {filepath.name}: {e}")

        print(f"\nTotal routes loaded: {len(self.routes)}\n")

    def load_route(self, filepath: Path) -> Dict:
        """Load a single route file."""
        with open(filepath, 'r') as f:
            route_data = json.load(f)

        route_name = filepath.stem

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

        # Parse checkpoints (only need x, y for visualization)
        checkpoints = []
        for cp in checkpoint_data:
            checkpoints.append({
                'x': cp['x'],
                'y': cp['y'],
                'z': cp.get('z', 0.0)
            })

        # Determine route type from filename
        route_type = 'default'
        for prefix in ROUTE_COLORS.keys():
            if route_name.startswith(prefix):
                route_type = prefix
                break

        return {
            'name': route_name,
            'checkpoints': checkpoints,
            'total_distance': total_distance,
            'type': route_type,
            'color': ROUTE_COLORS.get(route_type, ROUTE_COLORS['default'])
        }

    def calculate_bounds(self, route_idx=None):
        """Calculate min/max coordinates for all routes or a single route."""
        if not self.routes:
            self.min_x = self.max_x = 0
            self.min_y = self.max_y = 0
            return

        all_x = []
        all_y = []

        # If route_idx specified, only use that route
        routes_to_use = [self.routes[route_idx]] if route_idx is not None else self.routes

        for route in routes_to_use:
            for cp in route['checkpoints']:
                all_x.append(cp['x'])
                all_y.append(cp['y'])

        self.min_x = min(all_x)
        self.max_x = max(all_x)
        self.min_y = min(all_y)
        self.max_y = max(all_y)

        # Add padding (10% on each side)
        x_range = self.max_x - self.min_x
        y_range = self.max_y - self.min_y

        padding_x = x_range * 0.1 if x_range > 0 else 10
        padding_y = y_range * 0.1 if y_range > 0 else 10

        self.min_x -= padding_x
        self.max_x += padding_x
        self.min_y -= padding_y
        self.max_y += padding_y

        self.x_range = self.max_x - self.min_x
        self.y_range = self.max_y - self.min_y

        if route_idx is None:
            print(f"World bounds:")
            print(f"  X: [{self.min_x:.1f}, {self.max_x:.1f}] (range: {self.x_range:.1f}m)")
            print(f"  Y: [{self.min_y:.1f}, {self.max_y:.1f}] (range: {self.y_range:.1f}m)")

    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (CARLA) to screen coordinates (pygame)."""
        # Normalize to [0, 1]
        norm_x = (x - self.min_x) / self.x_range if self.x_range > 0 else 0.5
        norm_y = (y - self.min_y) / self.y_range if self.y_range > 0 else 0.5

        # Convert to screen space (flip Y axis for pygame)
        screen_x = int(norm_x * WINDOW_WIDTH)
        screen_y = int((1.0 - norm_y) * WINDOW_HEIGHT)  # Flip Y

        return screen_x, screen_y

    def draw_grid(self, screen):
        """Draw background grid."""
        # Vertical lines
        for i in range(0, WINDOW_WIDTH, 50):
            pygame.draw.line(screen, GRID_COLOR, (i, 0), (i, WINDOW_HEIGHT), 1)

        # Horizontal lines
        for i in range(0, WINDOW_HEIGHT, 50):
            pygame.draw.line(screen, GRID_COLOR, (0, i), (WINDOW_WIDTH, i), 1)

    def draw_routes(self, screen, show_checkpoints=True, selected_route=None,
                    show_arrows=False, show_numbers=False, show_distances=False):
        """Draw all routes on screen."""
        font = pygame.font.SysFont('monospace', 10)

        for idx, route in enumerate(self.routes):
            checkpoints = route['checkpoints']
            color = route['color']

            # If a route is selected, dim others
            if selected_route is not None and idx != selected_route:
                color = tuple(c // 3 for c in color)  # Dim color

            # Draw route path (lines connecting checkpoints)
            points = []
            for cp in checkpoints:
                screen_x, screen_y = self.world_to_screen(cp['x'], cp['y'])
                points.append((screen_x, screen_y))

            # Draw path
            if len(points) >= 2:
                pygame.draw.lines(screen, color, False, points, 2)

            # Draw checkpoints (optional)
            if show_checkpoints and (selected_route is None or idx == selected_route):
                for point in points:
                    pygame.draw.circle(screen, color, point, CHECKPOINT_RADIUS)

            # Draw direction arrows (only for selected route)
            if show_arrows and selected_route == idx and len(points) >= 2:
                for i in range(0, len(points) - 1, ARROW_SPACING):
                    self._draw_arrow(screen, points[i], points[i + 1], color)

            # Draw checkpoint numbers (only for selected route)
            if show_numbers and selected_route == idx:
                for i, point in enumerate(points):
                    text = font.render(str(i), True, (255, 255, 255))
                    text_rect = text.get_rect(center=(point[0], point[1] - 12))
                    # Draw background for text
                    bg_rect = text_rect.inflate(4, 2)
                    pygame.draw.rect(screen, (0, 0, 0), bg_rect)
                    screen.blit(text, text_rect)

            # Draw distance markers (only for selected route)
            if show_distances and selected_route == idx and len(checkpoints) >= 2:
                cumulative_distance = 0.0
                next_marker = DISTANCE_MARKER_INTERVAL

                for i in range(1, len(checkpoints)):
                    # Calculate segment distance
                    dx = checkpoints[i]['x'] - checkpoints[i-1]['x']
                    dy = checkpoints[i]['y'] - checkpoints[i-1]['y']
                    segment_dist = math.sqrt(dx*dx + dy*dy)
                    cumulative_distance += segment_dist

                    # Draw marker if we've passed a threshold
                    while cumulative_distance >= next_marker:
                        # Interpolate position
                        excess = cumulative_distance - next_marker
                        ratio = 1.0 - (excess / segment_dist) if segment_dist > 0 else 0

                        marker_x = checkpoints[i-1]['x'] + (checkpoints[i]['x'] - checkpoints[i-1]['x']) * ratio
                        marker_y = checkpoints[i-1]['y'] + (checkpoints[i]['y'] - checkpoints[i-1]['y']) * ratio

                        screen_x, screen_y = self.world_to_screen(marker_x, marker_y)

                        # Draw marker
                        pygame.draw.circle(screen, (255, 255, 0), (screen_x, screen_y), 5)
                        pygame.draw.circle(screen, (0, 0, 0), (screen_x, screen_y), 5, 2)

                        # Draw distance label
                        dist_text = font.render(f"{next_marker:.0f}m", True, (255, 255, 255))
                        text_rect = dist_text.get_rect(center=(screen_x + 15, screen_y))
                        bg_rect = text_rect.inflate(4, 2)
                        pygame.draw.rect(screen, (0, 0, 0), bg_rect)
                        screen.blit(dist_text, text_rect)

                        next_marker += DISTANCE_MARKER_INTERVAL

            # Draw start marker (green circle)
            if points:
                start_color = (0, 255, 0) if (selected_route is None or idx == selected_route) else (0, 100, 0)
                pygame.draw.circle(screen, start_color, points[0], START_MARKER_RADIUS)
                pygame.draw.circle(screen, (0, 0, 0), points[0], START_MARKER_RADIUS, 2)

            # Draw end marker (red circle)
            if points:
                end_color = (255, 0, 0) if (selected_route is None or idx == selected_route) else (100, 0, 0)
                pygame.draw.circle(screen, end_color, points[-1], END_MARKER_RADIUS)
                pygame.draw.circle(screen, (0, 0, 0), points[-1], END_MARKER_RADIUS, 2)

    def _draw_arrow(self, screen, start, end, color):
        """Draw an arrow from start to end point."""
        # Calculate direction
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx*dx + dy*dy)

        if length < 0.1:
            return

        # Normalize
        dx /= length
        dy /= length

        # Arrow head size
        arrow_size = 8

        # Calculate arrow midpoint
        mid_x = (start[0] + end[0]) / 2
        mid_y = (start[1] + end[1]) / 2

        # Arrow head points
        left_x = mid_x - arrow_size * (dx * 0.5 + dy * 0.866)
        left_y = mid_y - arrow_size * (dy * 0.5 - dx * 0.866)

        right_x = mid_x - arrow_size * (dx * 0.5 - dy * 0.866)
        right_y = mid_y - arrow_size * (dy * 0.5 + dx * 0.866)

        # Draw arrow head
        pygame.draw.polygon(screen, color, [
            (mid_x, mid_y),
            (left_x, left_y),
            (right_x, right_y)
        ])

    def draw_legend(self, screen):
        """Draw legend showing route types."""
        font = pygame.font.SysFont('monospace', 14)
        y_offset = 10

        # Title
        title = font.render("Route Types:", True, TEXT_COLOR)
        screen.blit(title, (10, y_offset))
        y_offset += 25

        # Count routes by type
        type_counts = {}
        for route in self.routes:
            route_type = route['type']
            type_counts[route_type] = type_counts.get(route_type, 0) + 1

        # Draw each type
        for route_type, color in sorted(ROUTE_COLORS.items()):
            if route_type == 'default':
                continue

            count = type_counts.get(route_type, 0)
            if count > 0:
                # Color box
                pygame.draw.rect(screen, color, (10, y_offset, 20, 15))
                pygame.draw.rect(screen, TEXT_COLOR, (10, y_offset, 20, 15), 1)

                # Label
                label = font.render(f"{route_type}: {count}", True, TEXT_COLOR)
                screen.blit(label, (35, y_offset))

                y_offset += 20

        # Total
        y_offset += 10
        total = font.render(f"Total: {len(self.routes)} routes", True, TEXT_COLOR)
        screen.blit(total, (10, y_offset))

    def draw_info(self, screen, selected_route=None, show_arrows=False,
                  show_numbers=False, show_distances=False, zoom_mode=False):
        """Draw info panel."""
        font = pygame.font.SysFont('monospace', 12)

        # Controls
        y_offset = WINDOW_HEIGHT - 140

        if selected_route is not None:
            controls = [
                "Controls:",
                "  LEFT/RIGHT: Prev/Next route",
                "  0: Show all routes",
                "  C: Toggle checkpoints",
                "  A: Toggle arrows",
                "  N: Toggle numbers",
                "  D: Toggle distances",
                "  Z: Toggle zoom",
                "  ESC/Q: Quit"
            ]
        else:
            controls = [
                "Controls:",
                "  1-9: Select route",
                "  0: Show all routes",
                "  C: Toggle checkpoints",
                "  ESC/Q: Quit"
            ]

        for line in controls:
            text = font.render(line, True, TEXT_COLOR)
            screen.blit(text, (10, y_offset))
            y_offset += 15

        # Selected route detailed info
        if selected_route is not None and 0 <= selected_route < len(self.routes):
            route = self.routes[selected_route]

            # Calculate curvature metrics
            checkpoints = route['checkpoints']
            angles = []
            if len(checkpoints) >= 3:
                for i in range(1, len(checkpoints) - 1):
                    # Calculate angle between segments
                    dx1 = checkpoints[i]['x'] - checkpoints[i-1]['x']
                    dy1 = checkpoints[i]['y'] - checkpoints[i-1]['y']
                    dx2 = checkpoints[i+1]['x'] - checkpoints[i]['x']
                    dy2 = checkpoints[i+1]['y'] - checkpoints[i]['y']

                    angle1 = math.atan2(dy1, dx1)
                    angle2 = math.atan2(dy2, dx2)

                    angle_diff = abs(math.degrees(angle2 - angle1))
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff

                    angles.append(angle_diff)

            avg_angle = sum(angles) / len(angles) if angles else 0
            max_angle = max(angles) if angles else 0

            info_lines = [
                f"Route: {route['name']}",
                f"Type: {route['type']}",
                f"",
                f"Checkpoints: {len(checkpoints)}",
                f"Total distance: {route['total_distance']:.1f}m",
                f"",
                f"Avg turn: {avg_angle:.1f}°",
                f"Max turn: {max_angle:.1f}°",
                f"",
                f"View options:",
                f"  Arrows: {'ON' if show_arrows else 'OFF'}",
                f"  Numbers: {'ON' if show_numbers else 'OFF'}",
                f"  Distances: {'ON' if show_distances else 'OFF'}",
                f"  Zoom: {'ON' if zoom_mode else 'OFF'}"
            ]

            x_offset = WINDOW_WIDTH - 220
            y_offset = 10

            # Draw background box
            box_height = len(info_lines) * 15 + 10
            pygame.draw.rect(screen, (20, 20, 30), (x_offset - 5, y_offset - 5, 215, box_height))
            pygame.draw.rect(screen, (100, 100, 100), (x_offset - 5, y_offset - 5, 215, box_height), 1)

            for line in info_lines:
                text = font.render(line, True, TEXT_COLOR)
                screen.blit(text, (x_offset, y_offset))
                y_offset += 15

# =============================================================================
# MAIN VISUALIZATION LOOP
# =============================================================================

def visualize_routes(routes_dir: str = './routes'):
    """Main visualization function."""

    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption('Route Visualizer')
    clock = pygame.time.Clock()

    # Load routes
    visualizer = RouteVisualizer(routes_dir)

    if len(visualizer.routes) == 0:
        print("No routes to visualize!")
        return

    # State
    running = True
    show_checkpoints = True
    selected_route = None  # None = show all
    show_arrows = False
    show_numbers = False
    show_distances = False
    zoom_mode = False

    print(f"\n{'='*70}")
    print("ROUTE VISUALIZER")
    print(f"{'='*70}")
    print("Controls:")
    print("  1-9: Select individual route")
    print("  LEFT/RIGHT: Navigate between routes")
    print("  0: Show all routes")
    print("  C: Toggle checkpoints")
    print("  A: Toggle direction arrows")
    print("  N: Toggle checkpoint numbers")
    print("  D: Toggle distance markers")
    print("  Z: Toggle zoom (fit selected route)")
    print("  ESC/Q: Quit")
    print(f"{'='*70}\n")

    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    running = False

                elif event.key == pygame.K_c:
                    show_checkpoints = not show_checkpoints
                    print(f"Checkpoints: {'ON' if show_checkpoints else 'OFF'}")

                elif event.key == pygame.K_a:
                    show_arrows = not show_arrows
                    print(f"Direction arrows: {'ON' if show_arrows else 'OFF'}")

                elif event.key == pygame.K_n:
                    show_numbers = not show_numbers
                    print(f"Checkpoint numbers: {'ON' if show_numbers else 'OFF'}")

                elif event.key == pygame.K_d:
                    show_distances = not show_distances
                    print(f"Distance markers: {'ON' if show_distances else 'OFF'}")

                elif event.key == pygame.K_z:
                    zoom_mode = not zoom_mode
                    if zoom_mode and selected_route is not None:
                        visualizer.calculate_bounds(selected_route)
                        print("Zoom: ON (fitted to selected route)")
                    else:
                        visualizer.calculate_bounds()
                        print("Zoom: OFF (showing all routes)")

                elif event.key == pygame.K_0:
                    selected_route = None
                    zoom_mode = False
                    visualizer.calculate_bounds()
                    print("Showing all routes")

                elif event.key == pygame.K_LEFT:
                    if selected_route is not None:
                        selected_route = (selected_route - 1) % len(visualizer.routes)
                        route = visualizer.routes[selected_route]
                        if zoom_mode:
                            visualizer.calculate_bounds(selected_route)
                        print(f"← Previous: {route['name']} ({len(route['checkpoints'])} checkpoints, {route['total_distance']:.1f}m)")
                    else:
                        selected_route = len(visualizer.routes) - 1
                        route = visualizer.routes[selected_route]
                        print(f"Selected: {route['name']}")

                elif event.key == pygame.K_RIGHT:
                    if selected_route is not None:
                        selected_route = (selected_route + 1) % len(visualizer.routes)
                        route = visualizer.routes[selected_route]
                        if zoom_mode:
                            visualizer.calculate_bounds(selected_route)
                        print(f"→ Next: {route['name']} ({len(route['checkpoints'])} checkpoints, {route['total_distance']:.1f}m)")
                    else:
                        selected_route = 0
                        route = visualizer.routes[selected_route]
                        print(f"Selected: {route['name']}")

                elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
                                   pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9]:
                    route_num = event.key - pygame.K_1  # 0-8
                    if route_num < len(visualizer.routes):
                        selected_route = route_num
                        route = visualizer.routes[selected_route]
                        if zoom_mode:
                            visualizer.calculate_bounds(selected_route)
                        print(f"Selected: {route['name']} ({len(route['checkpoints'])} checkpoints, {route['total_distance']:.1f}m)")
                    else:
                        print(f"Route {route_num + 1} does not exist (only {len(visualizer.routes)} routes loaded)")

        # Drawing
        screen.fill(BACKGROUND_COLOR)

        # Draw grid
        visualizer.draw_grid(screen)

        # Draw routes
        visualizer.draw_routes(screen, show_checkpoints, selected_route,
                              show_arrows, show_numbers, show_distances)

        # Draw legend (only when showing all routes)
        if selected_route is None:
            visualizer.draw_legend(screen)

        # Draw info
        visualizer.draw_info(screen, selected_route, show_arrows,
                           show_numbers, show_distances, zoom_mode)

        # Update display
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    print("\nVisualization closed.")

# =============================================================================
# CLI
# =============================================================================

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description='Visualize routes stored in JSON files',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--routes', type=str, default='./routes',
                        help='Directory containing route JSON files (default: ./routes)')

    args = parser.parse_args()

    try:
        visualize_routes(args.routes)
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)
