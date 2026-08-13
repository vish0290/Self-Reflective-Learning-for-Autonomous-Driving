#!/usr/bin/env python3
"""
Segmentation-Based Driver with Route Navigation

Uses CARLA's semantic segmentation camera for low-level steering
and route checkpoints for high-level navigation decisions.

Pipeline:
    Seg camera -> road mask -> extract road center path (2D pixels)
    -> TrajectoryDecoder -> 3D waypoints -> PID controller -> drive
    Route checkpoints -> navigation token (turn left/right/straight)

Usage:
    python core/basic_drive.py
    python core/basic_drive.py --route routes/left_1.json
    python core/basic_drive.py --routes-dir routes/ --speed 30
    python core/basic_drive.py --duration 120 --no-display
"""

import argparse
import math
import signal
import time

import carla
import numpy as np

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

from traj_planner import (
    CameraConfig, TrajectoryDecoder, TrajectoryEncoder, Waypoint2D,
    WaypointGenerator, Waypoint3D,
)
from pid_controller import VehiclePIDController
from route_builder import RouteBuilder
from navigation_analyzer import generate_navigation_token

# =============================================================================
# CONFIG
# =============================================================================

WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480

CAMERA_CONFIG = CameraConfig(
    width=WINDOW_WIDTH, height=WINDOW_HEIGHT,
    fov=90, x=2.0, y=0.0, z=1.8,
    pitch=-15, yaw=0, roll=0,
)

# CARLA semantic segmentation tags (latest / CityScapes-aligned)
# See: https://carla.readthedocs.io/en/latest/ref_sensors/
ROAD_TAG = 1
ROADLINE_TAG = 24
SIDEWALK_TAG = 2
GROUND_TAG = 25

# How many path points to extract from the segmentation
NUM_PATH_POINTS = 12

# Scan from 95% of image height (near) to 40% (far horizon)
SCAN_NEAR = 0.95
SCAN_FAR = 0.40

# Direction thresholds (pixel deviation from center)
TURN_THRESHOLD_PX = 40  # pixels offset to count as a turn

# BEV mini-map
BEV_SIZE = 160
BEV_RANGE = 35.0
BEV_BG = (20, 20, 30)
BEV_MARGIN = 10

# =============================================================================
# SENSOR DATA
# =============================================================================

class SensorData:
    def __init__(self):
        self.rgb_image = None
        self.seg_labels = None  # H x W uint8 label map

sensor_data = SensorData()


def process_rgb(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    sensor_data.rgb_image = array[:, :, ::-1].copy()


def process_seg(image):
    """Semantic segmentation callback. Red channel = tag ID in BGRA layout."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    sensor_data.seg_labels = array[:, :, 2].copy()  # R channel = tag


# =============================================================================
# ROAD PATH EXTRACTION FROM SEGMENTATION
# =============================================================================

# Resolved tag IDs (filled on first frame by detect_road_tags)
_road_tags = None


def detect_road_tags(seg_labels):
    """
    Auto-detect which tag IDs correspond to road in this CARLA version.

    Newer CARLA (0.9.14+): Road=1, RoadLine=24
    Older CARLA (<0.9.14):  Road=7, RoadLine=6

    We check the bottom third of the image — most of it should be road.
    """
    h = seg_labels.shape[0]
    bottom = seg_labels[int(h * 0.7):, :]
    unique, counts = np.unique(bottom, return_counts=True)
    tag_counts = dict(zip(unique.tolist(), counts.tolist()))

    # Print for debugging
    top5 = sorted(tag_counts.items(), key=lambda x: -x[1])[:5]
    print(f"[SEG] Tags in bottom 30%: {top5}")

    # Try new tags first
    new_road_count = tag_counts.get(1, 0) + tag_counts.get(24, 0)
    old_road_count = tag_counts.get(7, 0) + tag_counts.get(6, 0)

    if new_road_count > old_road_count and new_road_count > 100:
        print(f"[SEG] Using NEW tags: Road=1, RoadLine=24")
        return {1, 24}
    elif old_road_count > 100:
        print(f"[SEG] Using OLD tags: Road=7, RoadLine=6")
        return {7, 6}
    else:
        # Fallback: pick the most common tag in the bottom strip
        # (excluding 0=Unlabeled, 11=Sky)
        for tag, count in top5:
            if tag not in (0, 11, 13) and count > 100:
                print(f"[SEG] Fallback: treating tag {tag} as road "
                      f"({count} pixels)")
                return {tag}
        print(f"[SEG] WARNING: could not detect road tags!")
        return {1, 24}  # default guess


def is_road(seg_labels, road_tags):
    """Return boolean mask of road pixels."""
    mask = np.zeros(seg_labels.shape, dtype=bool)
    for tag in road_tags:
        mask |= (seg_labels == tag)
    return mask


# Road-line tags (new + old CARLA) — used for yellow line detection
ROADLINE_TAGS_SET = {24, 6}


def find_yellow_line(seg_labels, rgb_image):
    """
    Find the yellow center line column per row using seg + RGB color.

    CARLA tags ALL road markings as tag 24 (RoadLine) — no distinction.
    We use the RGB image to identify yellow pixels (center divider)
    vs white pixels (lane divider within same direction).

    Returns:
        np.ndarray of shape (H,) — yellow line column per row, -1 if none.
    """
    h, w = seg_labels.shape
    yellow_cols = np.full(h, -1, dtype=int)

    if rgb_image is None:
        return yellow_cols

    # Road line pixels (segmentation)
    line_mask = np.zeros((h, w), dtype=bool)
    for tag in ROADLINE_TAGS_SET:
        line_mask |= (seg_labels == tag)

    # Yellow color in RGB: R > 180, G > 150, B < 80
    yellow_color = (
        (rgb_image[:, :, 0] > 180) &
        (rgb_image[:, :, 1] > 150) &
        (rgb_image[:, :, 2] < 80)
    )

    # Yellow road line pixels = intersection
    yellow_line = line_mask & yellow_color

    # Per-row: median column of yellow pixels (robust to noise)
    for row in range(h):
        cols = np.where(yellow_line[row])[0]
        if len(cols) >= 3:  # need a few pixels to be confident
            yellow_cols[row] = int(np.median(cols))

    return yellow_cols


def mask_ego_side(road_mask, yellow_cols, margin=5):
    """
    Mask out opposing traffic lanes (left of yellow center line).

    In right-hand traffic, the ego vehicle is to the RIGHT of the
    yellow center line.  Pixels to the LEFT of the line are opposing
    traffic and should be excluded.

    Args:
        road_mask: boolean (H, W) road pixel mask
        yellow_cols: per-row yellow line column (-1 if not detected)
        margin: pixels past the yellow line to exclude
    Returns:
        Modified road_mask with opposing traffic removed.
    """
    masked = road_mask.copy()
    for row in range(masked.shape[0]):
        yc = yellow_cols[row]
        if yc > 0:
            masked[row, :yc + margin] = False
    return masked


# =============================================================================
# CARLA MAP-BASED CORRIDOR MASKING
# =============================================================================

def corridor_to_row_bounds(left_2d, right_2d, image_height, image_width):
    """
    Convert projected corridor boundaries to per-row column bounds.

    Projects the 3D ego-direction lane edges (from CARLA map API) into
    2D camera pixels, then interpolates to get (left_col, right_col)
    for every image row.

    Args:
        left_2d: List of Waypoint2D for left corridor edge
        right_2d: List of Waypoint2D for right corridor edge
        image_height: Image height in pixels
        image_width: Image width in pixels

    Returns:
        np.ndarray (H, 2) — [left_col, right_col] per row. -1 = no bound.
    """
    bounds = np.full((image_height, 2), -1, dtype=int)

    if not left_2d or not right_2d or len(left_2d) < 2 or len(right_2d) < 2:
        return bounds

    left_rows = np.array([p.v for p in left_2d])
    left_cols = np.array([p.u for p in left_2d])
    right_rows = np.array([p.v for p in right_2d])
    right_cols = np.array([p.u for p in right_2d])

    # Sort by row (top to bottom)
    left_order = np.argsort(left_rows)
    right_order = np.argsort(right_rows)
    left_rows, left_cols = left_rows[left_order], left_cols[left_order]
    right_rows, right_cols = right_rows[right_order], right_cols[right_order]

    min_row = max(int(min(left_rows[0], right_rows[0])), 0)
    max_row = min(int(max(left_rows[-1], right_rows[-1])), image_height - 1)

    for row in range(min_row, max_row + 1):
        lc = np.interp(row, left_rows, left_cols)
        rc = np.interp(row, right_rows, right_cols)
        # Ensure left < right regardless of projection order
        bounds[row, 0] = max(0, int(min(lc, rc)))
        bounds[row, 1] = min(image_width - 1, int(max(lc, rc)))

    return bounds


# Maximum lateral drift of path center per row step (pixels).
# Prevents the center from jumping into parking/plazas that open up
# on one side.  A normal curve shifts ~5-10 px/step; parking is 50+.
MAX_CENTER_DRIFT = 15


def extract_road_path(seg_labels, road_tags, num_points=NUM_PATH_POINTS,
                      corridor_bounds=None):
    """
    Extract a driving path from the road segmentation mask.

    Scans horizontal rows from near (bottom) to far (upper).  For each
    row, restricts to ego-direction corridor (from CARLA map API), finds
    the contiguous road segment closest to the tracked center, and
    limits how fast the center can drift.

    Args:
        seg_labels: (H, W) uint8 segmentation label map
        road_tags: set of tag IDs that count as road
        num_points: number of path sample points
        corridor_bounds: (H, 2) array of [left_col, right_col] per row
                         from corridor_to_row_bounds(), or None

    Returns:
        path:  list of (u, v) — road center in pixels
        left:  list of (u, v) — left road edge
        right: list of (u, v) — right road edge
    """
    h, w = seg_labels.shape

    row_near = int(h * SCAN_NEAR)
    row_far = int(h * SCAN_FAR)
    rows = np.linspace(row_near, row_far, num_points).astype(int)

    path, left_edge, right_edge = [], [], []
    prev_center = w / 2.0  # start at image center (ego lane)

    for row in rows:
        road_mask_row = np.zeros(w, dtype=bool)
        for tag in road_tags:
            road_mask_row |= (seg_labels[row, :] == tag)

        # Restrict to ego-direction corridor from CARLA map API
        if corridor_bounds is not None and corridor_bounds[row, 0] >= 0:
            road_mask_row[:corridor_bounds[row, 0]] = False
            road_mask_row[corridor_bounds[row, 1] + 1:] = False

        road_pixels = np.where(road_mask_row)[0]

        if len(road_pixels) < 5:
            continue

        # Find contiguous road segment closest to prev_center.
        # Gaps > 8px break segments (handles disconnected sidewalks).
        diffs = np.diff(road_pixels)
        breaks = np.where(diffs > 8)[0]
        starts = np.concatenate([[0], breaks + 1])
        ends = np.concatenate([breaks, [len(road_pixels) - 1]])

        best_l, best_r = float(road_pixels[0]), float(road_pixels[-1])
        min_dist = float('inf')
        for s, e in zip(starts, ends):
            seg_l, seg_r = float(road_pixels[s]), float(road_pixels[e])
            mid = (seg_l + seg_r) / 2.0
            dist = abs(mid - prev_center)
            if dist < min_dist:
                min_dist = dist
                best_l, best_r = seg_l, seg_r

        l, r = best_l, best_r
        c = (l + r) / 2.0

        # Constrain drift so the center can't jump into parking
        c = float(np.clip(c, prev_center - MAX_CENTER_DRIFT,
                          prev_center + MAX_CENTER_DRIFT))
        prev_center = c

        path.append((c, float(row)))
        left_edge.append((l, float(row)))
        right_edge.append((r, float(row)))

    return path, left_edge, right_edge


def path_to_waypoints_2d(path):
    """Convert pixel path [(u,v), ...] to Waypoint2D list."""
    return [Waypoint2D(u=p[0], v=p[1]) for p in path]


def detect_direction(path, image_width):
    """
    Determine turn direction from the segmentation path.

    Compares the road center at far range vs near range.
    If the far center is shifted left/right of the near center,
    we're on a curve.
    """
    if len(path) < 4:
        return "go straight"

    # Near = first few points, far = last few points
    near_u = np.mean([p[0] for p in path[:3]])
    far_u = np.mean([p[0] for p in path[-3:]])
    cx = image_width / 2.0

    # How much the far road center deviates from near road center
    shift = far_u - near_u

    if shift < -TURN_THRESHOLD_PX:
        return "turn left"
    elif shift > TURN_THRESHOLD_PX:
        return "turn right"
    return "go straight"


# =============================================================================
# DRAW HELPERS
# =============================================================================

DIR_COLORS = {
    "go straight": (50, 255, 50),
    "turn left":   (50, 150, 255),
    "turn right":  (255, 150, 50),
}


def draw_road_overlay(display, seg_labels, road_tags, corridor_bounds=None):
    """Draw semi-transparent green overlay on ego-direction road pixels only."""
    road_mask = is_road(seg_labels, road_tags)

    # Restrict to ego-direction corridor from CARLA map API
    if corridor_bounds is not None:
        h, w = road_mask.shape
        cols = np.arange(w)
        valid = (corridor_bounds[:, 0] >= 0) & (corridor_bounds[:, 1] >= 0)
        left_b = corridor_bounds[:, 0:1]   # (H, 1)
        right_b = corridor_bounds[:, 1:2]  # (H, 1)
        corridor_mask = (cols[None, :] >= left_b) & (cols[None, :] <= right_b)
        corridor_mask[~valid] = True  # keep full seg for rows without corridor
        road_mask &= corridor_mask

    overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
    overlay_array = np.zeros((WINDOW_HEIGHT, WINDOW_WIDTH, 4), dtype=np.uint8)
    overlay_array[road_mask, 1] = 180  # green channel
    overlay_array[road_mask, 3] = 50   # alpha
    pygame.surfarray.blit_array(overlay, overlay_array[:, :, :3].swapaxes(0, 1))
    overlay.set_alpha(50)
    display.blit(overlay, (0, 0))


def draw_seg_path(display, path, left_edge, right_edge, direction):
    """Draw extracted path + corridor edges on the camera view."""
    color = DIR_COLORS.get(direction, (200, 200, 200))

    # Corridor fill (between left and right edges)
    if len(left_edge) >= 2 and len(right_edge) >= 2:
        polygon = [(int(p[0]), int(p[1])) for p in left_edge] + \
                  [(int(p[0]), int(p[1])) for p in reversed(right_edge)]
        if len(polygon) >= 3:
            overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
            pygame.draw.polygon(overlay, (0, 200, 100, 45), polygon)
            display.blit(overlay, (0, 0))

    # Left/right edge lines
    if len(left_edge) >= 2:
        pts = [(int(p[0]), int(p[1])) for p in left_edge]
        pygame.draw.lines(display, (255, 255, 0), False, pts, 2)
    if len(right_edge) >= 2:
        pts = [(int(p[0]), int(p[1])) for p in right_edge]
        pygame.draw.lines(display, (255, 255, 0), False, pts, 2)

    # Center path
    if len(path) >= 2:
        pts = [(int(p[0]), int(p[1])) for p in path]
        pygame.draw.lines(display, color, False, pts, 3)
    for p in path:
        pygame.draw.circle(display, (255, 255, 255), (int(p[0]), int(p[1])), 4)
        pygame.draw.circle(display, color, (int(p[0]), int(p[1])), 2)


def draw_direction_arrow(display, direction):
    """Draw direction indicator at bottom of screen."""
    if direction == "turn left":
        symbol, color = "<< LEFT", DIR_COLORS["turn left"]
    elif direction == "turn right":
        symbol, color = "RIGHT >>", DIR_COLORS["turn right"]
    else:
        symbol, color = "STRAIGHT", DIR_COLORS["go straight"]

    big_font = pygame.font.SysFont('monospace', 28, bold=True)
    surf = big_font.render(symbol, True, color)
    x = WINDOW_WIDTH // 2 - surf.get_width() // 2
    y = WINDOW_HEIGHT - 50
    bg = pygame.Rect(x - 10, y - 5, surf.get_width() + 20, surf.get_height() + 10)
    pygame.draw.rect(display, (0, 0, 0), bg)
    pygame.draw.rect(display, color, bg, 2)
    display.blit(surf, (x, y))


NAV_TOKEN_DISPLAY = {
    "lane_keeping":          ("KEEP LANE",   (50, 255, 50)),
    "turn_left":             ("<< LEFT",     (50, 150, 255)),
    "turn_right":            ("RIGHT >>",    (255, 150, 50)),
    "u_turn":                ("U-TURN",      (255, 50, 50)),
    "merge_left":            ("< MERGE L",   (100, 200, 255)),
    "merge_right":           ("MERGE R >",   (255, 200, 100)),
    "intersection_approach": ("INTERSECTION", (255, 255, 50)),
}


def draw_nav_command(display, nav_token, route_name, route_progress):
    """Draw route navigation command at top-right of screen."""
    label, color = NAV_TOKEN_DISPLAY.get(nav_token, ("---", (180, 180, 180)))

    big_font = pygame.font.SysFont('monospace', 22, bold=True)
    small_font = pygame.font.SysFont('monospace', 13)

    # Nav token
    nav_surf = big_font.render(label, True, color)
    # Route name + progress
    prog_text = f"{route_name}  {route_progress}"
    prog_surf = small_font.render(prog_text, True, (200, 200, 200))

    box_w = max(nav_surf.get_width(), prog_surf.get_width()) + 20
    box_h = nav_surf.get_height() + prog_surf.get_height() + 14
    box_x = WINDOW_WIDTH - box_w - 10
    box_y = 8

    bg = pygame.Rect(box_x, box_y, box_w, box_h)
    pygame.draw.rect(display, (0, 0, 0), bg)
    pygame.draw.rect(display, color, bg, 2)

    display.blit(nav_surf, (box_x + (box_w - nav_surf.get_width()) // 2, box_y + 4))
    display.blit(prog_surf, (box_x + (box_w - prog_surf.get_width()) // 2,
                             box_y + nav_surf.get_height() + 8))


def draw_route_waypoints_bev(bev_surface, route_wps, vehicle_transform,
                             bev_size, bev_range):
    """Draw route waypoints on the BEV mini-map as cyan dots."""
    if not route_wps:
        return
    vx = vehicle_transform.location.x
    vy = vehicle_transform.location.y
    vyaw = math.radians(vehicle_transform.rotation.yaw)
    scale = bev_size / (2.0 * bev_range)
    cx, cy = bev_size // 2, bev_size // 2

    for wp in route_wps:
        dx, dy = wp.x - vx, wp.y - vy
        rx = dx * math.cos(-vyaw) - dy * math.sin(-vyaw)
        ry = dx * math.sin(-vyaw) + dy * math.cos(-vyaw)
        bx = max(0, min(bev_size - 1, int(cx + ry * scale)))
        by = max(0, min(bev_size - 1, int(cy - rx * scale)))
        pygame.draw.circle(bev_surface, (0, 255, 255), (bx, by), 3)


def draw_bev_minimap(display, waypoints_3d, left_3d, right_3d, vehicle_transform,
                     route_wps=None):
    """BEV mini-map showing decoded 3D road corridor + trajectory."""
    vx = vehicle_transform.location.x
    vy = vehicle_transform.location.y
    vyaw = math.radians(vehicle_transform.rotation.yaw)

    bev = pygame.Surface((BEV_SIZE, BEV_SIZE), pygame.SRCALPHA)
    bev.fill(BEV_BG)

    scale = BEV_SIZE / (2.0 * BEV_RANGE)
    cx, cy = BEV_SIZE // 2, BEV_SIZE // 2

    def w2b(wx, wy):
        dx, dy = wx - vx, wy - vy
        rx = dx * math.cos(-vyaw) - dy * math.sin(-vyaw)
        ry = dx * math.sin(-vyaw) + dy * math.cos(-vyaw)
        return (max(0, min(BEV_SIZE - 1, int(cx + ry * scale))),
                max(0, min(BEV_SIZE - 1, int(cy - rx * scale))))

    # Road corridor fill
    if left_3d and right_3d and len(left_3d) >= 2 and len(right_3d) >= 2:
        l_bev = [w2b(p.x, p.y) for p in left_3d]
        r_bev = [w2b(p.x, p.y) for p in right_3d]
        poly = l_bev + list(reversed(r_bev))
        if len(poly) >= 3:
            pygame.draw.polygon(bev, (40, 120, 60), poly)
        # Boundary lines
        pygame.draw.lines(bev, (255, 255, 0), False, l_bev, 1)
        pygame.draw.lines(bev, (255, 255, 0), False, r_bev, 1)

    # Trajectory
    if waypoints_3d and len(waypoints_3d) >= 2:
        wp_bev = [w2b(w.x, w.y) for w in waypoints_3d]
        pygame.draw.lines(bev, (255, 100, 100), False, wp_bev, 2)

    # Vehicle
    tri = [(cx, cy - 6), (cx - 4, cy + 4), (cx + 4, cy + 4)]
    pygame.draw.polygon(bev, (255, 255, 255), tri)

    # Route waypoints (cyan dots)
    if route_wps:
        draw_route_waypoints_bev(bev, route_wps, vehicle_transform,
                                 BEV_SIZE, BEV_RANGE)

    pygame.draw.rect(bev, (100, 100, 100), (0, 0, BEV_SIZE, BEV_SIZE), 1)

    bev_x = WINDOW_WIDTH - BEV_SIZE - BEV_MARGIN
    bev_y = WINDOW_HEIGHT - BEV_SIZE - BEV_MARGIN - 45
    display.blit(bev, (bev_x, bev_y))


# =============================================================================
# MAIN
# =============================================================================

def run(town, target_speed_kmh, duration, use_display,
        route_file=None, routes_dir=None):
    shutdown = False
    def on_signal(sig, frame):
        nonlocal shutdown
        print("\nShutdown requested...")
        shutdown = True
    signal.signal(signal.SIGINT, on_signal)

    # ------------------------------------------------------------------
    # Route loading
    # ------------------------------------------------------------------
    route_builder = None
    current_route = None
    route_index = 0

    if route_file:
        route_builder = RouteBuilder(route_file=route_file)
        current_route = route_builder.get_route(0)
        print(f"Loaded route: {current_route.name} "
              f"({len(current_route.checkpoints)} checkpoints, "
              f"{current_route.total_distance:.0f}m)")
    elif routes_dir:
        route_builder = RouteBuilder(routes_dir=routes_dir)
        current_route = route_builder.get_route(0)
        print(f"Starting with route: {current_route.name}")

    # If route specifies a town, extract it
    if current_route and current_route.metadata:
        meta = current_route.metadata
        if "scenarios" in meta and "custom" in meta["scenarios"]:
            custom = meta["scenarios"]["custom"]
            if custom:
                route_town = custom[0].get("town", "")
                # e.g. "Carla/Maps/Town10HD" -> "Town10HD"
                if "/" in route_town:
                    route_town = route_town.split("/")[-1]
                if route_town:
                    town = route_town
                    print(f"Route specifies town: {town}")

    # Display
    display = clock = font = None
    if use_display and PYGAME_AVAILABLE:
        pygame.init()
        display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Segmentation Driver" +
                                   (f" - {current_route.name}" if current_route else ""))
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 14)

    # CARLA
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world(town)
    bp_lib = world.get_blueprint_library()

    actors = []
    start_time = time.time()

    # CARLA map API for lane-level navigation
    carla_map = world.get_map()
    wp_generator = WaypointGenerator(carla_map)

    # Components
    encoder = TrajectoryEncoder(CAMERA_CONFIG)
    controller = VehiclePIDController(
        target_speed_kmh=target_speed_kmh,
        lateral_kp=0.9,
        lateral_kd=0.15,
        lookahead_distance=15.0,
    )

    try:
        # Spawn vehicle — use route start if available
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        if current_route:
            spawn_info = route_builder.get_spawn_transform(current_route)
            spawn_tf = carla.Transform(
                carla.Location(
                    x=spawn_info["location"]["x"],
                    y=spawn_info["location"]["y"],
                    z=spawn_info["location"]["z"],
                ),
                carla.Rotation(yaw=spawn_info["rotation"]["yaw"]),
            )
        else:
            spawn_points = world.get_map().get_spawn_points()
            spawn_tf = spawn_points[0]

        vehicle = world.spawn_actor(vehicle_bp, spawn_tf)
        actors.append(vehicle)
        print(f"Spawned at ({spawn_tf.location.x:.1f}, {spawn_tf.location.y:.1f})")

        cam_transform = carla.Transform(
            carla.Location(x=CAMERA_CONFIG.x, y=CAMERA_CONFIG.y, z=CAMERA_CONFIG.z),
            carla.Rotation(pitch=CAMERA_CONFIG.pitch)
        )

        # RGB camera (for display)
        rgb_bp = bp_lib.find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', str(WINDOW_WIDTH))
        rgb_bp.set_attribute('image_size_y', str(WINDOW_HEIGHT))
        rgb_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))
        rgb_cam = world.spawn_actor(rgb_bp, cam_transform, attach_to=vehicle)
        actors.append(rgb_cam)
        rgb_cam.listen(process_rgb)

        # Semantic segmentation camera (for road detection)
        seg_bp = bp_lib.find('sensor.camera.semantic_segmentation')
        seg_bp.set_attribute('image_size_x', str(WINDOW_WIDTH))
        seg_bp.set_attribute('image_size_y', str(WINDOW_HEIGHT))
        seg_bp.set_attribute('fov', str(CAMERA_CONFIG.fov))
        seg_cam = world.spawn_actor(seg_bp, cam_transform, attach_to=vehicle)
        actors.append(seg_cam)
        seg_cam.listen(process_seg)

        time.sleep(1.0)
        mode_str = "MAP + ROUTE" if current_route else "MAP API"
        print(f"\nDriving at {target_speed_kmh} km/h from {mode_str}")
        print(f"Press Ctrl+C or ESC to stop\n")

        frame = 0
        last_direction = "go straight"
        road_tags = None  # auto-detected on first seg frame
        nav_token = "lane_keeping"
        nav_candidate = "lane_keeping"  # candidate waiting for stability
        nav_stable_count = 0            # frames the candidate has persisted
        NAV_STABLE_THRESHOLD = 5        # frames needed to accept a change
        route_wps = []     # route waypoints for BEV display
        routes_completed = 0

        while not shutdown:
            elapsed = time.time() - start_time
            if duration and elapsed > duration:
                print(f"\nReached duration limit ({duration}s)")
                break

            if display:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        shutdown = True
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        shutdown = True

            # Vehicle state
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(
                velocity.x**2 + velocity.y**2 + velocity.z**2)

            # ============================================================
            # PERCEPTION: CARLA map API for lane waypoints + seg overlay
            # ============================================================
            path_px = []
            left_px = []
            right_px = []
            direction = "go straight"

            # --- CARLA map API: ego-direction corridor + lane waypoints ---
            corridor = wp_generator.get_drivable_corridor(
                vehicle, max_distance=35.0, sample_step=2.0)
            lane_wps = wp_generator.get_road_waypoints(
                vehicle, max_distance=30.0, num_output=10)

            # Project corridor boundaries to 2D for overlay masking
            corridor_bounds = None
            corr_left = corridor.adj_left or corridor.left
            corr_right = corridor.adj_right or corridor.right
            if corr_left and corr_right:
                left_2d = encoder.encode(corr_left, transform)
                right_2d = encoder.encode(corr_right, transform)
                corridor_bounds = corridor_to_row_bounds(
                    left_2d, right_2d, WINDOW_HEIGHT, WINDOW_WIDTH)

            # --- Segmentation: 2D path for visualisation only ---
            if sensor_data.seg_labels is not None:
                # Auto-detect road tags on first frame
                if road_tags is None:
                    road_tags = detect_road_tags(sensor_data.seg_labels)

                # Extract 2D path restricted to ego-direction corridor
                path_px, left_px, right_px = extract_road_path(
                    sensor_data.seg_labels, road_tags,
                    corridor_bounds=corridor_bounds)

                if path_px:
                    direction = detect_direction(path_px, WINDOW_WIDTH)

            # Log direction changes
            if direction != last_direction:
                print(f"[SEG] {last_direction} -> {direction}")
                last_direction = direction

            # ============================================================
            # ROUTE NAVIGATION + STEERING WAYPOINTS
            # ============================================================
            route_wps = []
            steer_source = "seg"
            if current_route and route_builder:
                # Get route waypoints at distances ahead (for both nav + steering)
                route_wps = route_builder.get_waypoints_at_distances(
                    current_route, transform.location,
                    distances=[3, 6, 9, 12, 15, 20, 25, 30],
                )

                # Navigation token from route waypoints (with stability filter)
                wp_list = [[wp.x, wp.y, wp.z] for wp in route_wps]
                raw_nav = generate_navigation_token(wp_list)

                # Require token to be stable for N frames before accepting
                if raw_nav == nav_candidate:
                    nav_stable_count += 1
                else:
                    nav_candidate = raw_nav
                    nav_stable_count = 1

                if nav_stable_count >= NAV_STABLE_THRESHOLD and nav_token != nav_candidate:
                    print(f"[ROUTE] {nav_token} -> {nav_candidate}")
                    nav_token = nav_candidate

                # Check route completion
                if route_builder.is_route_complete(
                        current_route, transform.location, threshold_meters=5.0):
                    routes_completed += 1
                    print(f"\n[ROUTE] Completed: {current_route.name} "
                          f"({routes_completed} total)")

                    # Advance to next route if available
                    if route_builder.get_route_count() > 1:
                        route_index = (route_index + 1) % route_builder.get_route_count()
                        current_route = route_builder.get_route(route_index)
                        print(f"[ROUTE] Next: {current_route.name}")
                        controller.reset()
                    else:
                        print(f"[ROUTE] All routes done! Free driving...")
                        current_route = None

            # ============================================================
            # CONTROL: PID steering
            # Priority: route waypoints > map API lane wps > stop
            # Map API gives reliable 3D lane waypoints directly —
            # no seg-to-3D error amplification.
            # ============================================================
            if route_wps:
                control = controller.compute_control(
                    route_wps, transform, speed_kmh)
                steer_source = "route"
            elif lane_wps:
                control = controller.compute_control(
                    lane_wps, transform, speed_kmh)
                steer_source = "map"
            else:
                # No waypoints at all — stop
                control = controller.compute_control(
                    [], transform, speed_kmh)
                steer_source = "none"
            vehicle.apply_control(control)

            # ============================================================
            # RENDER
            # ============================================================
            if display and sensor_data.rgb_image is not None:
                surface = pygame.surfarray.make_surface(
                    sensor_data.rgb_image.swapaxes(0, 1))
                display.blit(surface, (0, 0))

                # Road overlay (green tint on ego-direction road pixels only)
                if sensor_data.seg_labels is not None and road_tags:
                    draw_road_overlay(display, sensor_data.seg_labels,
                                      road_tags,
                                      corridor_bounds=corridor_bounds)

                # Extracted path + corridor edges
                if path_px:
                    draw_seg_path(display, path_px, left_px,
                                  right_px, direction)

                # BEV mini-map — corridor from map API, trajectory from steering source
                bev_traj = route_wps if route_wps else lane_wps
                bev_left = corr_left if corr_left else []
                bev_right = corr_right if corr_right else []
                draw_bev_minimap(display, bev_traj, bev_left,
                                 bev_right, transform,
                                 route_wps=route_wps if steer_source == "map" else None)

                # Direction arrow (seg-based)
                draw_direction_arrow(display, direction)

                # Route navigation command (top-right)
                if current_route:
                    closest_idx = route_builder.find_closest_checkpoint(
                        current_route, transform.location)
                    total_cps = len(current_route.checkpoints)
                    pct = int(100 * closest_idx / max(total_cps - 1, 1))
                    draw_nav_command(display, nav_token, current_route.name,
                                    f"{pct}%  [{closest_idx}/{total_cps}]")

                # Info overlay
                info_lines = [
                    f"Speed: {speed_kmh:.1f} km/h  Ctrl: {steer_source.upper()}",
                    f"Steer: {control.steer:+.2f}  Throttle: {control.throttle:.2f}",
                    f"Seg: {direction.upper()}",
                    f"Nav: {nav_token}" if current_route else "",
                    f"Seg pts: {len(path_px)}  Lane wps: {len(lane_wps)}  Route wps: {len(route_wps)}",
                    f"Elapsed: {int(elapsed)}s",
                ]
                info_lines = [l for l in info_lines if l]  # remove empty
                y = 8
                for line in info_lines:
                    text = font.render(line, True, (255, 255, 255))
                    bg_rect = pygame.Rect(5, y - 2, text.get_width() + 10,
                                          text.get_height() + 4)
                    pygame.draw.rect(display, (0, 0, 0), bg_rect)
                    display.blit(text, (10, y))
                    y += 18

                pygame.display.flip()
                clock.tick(30)
            else:
                world.tick()
                time.sleep(0.033)

            if frame % 30 == 0:
                nav_str = f" | Nav: {nav_token:<12s}" if current_route else ""
                print(f"\r  Speed: {speed_kmh:5.1f} km/h | "
                      f"Steer: {control.steer:+.2f} [{steer_source}] | "
                      f"Seg: {direction:<12s}"
                      f"{nav_str} | "
                      f"Elapsed: {int(elapsed)}s", end='')
            frame += 1

    finally:
        print("\n\nCleaning up...")
        for actor in actors:
            actor.destroy()
        if display:
            pygame.quit()
        elapsed = time.time() - start_time
        print(f"Drove for {elapsed:.0f}s")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Segmentation-based autonomous driver with route navigation')
    parser.add_argument('--town', default='Town01',
                        help='CARLA town (overridden by route if specified)')
    parser.add_argument('--speed', type=float, default=25.0,
                        help='Target speed km/h')
    parser.add_argument('--duration', type=float,
                        help='Max duration in seconds')
    parser.add_argument('--no-display', action='store_true',
                        help='Headless mode')
    parser.add_argument('--route', type=str, default=None,
                        help='Single route JSON file (e.g. routes/left_1.json)')
    parser.add_argument('--routes-dir', type=str, default=None,
                        help='Directory of route JSON files (loads all)')
    args = parser.parse_args()

    run(town=args.town,
        target_speed_kmh=args.speed,
        duration=args.duration,
        use_display=not args.no_display,
        route_file=args.route,
        routes_dir=args.routes_dir)
