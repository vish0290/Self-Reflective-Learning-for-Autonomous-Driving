#!/usr/bin/env python3
"""
CARLA Scenario Route Finder and Dataset Generator

Analyzes CARLA map topology to find specific route types:
- Lane Maintenance (straight roads)
- Right Curve Turn
- Left Curve Turn
- Intersection - Go Straight
- Intersection - Turn Left
- Intersection - Turn Right

Saves routes as checkpoints in JSON format for data collection.
"""

import carla
import pygame
import numpy as np
import math
import json
import os
import sys
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional, Tuple
from enum import Enum
from datetime import datetime

# Try to import CARLA's Global Route Planner
try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    HAS_GLOBAL_PLANNER = True
    print("GlobalRoutePlanner available")
except ImportError:
    HAS_GLOBAL_PLANNER = False
    print("GlobalRoutePlanner not available - using fallback routing")


# =============================================================================
# ENUMS AND DATA CLASSES
# =============================================================================

class RouteType(Enum):
    LANE_MAINTENANCE = "lane_maintenance"
    RIGHT_CURVE = "right_curve"
    LEFT_CURVE = "left_curve"
    INTERSECTION_STRAIGHT = "intersection_straight"
    INTERSECTION_LEFT = "intersection_left"
    INTERSECTION_RIGHT = "intersection_right"


@dataclass
class Checkpoint:
    """A single checkpoint (waypoint) in a route."""
    x: float
    y: float
    z: float
    yaw: float
    road_id: int
    lane_id: int

    @classmethod
    def from_waypoint(cls, wp) -> 'Checkpoint':
        loc = wp.transform.location
        rot = wp.transform.rotation
        return cls(
            x=loc.x, y=loc.y, z=loc.z,
            yaw=rot.yaw,
            road_id=wp.road_id,
            lane_id=wp.lane_id
        )

    def to_carla_location(self) -> carla.Location:
        return carla.Location(x=self.x, y=self.y, z=self.z)

    def to_carla_transform(self) -> carla.Transform:
        return carla.Transform(
            carla.Location(x=self.x, y=self.y, z=self.z),
            carla.Rotation(yaw=self.yaw)
        )


@dataclass
class ScenarioRoute:
    """A complete route scenario with metadata."""
    id: str
    route_type: str
    town: str
    description: str
    checkpoints: List[Dict]
    total_distance: float
    avg_curvature: float
    has_junction: bool
    created_at: str

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> 'ScenarioRoute':
        return cls(**data)


# =============================================================================
# CARLA ROUTE PLANNER WRAPPER
# =============================================================================

class RoutePlanner:
    """
    Wrapper for CARLA's GlobalRoutePlanner.
    Computes optimal routes between two points using road network topology.
    """

    def __init__(self, carla_map, sampling_resolution: float = 1.0):
        self.map = carla_map
        self.sampling_resolution = sampling_resolution
        self._planner = None

        if HAS_GLOBAL_PLANNER:
            self._planner = GlobalRoutePlanner(self.map, self.sampling_resolution)
            print(f"GlobalRoutePlanner initialized with {sampling_resolution}m resolution")

    def compute_route(self, start_location: carla.Location, end_location: carla.Location) -> List:
        """
        Compute route between two locations.

        Returns:
            List of (waypoint, road_option) tuples
        """
        if self._planner is None:
            return self._fallback_route(start_location, end_location)

        try:
            route = self._planner.trace_route(start_location, end_location)
            return route
        except Exception as e:
            print(f"GlobalRoutePlanner failed: {e}, using fallback")
            return self._fallback_route(start_location, end_location)

    def compute_route_waypoints(self, start_location: carla.Location,
                                 end_location: carla.Location) -> List:
        """
        Compute route and return just the waypoints (not road options).
        """
        route = self.compute_route(start_location, end_location)
        return [wp for wp, _ in route]

    def _fallback_route(self, start_location: carla.Location,
                        end_location: carla.Location) -> List:
        """
        Fallback routing when GlobalRoutePlanner is not available.
        Uses greedy waypoint following toward destination.
        """
        waypoints = []
        current_wp = self.map.get_waypoint(start_location)

        if current_wp is None:
            return waypoints

        waypoints.append((current_wp, None))
        max_iterations = 2000

        for _ in range(max_iterations):
            next_wps = current_wp.next(self.sampling_resolution)
            if not next_wps:
                break

            # Choose waypoint closest to destination
            best_wp = min(next_wps,
                         key=lambda w: w.transform.location.distance(end_location))

            waypoints.append((best_wp, None))
            current_wp = best_wp

            # Check if arrived
            if current_wp.transform.location.distance(end_location) < 5.0:
                break

        return waypoints

    def get_route_info(self, route: List) -> Dict:
        """
        Analyze a route and return information about it.
        """
        if not route:
            return {'distance': 0, 'has_junction': False, 'turn_count': 0}

        total_distance = 0.0
        has_junction = False
        turn_count = 0

        for i in range(len(route) - 1):
            wp1, opt1 = route[i]
            wp2, opt2 = route[i + 1]

            total_distance += wp1.transform.location.distance(wp2.transform.location)

            if wp1.is_junction or wp2.is_junction:
                has_junction = True

            # Count turns based on road options (if available)
            if opt2 is not None:
                opt_name = str(opt2).lower()
                if 'left' in opt_name or 'right' in opt_name:
                    turn_count += 1

        return {
            'distance': total_distance,
            'has_junction': has_junction,
            'turn_count': turn_count
        }


# =============================================================================
# SCENARIO FINDER
# =============================================================================

class ScenarioFinder:
    """
    Analyzes CARLA map to find specific route scenarios.
    Uses CARLA's GlobalRoutePlanner for optimal pathfinding.
    """

    # Waypoint step size in meters - smaller = denser, smoother routes
    WAYPOINT_STEP = 1.0

    def __init__(self, world, carla_map):
        self.world = world
        self.map = carla_map
        self.town_name = carla_map.name

        # Initialize route planner
        self.route_planner = RoutePlanner(carla_map, sampling_resolution=self.WAYPOINT_STEP)

    def find_all_scenarios(self, min_length: float = 50.0, max_length: float = 200.0) -> Dict[str, List[ScenarioRoute]]:
        """
        Find all types of scenarios in the current map.

        Args:
            min_length: Minimum route length in meters
            max_length: Maximum route length in meters

        Returns:
            Dictionary with route types as keys and lists of ScenarioRoutes as values
        """
        scenarios = {
            RouteType.LANE_MAINTENANCE.value: [],
            RouteType.RIGHT_CURVE.value: [],
            RouteType.LEFT_CURVE.value: [],
            RouteType.INTERSECTION_STRAIGHT.value: [],
            RouteType.INTERSECTION_LEFT.value: [],
            RouteType.INTERSECTION_RIGHT.value: [],
        }

        print(f"Analyzing map: {self.town_name}")

        print("Finding curve routes (including lane maintenance)...")
        lane_maint, left_curves, right_curves = self._find_all_road_segments(min_length, max_length)
        scenarios[RouteType.LANE_MAINTENANCE.value] = lane_maint
        scenarios[RouteType.LEFT_CURVE.value] = left_curves
        scenarios[RouteType.RIGHT_CURVE.value] = right_curves

        print("Finding intersection routes...")
        straight, left, right = self._find_intersections(min_length, max_length)
        scenarios[RouteType.INTERSECTION_STRAIGHT.value] = straight
        scenarios[RouteType.INTERSECTION_LEFT.value] = left
        scenarios[RouteType.INTERSECTION_RIGHT.value] = right

        # Print summary
        print("\n=== Scenario Summary ===")
        for route_type, routes in scenarios.items():
            print(f"  {route_type}: {len(routes)} routes found")

        return scenarios

    def _find_all_road_segments(self, min_length: float, max_length: float) -> Tuple[List[ScenarioRoute], List[ScenarioRoute], List[ScenarioRoute]]:
        """
        Find all road segments and categorize them as:
        - Lane maintenance (straight, curvature < 0.5 deg/m)
        - Left curve (total turn > 30 deg left)
        - Right curve (total turn > 30 deg right)
        """
        lane_maintenance = []
        left_curves = []
        right_curves = []

        # Get sparse waypoints to use as starting points
        start_waypoints = self.map.generate_waypoints(10.0)
        driving_waypoints = [wp for wp in start_waypoints if wp.lane_type == carla.LaneType.Driving]

        print(f"  Analyzing {len(driving_waypoints)} starting points...")

        visited = set()

        for start_wp in driving_waypoints:
            # Skip if we've already processed this road segment
            key = (start_wp.road_id, start_wp.lane_id, int(start_wp.s / 30))
            if key in visited:
                continue

            # Skip junctions
            if start_wp.is_junction:
                continue

            # Build dense route forward
            route_wps = self._build_dense_route(start_wp, max_length)

            if len(route_wps) < 20:  # Need enough waypoints
                continue

            # Check for junctions in route
            has_junction = any(wp.is_junction for wp in route_wps)
            if has_junction:
                continue

            total_dist = self._calculate_route_distance(route_wps)
            if total_dist < min_length:
                continue

            visited.add(key)

            # Calculate total yaw change and curvature
            total_yaw_change, avg_curvature = self._analyze_route_curvature(route_wps)

            # Categorize based on total turn angle
            abs_turn = abs(total_yaw_change)

            if abs_turn < 15:  # Less than 15 degrees total = straight
                if len(lane_maintenance) < 20:
                    route = self._create_scenario_route(
                        route_wps,
                        RouteType.LANE_MAINTENANCE,
                        f"Straight segment on road {start_wp.road_id}",
                        total_dist, avg_curvature, False
                    )
                    lane_maintenance.append(route)

            elif total_yaw_change > 25:  # Left turn > 25 degrees
                if len(left_curves) < 20:
                    route = self._create_scenario_route(
                        route_wps,
                        RouteType.LEFT_CURVE,
                        f"Left curve ({total_yaw_change:.0f}°) on road {start_wp.road_id}",
                        total_dist, avg_curvature, False
                    )
                    left_curves.append(route)

            elif total_yaw_change < -25:  # Right turn > 25 degrees
                if len(right_curves) < 20:
                    route = self._create_scenario_route(
                        route_wps,
                        RouteType.RIGHT_CURVE,
                        f"Right curve ({abs(total_yaw_change):.0f}°) on road {start_wp.road_id}",
                        total_dist, avg_curvature, False
                    )
                    right_curves.append(route)

            # Stop if we have enough of each
            if len(lane_maintenance) >= 20 and len(left_curves) >= 20 and len(right_curves) >= 20:
                break

        return lane_maintenance, left_curves, right_curves

    def _find_intersections(self, min_length: float, max_length: float) -> Tuple[List[ScenarioRoute], List[ScenarioRoute], List[ScenarioRoute]]:
        """
        Find intersection routes using GlobalRoutePlanner.
        Routes go: approach road -> through junction -> exit road
        """
        straight_routes = []
        left_routes = []
        right_routes = []

        # Find all junctions
        topology = self.map.get_topology()
        junction_waypoints = set()

        for wp_start, wp_end in topology:
            if wp_start.is_junction:
                junction_waypoints.add(wp_start)
            if wp_end.is_junction:
                junction_waypoints.add(wp_end)

        # Get unique junctions
        junctions = {}
        for wp in junction_waypoints:
            junc = wp.get_junction()
            if junc:
                junctions[junc.id] = junc

        print(f"  Found {len(junctions)} junctions")

        for junc_id, junction in junctions.items():
            # Get junction waypoints (entry -> exit pairs)
            junc_wps = junction.get_waypoints(carla.LaneType.Driving)

            for entry_wp, exit_wp in junc_wps:
                # Get approach point (before junction)
                approach_wps = entry_wp.previous(min_length / 2)
                if not approach_wps:
                    continue
                start_wp = approach_wps[-1] if len(approach_wps) > 1 else approach_wps[0]

                # Get destination point (after junction)
                exit_next_wps = exit_wp.next(min_length / 2)
                if not exit_next_wps:
                    continue
                end_wp = exit_next_wps[-1] if len(exit_next_wps) > 1 else exit_next_wps[0]

                # Use GlobalRoutePlanner to compute the full route
                start_loc = start_wp.transform.location
                end_loc = end_wp.transform.location

                route_with_options = self.route_planner.compute_route(start_loc, end_loc)

                if len(route_with_options) < 20:
                    continue

                # Extract waypoints
                full_route = [wp for wp, _ in route_with_options]

                # Determine turn direction based on entry/exit yaw
                entry_yaw = entry_wp.transform.rotation.yaw
                exit_yaw = exit_wp.transform.rotation.yaw

                yaw_diff = self._angle_diff(exit_yaw, entry_yaw)

                total_dist = self._calculate_route_distance(full_route)
                _, avg_curv = self._analyze_route_curvature(full_route)

                if abs(yaw_diff) < 30:  # Straight through
                    if len(straight_routes) < 15:
                        route = self._create_scenario_route(
                            full_route,
                            RouteType.INTERSECTION_STRAIGHT,
                            f"Intersection straight at junction {junc_id}",
                            total_dist, avg_curv, True
                        )
                        straight_routes.append(route)

                elif yaw_diff > 45:  # Left turn
                    if len(left_routes) < 15:
                        route = self._create_scenario_route(
                            full_route,
                            RouteType.INTERSECTION_LEFT,
                            f"Intersection left turn ({yaw_diff:.0f}°) at junction {junc_id}",
                            total_dist, avg_curv, True
                        )
                        left_routes.append(route)

                elif yaw_diff < -45:  # Right turn
                    if len(right_routes) < 15:
                        route = self._create_scenario_route(
                            full_route,
                            RouteType.INTERSECTION_RIGHT,
                            f"Intersection right turn ({abs(yaw_diff):.0f}°) at junction {junc_id}",
                            total_dist, avg_curv, True
                        )
                        right_routes.append(route)

            # Stop early if we have enough
            if len(straight_routes) >= 15 and len(left_routes) >= 15 and len(right_routes) >= 15:
                break

        return straight_routes, left_routes, right_routes

    def _build_dense_route(self, start_wp, max_length: float) -> List:
        """
        Build a route with dense waypoints (1m spacing) going forward.
        This ensures smooth, continuous paths without gaps.
        """
        waypoints = [start_wp]
        current = start_wp
        distance = 0.0

        while distance < max_length:
            # Get next waypoint at small step
            next_wps = current.next(self.WAYPOINT_STEP)
            if not next_wps:
                break

            # Choose the best next waypoint
            if len(next_wps) == 1:
                best_wp = next_wps[0]
            else:
                # Multiple options - choose the one that continues most smoothly
                current_yaw = current.transform.rotation.yaw
                best_wp = min(next_wps, key=lambda w: abs(self._angle_diff(w.transform.rotation.yaw, current_yaw)))

            # Verify waypoint is valid and connected
            step_dist = current.transform.location.distance(best_wp.transform.location)

            # Skip if gap is too large (indicates discontinuity)
            if step_dist > self.WAYPOINT_STEP * 3:
                break

            waypoints.append(best_wp)
            distance += step_dist
            current = best_wp

        return waypoints

    def _build_dense_route_backward(self, start_wp, max_length: float) -> List:
        """Build a dense route going backward from start waypoint."""
        waypoints = [start_wp]
        current = start_wp
        distance = 0.0

        while distance < max_length:
            prev_wps = current.previous(self.WAYPOINT_STEP)
            if not prev_wps:
                break

            # Choose best previous waypoint
            if len(prev_wps) == 1:
                best_wp = prev_wps[0]
            else:
                current_yaw = current.transform.rotation.yaw
                best_wp = min(prev_wps, key=lambda w: abs(self._angle_diff(w.transform.rotation.yaw, current_yaw)))

            step_dist = current.transform.location.distance(best_wp.transform.location)

            # Skip if gap is too large
            if step_dist > self.WAYPOINT_STEP * 3:
                break

            waypoints.append(best_wp)
            distance += step_dist
            current = best_wp

        return waypoints

    def _build_junction_path(self, entry_wp, exit_wp) -> List:
        """
        Build waypoints through a junction by interpolating between entry and exit.
        """
        waypoints = []

        # Get locations
        entry_loc = entry_wp.transform.location
        exit_loc = exit_wp.transform.location

        # Calculate distance through junction
        junction_dist = entry_loc.distance(exit_loc)

        if junction_dist < 2:
            return waypoints

        # Number of interpolation points
        num_points = max(3, int(junction_dist / self.WAYPOINT_STEP))

        # Interpolate through junction
        for i in range(1, num_points):
            t = i / num_points
            interp_loc = carla.Location(
                x=entry_loc.x + t * (exit_loc.x - entry_loc.x),
                y=entry_loc.y + t * (exit_loc.y - entry_loc.y),
                z=entry_loc.z + t * (exit_loc.z - entry_loc.z)
            )

            # Get the actual road waypoint at this location
            wp = self.map.get_waypoint(interp_loc)
            if wp:
                waypoints.append(wp)

        return waypoints

    def _calculate_route_distance(self, waypoints: List) -> float:
        """Calculate total distance of a route."""
        if len(waypoints) < 2:
            return 0.0

        total = 0.0
        for i in range(len(waypoints) - 1):
            loc1 = waypoints[i].transform.location
            loc2 = waypoints[i + 1].transform.location
            total += loc1.distance(loc2)
        return total

    def _analyze_route_curvature(self, waypoints: List) -> Tuple[float, float]:
        """
        Analyze route curvature.
        Returns (total_yaw_change, avg_curvature_per_meter)
        """
        if len(waypoints) < 3:
            return 0.0, 0.0

        total_yaw_change = 0.0
        total_distance = 0.0

        for i in range(len(waypoints) - 1):
            yaw1 = waypoints[i].transform.rotation.yaw
            yaw2 = waypoints[i + 1].transform.rotation.yaw

            yaw_diff = self._angle_diff(yaw2, yaw1)
            dist = waypoints[i].transform.location.distance(waypoints[i + 1].transform.location)

            total_yaw_change += yaw_diff
            total_distance += dist

        if total_distance == 0:
            return 0.0, 0.0

        avg_curvature = abs(total_yaw_change) / total_distance

        return total_yaw_change, avg_curvature

    def _angle_diff(self, angle1: float, angle2: float) -> float:
        """Calculate angle difference, normalized to [-180, 180]."""
        diff = angle1 - angle2
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def _create_scenario_route(self, waypoints: List, route_type: RouteType,
                               description: str, total_dist: float,
                               avg_curvature: float, has_junction: bool) -> ScenarioRoute:
        """Create a ScenarioRoute from waypoints."""
        checkpoints = [asdict(Checkpoint.from_waypoint(wp)) for wp in waypoints]

        route_id = f"{route_type.value}_{self.town_name}_{len(checkpoints)}_{int(datetime.now().timestamp())}"

        return ScenarioRoute(
            id=route_id,
            route_type=route_type.value,
            town=self.town_name,
            description=description,
            checkpoints=checkpoints,
            total_distance=total_dist,
            avg_curvature=avg_curvature,
            has_junction=has_junction,
            created_at=datetime.now().isoformat()
        )


# =============================================================================
# CHECKPOINT MANAGER
# =============================================================================

class CheckpointManager:
    """Handles saving and loading route checkpoints."""

    def __init__(self, save_dir: str = "route_checkpoints"):
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

    def save_scenarios(self, scenarios: Dict[str, List[ScenarioRoute]], filename: str = None):
        """Save all scenarios to a JSON file."""
        if filename is None:
            filename = f"scenarios_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

        filepath = os.path.join(self.save_dir, filename)

        data = {
            "created_at": datetime.now().isoformat(),
            "scenarios": {}
        }

        for route_type, routes in scenarios.items():
            data["scenarios"][route_type] = [r.to_dict() for r in routes]

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"Saved scenarios to: {filepath}")
        return filepath

    def load_scenarios(self, filename: str) -> Dict[str, List[ScenarioRoute]]:
        """Load scenarios from a JSON file."""
        filepath = os.path.join(self.save_dir, filename) if not os.path.isabs(filename) else filename

        with open(filepath, 'r') as f:
            data = json.load(f)

        scenarios = {}
        for route_type, routes_data in data["scenarios"].items():
            scenarios[route_type] = [ScenarioRoute.from_dict(r) for r in routes_data]

        return scenarios

    def list_saved_files(self) -> List[str]:
        """List all saved scenario files."""
        if not os.path.exists(self.save_dir):
            return []
        return [f for f in os.listdir(self.save_dir) if f.endswith('.json')]

    def get_routes_by_type(self, scenarios: Dict[str, List[ScenarioRoute]],
                          route_type: RouteType) -> List[ScenarioRoute]:
        """Get all routes of a specific type."""
        return scenarios.get(route_type.value, [])


# =============================================================================
# PYGAME UI FOR ROUTE BROWSER
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


ROUTE_TYPE_COLORS = {
    RouteType.LANE_MAINTENANCE.value: Colors.GREEN,
    RouteType.RIGHT_CURVE.value: Colors.ORANGE,
    RouteType.LEFT_CURVE.value: Colors.CYAN,
    RouteType.INTERSECTION_STRAIGHT.value: Colors.YELLOW,
    RouteType.INTERSECTION_LEFT.value: Colors.PURPLE,
    RouteType.INTERSECTION_RIGHT.value: Colors.RED,
}


# =============================================================================
# LANE FOLLOWING CONTROLLER (for test driving)
# =============================================================================

class LaneFollowingController:
    """
    Improved lane following controller for test driving routes.
    Uses weighted average of multiple ahead waypoints for smoother steering.
    """

    def __init__(self, target_speed_kmh: float = 30.0):
        self.target_speed_kmh = target_speed_kmh

    def compute_control(self, checkpoints: List[Dict], current_idx: int,
                        vehicle_transform, current_speed_kmh: float):
        """Compute vehicle control to follow checkpoints."""
        control = carla.VehicleControl()

        if not checkpoints or current_idx >= len(checkpoints):
            control.brake = 1.0
            return control, current_idx

        v_loc = vehicle_transform.location
        v_yaw = vehicle_transform.rotation.yaw

        # Find next waypoint
        new_idx = self._find_next_checkpoint(checkpoints, current_idx, v_loc)

        if new_idx >= len(checkpoints) - 1:
            control.brake = 1.0
            return control, new_idx

        # Compute steering using weighted lookahead
        steer = self._compute_steering_weighted(v_loc, v_yaw, checkpoints, new_idx, current_speed_kmh)

        # Compute throttle/brake
        throttle, brake = self._compute_speed_control(current_speed_kmh, abs(steer))

        control.throttle = float(np.clip(throttle, 0.0, 1.0))
        control.brake = float(np.clip(brake, 0.0, 1.0))
        control.steer = float(np.clip(steer, -1.0, 1.0))

        return control, new_idx

    def _find_next_checkpoint(self, checkpoints: List[Dict], start_idx: int, vehicle_loc) -> int:
        """Find next checkpoint to target."""
        min_dist = float('inf')
        closest_idx = start_idx

        # Search around current index
        search_start = max(0, start_idx - 3)
        search_end = min(len(checkpoints), start_idx + 50)

        for i in range(search_start, search_end):
            cp = checkpoints[i]
            cp_loc = carla.Location(x=cp['x'], y=cp['y'], z=cp['z'])
            dist = vehicle_loc.distance(cp_loc)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Move past checkpoints we've already passed (within 1.5m)
        while closest_idx < len(checkpoints) - 1:
            cp = checkpoints[closest_idx]
            cp_loc = carla.Location(x=cp['x'], y=cp['y'], z=cp['z'])
            dist = vehicle_loc.distance(cp_loc)

            if dist < 1.5:
                closest_idx += 1
            else:
                break

        return closest_idx

    def _compute_steering_weighted(self, vehicle_loc, vehicle_yaw: float,
                                   checkpoints: List[Dict], current_idx: int,
                                   current_speed_kmh: float) -> float:
        """
        Compute steering using weighted average of multiple lookahead points.
        This provides much smoother steering than single-point targeting.
        """
        # Adaptive lookahead based on speed
        base_lookahead = max(5, int(current_speed_kmh / 5))
        max_lookahead = min(base_lookahead + 10, len(checkpoints) - current_idx - 1)

        if max_lookahead < 3:
            # Fall back to simple steering if not enough waypoints
            target_cp = checkpoints[min(current_idx + 3, len(checkpoints) - 1)]
            return self._compute_steering_simple(vehicle_loc, vehicle_yaw, target_cp)

        # Compute weighted steering from multiple points
        total_weight = 0.0
        weighted_steer = 0.0

        for i in range(3, max_lookahead + 1):
            idx = current_idx + i
            if idx >= len(checkpoints):
                break

            cp = checkpoints[idx]

            # Weight closer points more heavily
            weight = 1.0 / (i * 0.5)

            steer = self._compute_steering_simple(vehicle_loc, vehicle_yaw, cp)
            weighted_steer += steer * weight
            total_weight += weight

        if total_weight > 0:
            return weighted_steer / total_weight
        else:
            target_cp = checkpoints[min(current_idx + 5, len(checkpoints) - 1)]
            return self._compute_steering_simple(vehicle_loc, vehicle_yaw, target_cp)

    def _compute_steering_simple(self, vehicle_loc, vehicle_yaw: float, target_cp: Dict) -> float:
        """Compute steering angle to a single target point."""
        dx = target_cp['x'] - vehicle_loc.x
        dy = target_cp['y'] - vehicle_loc.y

        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = target_angle - vehicle_yaw

        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        # 45 degrees = full steering
        steer = angle_diff / 45.0
        return float(np.clip(steer, -1.0, 1.0))

    def _compute_speed_control(self, current_speed_kmh: float, steer_magnitude: float):
        """Compute throttle and brake with turn-based speed reduction."""
        target_speed = self.target_speed_kmh

        # Slow down in turns - more aggressive reduction for sharper turns
        if steer_magnitude > 0.1:
            # At full steering (1.0), reduce to 35% of target speed
            speed_factor = 1.0 - (steer_magnitude * 0.65)
            target_speed *= max(0.35, speed_factor)

        speed_error = target_speed - current_speed_kmh

        if speed_error > 0:
            # Accelerate - proportional control
            throttle = min(0.7, max(0.25, speed_error / 15.0))
            brake = 0.0
        else:
            # Decelerate
            throttle = 0.0
            brake = min(0.8, abs(speed_error) / 10.0)

        return float(throttle), float(brake)


# =============================================================================
# ROUTE DRIVER (for test driving)
# =============================================================================

class RouteDriver:
    """Handles spawning and driving a vehicle along a route."""

    def __init__(self, world, carla_map):
        self.world = world
        self.map = carla_map

        self.vehicle = None
        self.camera = None
        self.camera_image = None

        self.controller = LaneFollowingController(target_speed_kmh=30.0)
        self.current_route = None
        self.current_checkpoint_idx = 0

        self.is_driving = False
        self.is_finished = False

    def start_drive(self, route: ScenarioRoute):
        """Start driving a route."""
        self.cleanup()

        if not route.checkpoints:
            print("Route has no checkpoints!")
            return False

        self.current_route = route
        self.current_checkpoint_idx = 0
        self.is_finished = False

        # Get spawn transform from first checkpoint
        first_cp = route.checkpoints[0]
        spawn_transform = carla.Transform(
            carla.Location(x=first_cp['x'], y=first_cp['y'], z=first_cp['z'] + 0.5),
            carla.Rotation(yaw=first_cp['yaw'])
        )

        # Spawn vehicle
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]

        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
        print(f"Spawned vehicle for route test")

        # Attach camera (3rd person view)
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '360')
        camera_bp.set_attribute('fov', '100')

        camera_transform = carla.Transform(
            carla.Location(x=-6.0, z=3.0),
            carla.Rotation(pitch=-15)
        )

        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self._process_camera)

        self.is_driving = True
        return True

    def _process_camera(self, image):
        """Process camera image."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        self.camera_image = array[:, :, :3][:, :, ::-1].copy()

    def update(self):
        """Update vehicle control."""
        if not self.is_driving or self.vehicle is None or self.current_route is None:
            return

        if self.is_finished:
            # Apply brake when finished
            control = carla.VehicleControl()
            control.brake = 1.0
            self.vehicle.apply_control(control)
            return

        # Get current speed
        velocity = self.vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        # Compute control
        control, new_idx = self.controller.compute_control(
            self.current_route.checkpoints,
            self.current_checkpoint_idx,
            self.vehicle.get_transform(),
            speed_kmh
        )

        self.current_checkpoint_idx = new_idx

        # Check if finished
        if new_idx >= len(self.current_route.checkpoints) - 3:
            self.is_finished = True
            print("Route completed!")

        # Apply control
        self.vehicle.apply_control(control)

    def stop_drive(self):
        """Stop the current drive."""
        self.is_driving = False
        if self.vehicle:
            control = carla.VehicleControl()
            control.brake = 1.0
            self.vehicle.apply_control(control)

    def cleanup(self):
        """Clean up vehicle and camera."""
        if self.camera:
            self.camera.stop()
            self.camera.destroy()
            self.camera = None

        if self.vehicle:
            self.vehicle.destroy()
            self.vehicle = None

        self.camera_image = None
        self.is_driving = False
        self.is_finished = False
        self.current_route = None
        self.current_checkpoint_idx = 0

    def get_progress(self) -> float:
        """Get progress percentage."""
        if not self.current_route or not self.current_route.checkpoints:
            return 0.0
        return (self.current_checkpoint_idx / len(self.current_route.checkpoints)) * 100

    def get_speed_kmh(self) -> float:
        """Get current speed in km/h."""
        if not self.vehicle:
            return 0.0
        velocity = self.vehicle.get_velocity()
        return 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


class RouteBrowser:
    """Pygame UI for browsing and selecting routes."""

    def __init__(self, width: int = 1280, height: int = 720):
        self.width = width
        self.height = height

        # CARLA
        self.client = None
        self.world = None
        self.carla_map = None

        # Scenarios
        self.scenarios = {}
        self.checkpoint_manager = CheckpointManager()

        # UI State
        self.selected_type = None
        self.selected_route_idx = 0
        self.scroll_offset = 0

        # Test driving
        self.route_driver = None
        self.is_test_driving = False

        # Pygame
        self.display = None
        self.clock = None
        self.font = None
        self.font_large = None

        self.running = True

    def connect(self, host='localhost', port=2000, town='Town10HD'):
        """Connect to CARLA."""
        print(f"Connecting to CARLA at {host}:{port}...")
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)

        print(f"Loading world: {town}")
        self.world = self.client.load_world(town)
        self.carla_map = self.world.get_map()

        # Enable synchronous mode for driving
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / 30.0
        self.world.apply_settings(settings)

        # Initialize route driver and planner
        self.route_driver = RouteDriver(self.world, self.carla_map)
        self.route_planner = RoutePlanner(self.carla_map, sampling_resolution=1.0)

        # Spawn points for custom route creation
        self.spawn_points = self.carla_map.get_spawn_points()
        print(f"Found {len(self.spawn_points)} spawn points")

    def init_pygame(self):
        """Initialize pygame."""
        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("CARLA Route Scenario Browser")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('monospace', 16)
        self.font_large = pygame.font.SysFont('monospace', 24, bold=True)

    def find_scenarios(self):
        """Find all scenarios in current map."""
        finder = ScenarioFinder(self.world, self.carla_map)
        self.scenarios = finder.find_all_scenarios()

    def save_scenarios(self, filename: str = None):
        """Save current scenarios."""
        return self.checkpoint_manager.save_scenarios(self.scenarios, filename)

    def load_scenarios(self, filename: str):
        """Load scenarios from file."""
        self.scenarios = self.checkpoint_manager.load_scenarios(filename)

    def create_custom_route(self, start_location: carla.Location,
                            end_location: carla.Location,
                            route_type: str = "custom") -> Optional[ScenarioRoute]:
        """
        Create a custom route between two locations using GlobalRoutePlanner.

        Args:
            start_location: Starting point
            end_location: Destination point
            route_type: Type label for the route

        Returns:
            ScenarioRoute object or None if route couldn't be computed
        """
        # Compute route using GlobalRoutePlanner
        route_with_options = self.route_planner.compute_route(start_location, end_location)

        if len(route_with_options) < 5:
            print("Could not compute route - too short or no path found")
            return None

        # Extract waypoints
        waypoints = [wp for wp, _ in route_with_options]

        # Calculate route metrics
        total_dist = 0.0
        for i in range(len(waypoints) - 1):
            total_dist += waypoints[i].transform.location.distance(
                waypoints[i + 1].transform.location
            )

        # Analyze curvature
        total_yaw_change = 0.0
        for i in range(len(waypoints) - 1):
            yaw1 = waypoints[i].transform.rotation.yaw
            yaw2 = waypoints[i + 1].transform.rotation.yaw
            diff = yaw2 - yaw1
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
            total_yaw_change += diff

        avg_curvature = abs(total_yaw_change) / max(total_dist, 1.0)
        has_junction = any(wp.is_junction for wp in waypoints)

        # Create checkpoints
        checkpoints = []
        for wp in waypoints:
            loc = wp.transform.location
            rot = wp.transform.rotation
            checkpoints.append({
                'x': loc.x, 'y': loc.y, 'z': loc.z,
                'yaw': rot.yaw,
                'road_id': wp.road_id,
                'lane_id': wp.lane_id
            })

        route_id = f"custom_{self.carla_map.name}_{len(checkpoints)}_{int(datetime.now().timestamp())}"

        route = ScenarioRoute(
            id=route_id,
            route_type=route_type,
            town=self.carla_map.name,
            description=f"Custom route: {total_dist:.0f}m, {len(waypoints)} waypoints",
            checkpoints=checkpoints,
            total_distance=total_dist,
            avg_curvature=avg_curvature,
            has_junction=has_junction,
            created_at=datetime.now().isoformat()
        )

        print(f"Created custom route: {total_dist:.0f}m, {len(waypoints)} waypoints")
        return route

    def create_route_between_spawn_points(self, start_idx: int, end_idx: int) -> Optional[ScenarioRoute]:
        """
        Create a route between two spawn points by index.

        Args:
            start_idx: Index of start spawn point
            end_idx: Index of end spawn point

        Returns:
            ScenarioRoute or None
        """
        if start_idx < 0 or start_idx >= len(self.spawn_points):
            print(f"Invalid start index: {start_idx}")
            return None
        if end_idx < 0 or end_idx >= len(self.spawn_points):
            print(f"Invalid end index: {end_idx}")
            return None

        start_loc = self.spawn_points[start_idx].location
        end_loc = self.spawn_points[end_idx].location

        return self.create_custom_route(start_loc, end_loc, route_type="spawn_to_spawn")

    def get_selected_route(self) -> Optional[ScenarioRoute]:
        """Get currently selected route."""
        if self.selected_type is None:
            return None
        routes = self.scenarios.get(self.selected_type, [])
        if 0 <= self.selected_route_idx < len(routes):
            return routes[self.selected_route_idx]
        return None

    def draw_route_in_world(self, route: ScenarioRoute, color: carla.Color = None):
        """Draw route checkpoints in CARLA world."""
        if color is None:
            pygame_color = ROUTE_TYPE_COLORS.get(route.route_type, Colors.WHITE)
            color = carla.Color(pygame_color[0], pygame_color[1], pygame_color[2])

        checkpoints = route.checkpoints

        for i, cp in enumerate(checkpoints):
            loc = carla.Location(x=cp['x'], y=cp['y'], z=cp['z'] + 0.5)

            # Draw point
            self.world.debug.draw_point(loc, size=0.1, color=color, life_time=0.1)

            # Draw line to next checkpoint
            if i < len(checkpoints) - 1:
                next_cp = checkpoints[i + 1]
                next_loc = carla.Location(x=next_cp['x'], y=next_cp['y'], z=next_cp['z'] + 0.5)
                self.world.debug.draw_line(loc, next_loc, thickness=0.1, color=color, life_time=0.1)

        # Draw start marker
        if checkpoints:
            start = checkpoints[0]
            self.world.debug.draw_string(
                carla.Location(x=start['x'], y=start['y'], z=start['z'] + 2),
                "START",
                color=carla.Color(0, 255, 0),
                life_time=0.1
            )

            end = checkpoints[-1]
            self.world.debug.draw_string(
                carla.Location(x=end['x'], y=end['y'], z=end['z'] + 2),
                "END",
                color=carla.Color(255, 0, 0),
                life_time=0.1
            )

    def render(self):
        """Render the UI."""
        self.display.fill(Colors.DARK_GRAY)

        if self.is_test_driving:
            self._render_test_drive_view()
        else:
            self._render_browser_view()

        pygame.display.flip()

    def _render_browser_view(self):
        """Render the route browser view."""
        # Title
        title = self.font_large.render("CARLA Route Scenario Browser", True, Colors.WHITE)
        self.display.blit(title, (20, 20))

        # Instructions
        instructions = [
            "1-6: Select type | UP/DOWN: Navigate | T: Test drive | C: Create custom route",
            "S: Save | L: Load | F: Find scenarios | Q: Quit"
        ]
        for i, text in enumerate(instructions):
            surf = self.font.render(text, True, Colors.GRAY)
            self.display.blit(surf, (20, 55 + i * 18))

        # Route type buttons
        self._render_route_type_buttons()

        # Route list
        self._render_route_list()

        # Route details
        self._render_route_details()

        # Minimap
        self._render_minimap()

    def _render_test_drive_view(self):
        """Render the test driving view with camera feed."""
        # Camera feed
        if self.route_driver and self.route_driver.camera_image is not None:
            # Scale camera image to fit left side
            cam_width = 900
            cam_height = 506  # 16:9 aspect
            surface = pygame.surfarray.make_surface(self.route_driver.camera_image.swapaxes(0, 1))
            surface = pygame.transform.scale(surface, (cam_width, cam_height))
            self.display.blit(surface, (10, 10))

            # Camera border
            pygame.draw.rect(self.display, Colors.WHITE, (10, 10, cam_width, cam_height), 2)
        else:
            # Placeholder
            pygame.draw.rect(self.display, Colors.BLACK, (10, 10, 900, 506))
            text = self.font_large.render("Waiting for camera...", True, Colors.GRAY)
            self.display.blit(text, (350, 250))

        # Info panel on right side
        panel_x = 930
        panel_y = 20

        # Title
        title = self.font_large.render("TEST DRIVE", True, Colors.GREEN)
        self.display.blit(title, (panel_x, panel_y))

        # Route info
        if self.route_driver and self.route_driver.current_route:
            route = self.route_driver.current_route
            y = panel_y + 40

            info = [
                f"Route Type: {route.route_type}",
                f"Distance: {route.total_distance:.0f}m",
                f"Checkpoints: {len(route.checkpoints)}",
                "",
                f"Speed: {self.route_driver.get_speed_kmh():.1f} km/h",
                f"Progress: {self.route_driver.get_progress():.1f}%",
                f"Checkpoint: {self.route_driver.current_checkpoint_idx}/{len(route.checkpoints)}",
            ]

            for text in info:
                color = Colors.WHITE if text else Colors.GRAY
                surf = self.font.render(text, True, color)
                self.display.blit(surf, (panel_x, y))
                y += 22

            # Progress bar
            y += 10
            bar_width = 320
            bar_height = 20
            progress = self.route_driver.get_progress() / 100.0

            pygame.draw.rect(self.display, Colors.DARK_GRAY, (panel_x, y, bar_width, bar_height))
            pygame.draw.rect(self.display, Colors.GREEN, (panel_x, y, int(bar_width * progress), bar_height))
            pygame.draw.rect(self.display, Colors.WHITE, (panel_x, y, bar_width, bar_height), 1)

            # Status
            y += 40
            if self.route_driver.is_finished:
                status = "ROUTE COMPLETED!"
                status_color = Colors.GREEN
            else:
                status = "DRIVING..."
                status_color = Colors.CYAN

            surf = self.font_large.render(status, True, status_color)
            self.display.blit(surf, (panel_x, y))

        # Controls
        controls_y = 530
        controls = [
            "ESC: Stop test drive",
            "+/-: Adjust speed",
            "R: Restart route",
        ]
        for i, text in enumerate(controls):
            surf = self.font.render(text, True, Colors.GRAY)
            self.display.blit(surf, (panel_x, controls_y + i * 20))

        # Minimap during drive
        self._render_drive_minimap()

    def _render_route_type_buttons(self):
        """Render route type selection buttons."""
        y = 100
        x = 20
        btn_width = 200
        btn_height = 35

        route_types = [
            (RouteType.LANE_MAINTENANCE, "1. Lane Maintenance"),
            (RouteType.LEFT_CURVE, "2. Left Curve"),
            (RouteType.RIGHT_CURVE, "3. Right Curve"),
            (RouteType.INTERSECTION_STRAIGHT, "4. Intersection Straight"),
            (RouteType.INTERSECTION_LEFT, "5. Intersection Left"),
            (RouteType.INTERSECTION_RIGHT, "6. Intersection Right"),
        ]

        for rt, label in route_types:
            rect = pygame.Rect(x, y, btn_width, btn_height)
            color = ROUTE_TYPE_COLORS.get(rt.value, Colors.GRAY)

            # Highlight selected
            if self.selected_type == rt.value:
                pygame.draw.rect(self.display, color, rect)
                text_color = Colors.BLACK
            else:
                pygame.draw.rect(self.display, Colors.DARK_GRAY, rect)
                pygame.draw.rect(self.display, color, rect, 2)
                text_color = color

            # Count
            count = len(self.scenarios.get(rt.value, []))
            text = f"{label} ({count})"
            surf = self.font.render(text, True, text_color)
            self.display.blit(surf, (x + 10, y + 8))

            y += btn_height + 5

    def _render_route_list(self):
        """Render list of routes for selected type."""
        list_x = 240
        list_y = 100
        list_width = 400
        item_height = 28
        max_visible = 15

        # Header
        header = f"Routes" if self.selected_type else "Select a route type"
        surf = self.font_large.render(header, True, Colors.WHITE)
        self.display.blit(surf, (list_x, list_y - 30))

        if not self.selected_type:
            return

        routes = self.scenarios.get(self.selected_type, [])

        # Draw list background
        list_rect = pygame.Rect(list_x, list_y, list_width, max_visible * item_height)
        pygame.draw.rect(self.display, Colors.BLACK, list_rect)
        pygame.draw.rect(self.display, Colors.GRAY, list_rect, 1)

        visible_routes = routes[self.scroll_offset:self.scroll_offset + max_visible]

        for i, route in enumerate(visible_routes):
            idx = i + self.scroll_offset
            y = list_y + i * item_height

            item_rect = pygame.Rect(list_x + 2, y + 2, list_width - 4, item_height - 4)

            if idx == self.selected_route_idx:
                pygame.draw.rect(self.display, ROUTE_TYPE_COLORS.get(self.selected_type, Colors.BLUE), item_rect)
                text_color = Colors.BLACK
            else:
                text_color = Colors.WHITE

            # Route info
            text = f"#{idx+1} | {route.total_distance:.0f}m | curv: {route.avg_curvature:.3f}"
            surf = self.font.render(text, True, text_color)
            self.display.blit(surf, (list_x + 8, y + 5))

    def _render_route_details(self):
        """Render details of selected route."""
        details_x = 660
        details_y = 100

        surf = self.font_large.render("Route Details", True, Colors.WHITE)
        self.display.blit(surf, (details_x, details_y - 30))

        route = self.get_selected_route()
        if not route:
            surf = self.font.render("No route selected", True, Colors.GRAY)
            self.display.blit(surf, (details_x, details_y))
            return

        details = [
            f"ID: {route.id[:40]}...",
            f"Type: {route.route_type}",
            f"Town: {route.town}",
            f"",
            f"Checkpoints: {len(route.checkpoints)}",
            f"Distance: {route.total_distance:.1f} m",
            f"Avg Curvature: {route.avg_curvature:.4f}",
            f"Has Junction: {route.has_junction}",
            f"",
            f"Description:",
            f"  {route.description}",
        ]

        for i, text in enumerate(details):
            color = Colors.WHITE if text else Colors.GRAY
            surf = self.font.render(text, True, color)
            self.display.blit(surf, (details_x, details_y + i * 20))

    def _render_minimap(self):
        """Render minimap showing selected route."""
        map_x = 660
        map_y = 380
        map_width = 580
        map_height = 300

        # Background
        map_rect = pygame.Rect(map_x, map_y, map_width, map_height)
        pygame.draw.rect(self.display, Colors.BLACK, map_rect)
        pygame.draw.rect(self.display, Colors.GRAY, map_rect, 1)

        route = self.get_selected_route()
        if not route or not route.checkpoints:
            surf = self.font.render("No route to display", True, Colors.GRAY)
            self.display.blit(surf, (map_x + 10, map_y + 10))
            return

        checkpoints = route.checkpoints

        # Calculate bounds
        xs = [cp['x'] for cp in checkpoints]
        ys = [cp['y'] for cp in checkpoints]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        range_x = max(max_x - min_x, 1)
        range_y = max(max_y - min_y, 1)

        # Add padding
        padding = 20
        scale = min(
            (map_width - 2 * padding) / range_x,
            (map_height - 2 * padding) / range_y
        )

        def world_to_map(x, y):
            px = map_x + padding + (x - min_x) * scale
            py = map_y + padding + (y - min_y) * scale
            return int(px), int(py)

        # Draw route
        color = ROUTE_TYPE_COLORS.get(route.route_type, Colors.WHITE)

        points = [world_to_map(cp['x'], cp['y']) for cp in checkpoints]
        if len(points) > 1:
            pygame.draw.lines(self.display, color, False, points, 2)

        # Draw start/end markers
        if points:
            pygame.draw.circle(self.display, Colors.GREEN, points[0], 8)
            pygame.draw.circle(self.display, Colors.RED, points[-1], 8)

        # Labels
        surf = self.font.render("START", True, Colors.GREEN)
        self.display.blit(surf, (map_x + 10, map_y + map_height - 25))
        surf = self.font.render("END", True, Colors.RED)
        self.display.blit(surf, (map_x + 80, map_y + map_height - 25))

    def _render_drive_minimap(self):
        """Render minimap during test drive."""
        map_x = 930
        map_y = 350
        map_width = 320
        map_height = 160

        pygame.draw.rect(self.display, Colors.BLACK, (map_x, map_y, map_width, map_height))
        pygame.draw.rect(self.display, Colors.GRAY, (map_x, map_y, map_width, map_height), 1)

        if not self.route_driver or not self.route_driver.current_route:
            return

        route = self.route_driver.current_route
        checkpoints = route.checkpoints

        if not checkpoints:
            return

        # Calculate bounds
        xs = [cp['x'] for cp in checkpoints]
        ys = [cp['y'] for cp in checkpoints]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        range_x = max(max_x - min_x, 1)
        range_y = max(max_y - min_y, 1)

        padding = 15
        scale = min(
            (map_width - 2 * padding) / range_x,
            (map_height - 2 * padding) / range_y
        )

        def world_to_map(x, y):
            px = map_x + padding + (x - min_x) * scale
            py = map_y + padding + (y - min_y) * scale
            return int(px), int(py)

        # Draw route
        color = ROUTE_TYPE_COLORS.get(route.route_type, Colors.WHITE)
        current_idx = self.route_driver.current_checkpoint_idx

        for i, cp in enumerate(checkpoints):
            px, py = world_to_map(cp['x'], cp['y'])
            # Completed checkpoints in gray, remaining in color
            if i < current_idx:
                pygame.draw.circle(self.display, Colors.GRAY, (px, py), 2)
            else:
                pygame.draw.circle(self.display, color, (px, py), 2)

        # Draw start/end
        if checkpoints:
            start_pt = world_to_map(checkpoints[0]['x'], checkpoints[0]['y'])
            end_pt = world_to_map(checkpoints[-1]['x'], checkpoints[-1]['y'])
            pygame.draw.circle(self.display, Colors.GREEN, start_pt, 5)
            pygame.draw.circle(self.display, Colors.RED, end_pt, 5)

        # Draw vehicle position
        if self.route_driver.vehicle:
            v_loc = self.route_driver.vehicle.get_location()
            vx, vy = world_to_map(v_loc.x, v_loc.y)
            pygame.draw.circle(self.display, Colors.CYAN, (vx, vy), 6)
            pygame.draw.circle(self.display, Colors.WHITE, (vx, vy), 6, 2)

    def handle_events(self):
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                self._handle_keydown(event)

    def _handle_keydown(self, event):
        """Handle key presses."""
        # Handle test driving mode keys
        if self.is_test_driving:
            self._handle_test_drive_keys(event)
            return

        # Quit
        if event.key == pygame.K_q:
            self.running = False

        elif event.key == pygame.K_ESCAPE:
            self.running = False

        # Route type selection (1-6)
        elif event.key == pygame.K_1:
            self._select_type(RouteType.LANE_MAINTENANCE.value)
        elif event.key == pygame.K_2:
            self._select_type(RouteType.LEFT_CURVE.value)
        elif event.key == pygame.K_3:
            self._select_type(RouteType.RIGHT_CURVE.value)
        elif event.key == pygame.K_4:
            self._select_type(RouteType.INTERSECTION_STRAIGHT.value)
        elif event.key == pygame.K_5:
            self._select_type(RouteType.INTERSECTION_LEFT.value)
        elif event.key == pygame.K_6:
            self._select_type(RouteType.INTERSECTION_RIGHT.value)

        # Navigate routes
        elif event.key == pygame.K_UP:
            self._navigate_routes(-1)
        elif event.key == pygame.K_DOWN:
            self._navigate_routes(1)

        # Test drive selected route
        elif event.key == pygame.K_t:
            self._start_test_drive()

        # Save scenarios
        elif event.key == pygame.K_s:
            if self.scenarios:
                filepath = self.save_scenarios()
                print(f"Saved to: {filepath}")

        # Load scenarios
        elif event.key == pygame.K_l:
            files = self.checkpoint_manager.list_saved_files()
            if files:
                # Load most recent
                latest = sorted(files)[-1]
                self.load_scenarios(latest)
                print(f"Loaded: {latest}")
            else:
                print("No saved scenario files found")

        # Find scenarios
        elif event.key == pygame.K_f:
            print("Finding scenarios...")
            self.find_scenarios()

        # Create custom route between random spawn points
        elif event.key == pygame.K_c:
            self._create_and_drive_custom_route()

    def _create_and_drive_custom_route(self):
        """Create a custom route between two random spawn points and test drive it."""
        import random

        if len(self.spawn_points) < 2:
            print("Not enough spawn points!")
            return

        # Pick two random spawn points that are far enough apart
        attempts = 0
        while attempts < 20:
            start_idx = random.randint(0, len(self.spawn_points) - 1)
            end_idx = random.randint(0, len(self.spawn_points) - 1)

            if start_idx == end_idx:
                attempts += 1
                continue

            start_loc = self.spawn_points[start_idx].location
            end_loc = self.spawn_points[end_idx].location

            distance = start_loc.distance(end_loc)
            if distance > 50:  # At least 50m apart
                break
            attempts += 1

        print(f"Creating route from spawn #{start_idx} to spawn #{end_idx}")

        # Create the route using GlobalRoutePlanner
        route = self.create_route_between_spawn_points(start_idx, end_idx)

        if route:
            # Start test driving immediately
            if self.route_driver.start_drive(route):
                self.is_test_driving = True
                print(f"Starting custom route: {route.total_distance:.0f}m")
        else:
            print("Failed to create route")

    def _handle_test_drive_keys(self, event):
        """Handle keys during test drive mode."""
        if event.key == pygame.K_ESCAPE:
            # Stop test drive and return to browser
            self._stop_test_drive()

        elif event.key == pygame.K_r:
            # Restart current route
            route = self.get_selected_route()
            if route and self.route_driver:
                self.route_driver.start_drive(route)
                print("Restarting route...")

        elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
            # Increase speed
            if self.route_driver:
                self.route_driver.controller.target_speed_kmh = min(60, self.route_driver.controller.target_speed_kmh + 5)
                print(f"Target speed: {self.route_driver.controller.target_speed_kmh} km/h")

        elif event.key == pygame.K_MINUS:
            # Decrease speed
            if self.route_driver:
                self.route_driver.controller.target_speed_kmh = max(10, self.route_driver.controller.target_speed_kmh - 5)
                print(f"Target speed: {self.route_driver.controller.target_speed_kmh} km/h")

        elif event.key == pygame.K_q:
            # Quit completely
            self._stop_test_drive()
            self.running = False

    def _start_test_drive(self):
        """Start test driving the selected route."""
        route = self.get_selected_route()
        if not route:
            print("No route selected!")
            return

        if self.route_driver.start_drive(route):
            self.is_test_driving = True
            print(f"Starting test drive: {route.route_type}")

    def _stop_test_drive(self):
        """Stop test driving and return to browser."""
        if self.route_driver:
            self.route_driver.cleanup()
        self.is_test_driving = False
        print("Test drive stopped")

    def _select_type(self, route_type: str):
        """Select a route type."""
        self.selected_type = route_type
        self.selected_route_idx = 0
        self.scroll_offset = 0

    def _navigate_routes(self, direction: int):
        """Navigate through routes list."""
        if not self.selected_type:
            return

        routes = self.scenarios.get(self.selected_type, [])
        if not routes:
            return

        self.selected_route_idx = (self.selected_route_idx + direction) % len(routes)

        # Update scroll
        if self.selected_route_idx < self.scroll_offset:
            self.scroll_offset = self.selected_route_idx
        elif self.selected_route_idx >= self.scroll_offset + 15:
            self.scroll_offset = self.selected_route_idx - 14

    def run(self):
        """Main loop."""
        try:
            while self.running:
                self.handle_events()

                if self.is_test_driving:
                    # Update vehicle control
                    if self.route_driver:
                        self.route_driver.update()
                else:
                    # Draw selected route in CARLA world (browser mode)
                    route = self.get_selected_route()
                    if route:
                        self.draw_route_in_world(route)

                # Tick the world (synchronous mode)
                self.world.tick()

                self.render()
                self.clock.tick(30)

        finally:
            # Cleanup
            if self.route_driver:
                self.route_driver.cleanup()

            # Reset world settings
            if self.world:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)

            pygame.quit()


# =============================================================================
# MAIN
# =============================================================================

def main():
    import argparse

    parser = argparse.ArgumentParser(description='CARLA Route Scenario Finder')
    parser.add_argument('--host', default='localhost', help='CARLA host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port')
    parser.add_argument('--town', default='Town10HD', help='Town to load')
    parser.add_argument('--find', action='store_true', help='Auto-find scenarios on start')
    parser.add_argument('--load', type=str, help='Load scenarios from file')

    args = parser.parse_args()

    browser = RouteBrowser()
    browser.connect(args.host, args.port, args.town)
    browser.init_pygame()

    if args.load:
        browser.load_scenarios(args.load)
    elif args.find:
        browser.find_scenarios()

    browser.run()


if __name__ == '__main__':
    main()
