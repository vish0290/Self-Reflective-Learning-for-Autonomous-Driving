#!/usr/bin/env python3
"""
Route Builder Module for Autonomous Driving

Handles route loading from JSON checkpoint files and waypoint extraction.
This module consolidates the RouteLoader and CheckpointWaypointExtractor
that were duplicated across vlm_auto_drive.py and record_dataset.py.

Usage:
    from route_builder import RouteBuilder, RouteCheckpoint

    # Load routes
    builder = RouteBuilder(routes_dir='./routes')
    # or
    builder = RouteBuilder(route_file='./routes/highway.json')

    # Get waypoints at specific distances
    waypoints_3d = builder.get_waypoints_at_distances(
        route_index=0,
        vehicle_location=vehicle.get_transform().location,
        distances=[3, 6, 9, 12, 15]
    )
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Protocol


class Location(Protocol):
    """Protocol for location objects."""

    x: float
    y: float
    z: float


@dataclass
class RouteCheckpoint:
    """Single checkpoint in a route."""

    x: float
    y: float
    z: float
    yaw: float
    road_id: int
    lane_id: int


@dataclass
class Waypoint3D:
    """3D waypoint in world coordinates."""

    x: float
    y: float
    z: float


@dataclass
class Route:
    """Complete route with metadata."""

    name: str
    filepath: str
    checkpoints: List[RouteCheckpoint]
    total_distance: float
    metadata: Dict


# Default waypoint distances (meters)
DEFAULT_WAYPOINT_DISTANCES = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30]


class RouteBuilder:
    """
    Loads and manages route checkpoint files.

    Provides functionality to:
    - Load routes from JSON files (single or directory)
    - Extract waypoints at specific distances from checkpoints
    - Check route completion status
    """

    def __init__(
        self,
        routes_dir: Optional[str] = None,
        route_file: Optional[str] = None,
        waypoint_distances: Optional[List[float]] = None,
    ):
        """
        Initialize route builder.

        Args:
            routes_dir: Directory containing route JSON files (loads all)
            route_file: Single route file to load
            waypoint_distances: Distances for waypoint extraction (default: [3,6,9,...,30])

        Raises:
            ValueError: If neither routes_dir nor route_file is provided
        """
        self.routes: List[Route] = []
        self.waypoint_distances = waypoint_distances or DEFAULT_WAYPOINT_DISTANCES

        if route_file:
            self._load_route(route_file)
        elif routes_dir:
            self._load_routes_from_directory(routes_dir)
        else:
            raise ValueError("Must provide either routes_dir or route_file")

    def _load_route(self, filepath: str) -> Route:
        """Load a single route file."""
        with open(filepath, "r") as f:
            route_data = json.load(f)

        route_name = Path(filepath).stem

        # Handle different route file formats
        checkpoint_data = None
        total_distance = 0.0

        # Format 1: Direct checkpoints at root level
        if "checkpoints" in route_data:
            checkpoint_data = route_data["checkpoints"]
            total_distance = route_data.get("total_distance", 0)

        # Format 2: Nested in scenarios.custom[0]
        elif "scenarios" in route_data and "custom" in route_data["scenarios"]:
            custom_routes = route_data["scenarios"]["custom"]
            if custom_routes:
                custom_route = custom_routes[0]
                checkpoint_data = custom_route.get("checkpoints", [])
                # Calculate distance from checkpoints if not provided
                if checkpoint_data and len(checkpoint_data) > 1:
                    total_distance = self._calculate_route_distance(checkpoint_data)

        if not checkpoint_data:
            raise ValueError("No checkpoints found in route file")

        # Parse checkpoints
        checkpoints = []
        for cp in checkpoint_data:
            checkpoints.append(
                RouteCheckpoint(
                    x=cp["x"],
                    y=cp["y"],
                    z=cp["z"],
                    yaw=cp.get("yaw", 0.0),
                    road_id=cp.get("road_id", 0),
                    lane_id=cp.get("lane_id", 0),
                )
            )

        route = Route(
            name=route_name,
            filepath=filepath,
            checkpoints=checkpoints,
            total_distance=total_distance,
            metadata=route_data,
        )

        self.routes.append(route)
        return route

    def _load_routes_from_directory(self, dirpath: str) -> None:
        """Load all route JSON files from directory."""
        routes_path = Path(dirpath)
        if not routes_path.exists():
            raise FileNotFoundError(f"Routes directory not found: {dirpath}")

        json_files = sorted(routes_path.glob("*.json"))

        if not json_files:
            raise ValueError(f"No JSON files found in {dirpath}")

        for filepath in json_files:
            try:
                self._load_route(str(filepath))
                print(f"  Loaded: {filepath.stem}")
            except Exception as e:
                print(f"  Error loading {filepath.name}: {e}")

        print(f"\nTotal routes loaded: {len(self.routes)}")

        if not self.routes:
            raise ValueError(f"Failed to load any routes from {dirpath}")

    @staticmethod
    def _calculate_route_distance(checkpoint_data: List[Dict]) -> float:
        """Calculate total distance from checkpoint list."""
        total_distance = 0.0
        for i in range(1, len(checkpoint_data)):
            dx = checkpoint_data[i]["x"] - checkpoint_data[i - 1]["x"]
            dy = checkpoint_data[i]["y"] - checkpoint_data[i - 1]["y"]
            total_distance += math.sqrt(dx * dx + dy * dy)
        return total_distance

    def get_route(self, index: int) -> Route:
        """Get route by index (wraps around)."""
        return self.routes[index % len(self.routes)]

    def get_route_count(self) -> int:
        """Get total number of routes."""
        return len(self.routes)

    def get_route_by_name(self, name: str) -> Optional[Route]:
        """Get route by name."""
        for route in self.routes:
            if route.name == name:
                return route
        return None

    def find_closest_checkpoint(
        self, route: Route, vehicle_location: Location
    ) -> int:
        """Find checkpoint index closest to vehicle."""
        min_dist = float("inf")
        closest_idx = 0

        for i, cp in enumerate(route.checkpoints):
            dist = math.sqrt(
                (cp.x - vehicle_location.x) ** 2 + (cp.y - vehicle_location.y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def get_waypoints_at_distances(
        self,
        route: Route,
        vehicle_location: Location,
        distances: Optional[List[float]] = None,
    ) -> List[Waypoint3D]:
        """
        Extract waypoints at target distances from checkpoint route.

        Args:
            route: Route object to extract from
            vehicle_location: Current vehicle location
            distances: Target distances (default: self.waypoint_distances)

        Returns:
            List of Waypoint3D at target distances
        """
        distances = distances or self.waypoint_distances
        checkpoints = route.checkpoints

        # Find closest checkpoint
        closest_idx = self.find_closest_checkpoint(route, vehicle_location)

        # Calculate cumulative distances from closest checkpoint
        cumulative_dists = self._calculate_cumulative_distances(checkpoints, closest_idx)

        # Extract waypoints at target distances
        waypoints_3d = []

        for target_dist in distances:
            # Find checkpoint at this distance
            cp_idx = None
            for i, cum_dist in enumerate(cumulative_dists):
                if cum_dist >= target_dist:
                    cp_idx = closest_idx + i
                    break

            # If we've reached end of route, use last checkpoint
            if cp_idx is None or cp_idx >= len(checkpoints):
                cp_idx = len(checkpoints) - 1

            cp = checkpoints[cp_idx]
            waypoints_3d.append(Waypoint3D(x=cp.x, y=cp.y, z=cp.z))

        return waypoints_3d

    def _calculate_cumulative_distances(
        self, checkpoints: List[RouteCheckpoint], start_idx: int
    ) -> List[float]:
        """Calculate cumulative distances from start checkpoint."""
        cumulative = [0.0]

        for i in range(start_idx + 1, len(checkpoints)):
            prev_cp = checkpoints[i - 1]
            curr_cp = checkpoints[i]

            dist = math.sqrt(
                (curr_cp.x - prev_cp.x) ** 2 + (curr_cp.y - prev_cp.y) ** 2
            )
            cumulative.append(cumulative[-1] + dist)

        return cumulative

    def is_route_complete(
        self, route: Route, vehicle_location: Location, threshold_meters: float = 5.0
    ) -> bool:
        """
        Check if vehicle has reached route end.

        Args:
            route: Route to check
            vehicle_location: Current vehicle location
            threshold_meters: Distance threshold for completion

        Returns:
            True if vehicle is within threshold of last checkpoint
        """
        last_checkpoint = route.checkpoints[-1]
        dist_to_end = math.sqrt(
            (vehicle_location.x - last_checkpoint.x) ** 2
            + (vehicle_location.y - last_checkpoint.y) ** 2
        )
        return dist_to_end < threshold_meters

    def get_spawn_transform(self, route: Route) -> Dict:
        """
        Get spawn transform from first checkpoint.

        Returns:
            Dict with location (x, y, z) and rotation (yaw)
        """
        first_cp = route.checkpoints[0]
        return {
            "location": {"x": first_cp.x, "y": first_cp.y, "z": first_cp.z + 0.5},
            "rotation": {"yaw": first_cp.yaw},
        }


# =============================================================================
# TEST
# =============================================================================

if __name__ == "__main__":
    print("Testing Route Builder Module")
    print("=" * 50)

    # Test with mock data
    import tempfile
    import os

    # Create a temporary route file
    test_route = {
        "checkpoints": [
            {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 5.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 10.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 15.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 20.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 25.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 30.0, "y": 0.0, "z": 0.5, "yaw": 0.0, "road_id": 1, "lane_id": -1},
            {"x": 35.0, "y": 5.0, "z": 0.5, "yaw": 45.0, "road_id": 1, "lane_id": -1},
            {"x": 40.0, "y": 10.0, "z": 0.5, "yaw": 45.0, "road_id": 1, "lane_id": -1},
        ],
        "total_distance": 50.0,
    }

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False
    ) as tmp_file:
        json.dump(test_route, tmp_file)
        tmp_path = tmp_file.name

    try:
        # Load route
        builder = RouteBuilder(route_file=tmp_path, waypoint_distances=[5, 10, 15, 20])
        print(f"Loaded {builder.get_route_count()} route(s)")

        route = builder.get_route(0)
        print(f"Route name: {route.name}")
        print(f"Checkpoints: {len(route.checkpoints)}")
        print(f"Total distance: {route.total_distance}")

        # Mock vehicle location
        class MockLocation:
            x = 2.0
            y = 0.0
            z = 0.5

        location = MockLocation()

        # Get waypoints
        waypoints = builder.get_waypoints_at_distances(route, location)
        print(f"\nWaypoints at distances {builder.waypoint_distances}:")
        for i, wp in enumerate(waypoints):
            print(f"  {i + 1}. ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})")

        # Check completion
        is_complete = builder.is_route_complete(route, location)
        print(f"\nRoute complete: {is_complete}")

        # Get spawn transform
        spawn = builder.get_spawn_transform(route)
        print(f"\nSpawn transform:")
        print(f"  Location: {spawn['location']}")
        print(f"  Rotation: {spawn['rotation']}")

    finally:
        os.unlink(tmp_path)

    print("\nRoute Builder Module test complete!")
