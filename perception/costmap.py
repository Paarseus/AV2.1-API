"""
Costmap - Inflated Obstacle Map for Collision Checking

Wraps OccupancyGrid2D and inflates obstacles by robot radius.
After inflation, a single point query determines if the robot
(centered at that point) would collide.

Usage:
    from perception import OccupancyGrid2D, Costmap

    grid = OccupancyGrid2D(width=20, height=20, resolution=0.2)
    costmap = Costmap(robot_radius=0.5, safety_margin=0.2)

    # In control loop
    grid.update(lidar_points)
    costmap.update(grid)

    # Collision check
    collision, point = costmap.check_path(waypoints)
"""

import numpy as np
from threading import RLock
from typing import Tuple, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .occupancy_grid import OccupancyGrid2D


class Costmap:
    """
    Inflated costmap for collision checking.

    Robot radius is baked into the costmap via inflation.
    Query any point to check if placing robot center there causes collision.
    """

    def __init__(self, robot_radius: float, safety_margin: float = 0.0):
        """
        Args:
            robot_radius: Robot collision radius (meters)
            safety_margin: Extra clearance beyond robot_radius (meters)
        """
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.inflation_radius = robot_radius + safety_margin

        # Internal state
        self._costmap: Optional[np.ndarray] = None
        self._raw_obstacle_grid: Optional[np.ndarray] = None
        self._raw_obstacles: np.ndarray = np.array([]).reshape(0, 2)
        self._resolution: float = 0.1
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._lock = RLock()  # Thread safety for concurrent access

    def update(self, grid: 'OccupancyGrid2D', threshold: float = 0.55) -> None:
        """
        Rebuild costmap from occupancy grid.

        Args:
            grid: Source OccupancyGrid2D with obstacle data
            threshold: Occupancy probability threshold for obstacles
        """
        from scipy.ndimage import binary_dilation

        # Get data from grid (grid has its own lock)
        prob_grid = grid.to_probability_grid()
        obstacles = prob_grid > threshold
        raw_obstacles = grid.get_obstacle_points(threshold=threshold)

        # Create circular kernel for inflation
        radius_cells = int(np.ceil(self.inflation_radius / grid.resolution))
        y, x = np.ogrid[-radius_cells:radius_cells + 1, -radius_cells:radius_cells + 1]
        kernel = (x * x + y * y) <= (radius_cells * radius_cells)

        # Compute inflated costmap
        if obstacles.any():
            costmap = binary_dilation(obstacles, structure=kernel).astype(np.float32)
        else:
            costmap = np.zeros_like(prob_grid, dtype=np.float32)

        # Atomically update all internal state
        with self._lock:
            self._resolution = grid.resolution
            self._origin_x = grid.origin_x
            self._origin_y = grid.origin_y
            self._raw_obstacle_grid = obstacles.astype(np.float32)
            self._raw_obstacles = raw_obstacles
            self._costmap = costmap

    def get_cost(self, x: float, y: float) -> float:
        """
        Get cost at world position.

        Args:
            x: X position in meters
            y: Y position in meters

        Returns:
            1.0 if collision, 0.0 if free
        """
        with self._lock:
            if self._costmap is None:
                return 0.0

            row, col = self._world_to_grid(x, y)
            if 0 <= row < self._costmap.shape[0] and 0 <= col < self._costmap.shape[1]:
                return float(self._costmap[row, col])
            return 0.0

    def check_collision(self, x: float, y: float) -> bool:
        """Check if position is in collision."""
        return self.get_cost(x, y) > 0.5

    def check_path(
        self,
        waypoints: np.ndarray
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Check if path collides with obstacles.

        Args:
            waypoints: Array of positions, shape (N, 2) or (N, >=2)
                       Only x, y columns are used.

        Returns:
            (collision, point): collision=True if any point collides,
                               point is first collision location or None
        """
        with self._lock:
            if self._costmap is None:
                return False, None

        if waypoints is None or len(waypoints) == 0:
            return False, None

        pts = np.asarray(waypoints)
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)

        for wp in pts:
            if self.check_collision(wp[0], wp[1]):
                return True, np.array([wp[0], wp[1]])

        return False, None

    def get_obstacle_points(self) -> np.ndarray:
        """
        Get RAW obstacle positions (before inflation).

        These are the actual obstacle cell centers, not the inflated zone.
        Use for gradient-based cost in DWA.

        Returns:
            Array of [x, y] positions in world coordinates, shape (N, 2)
        """
        with self._lock:
            return self._raw_obstacles.copy()

    def to_rgb(self) -> Optional[np.ndarray]:
        """
        Convert costmap to RGB image for visualization.

        Colors:
            - White: Free space
            - Dark red: Raw obstacles
            - Orange: Inflation zone

        Returns:
            RGB numpy array (H, W, 3) with float values 0-1, or None if not initialized
        """
        with self._lock:
            if self._costmap is None:
                return None

            # Take copies to work with outside the lock
            costmap = self._costmap.copy()
            raw_obstacle_grid = self._raw_obstacle_grid.copy() if self._raw_obstacle_grid is not None else None

        h, w = costmap.shape
        rgb = np.ones((h, w, 3), dtype=np.float32)

        inflated = costmap > 0.5
        raw_obs = raw_obstacle_grid > 0.5 if raw_obstacle_grid is not None else inflated
        inflation_only = inflated & ~raw_obs

        rgb[raw_obs] = [0.8, 0.0, 0.0]       # Dark red
        rgb[inflation_only] = [1.0, 0.7, 0.4]  # Orange

        return rgb

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices (row, col)."""
        col = int(np.floor((x + self._origin_x) / self._resolution))
        row = int(np.floor((self._origin_y - y) / self._resolution))
        return row, col

    def _grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (x, y)."""
        x = col * self._resolution - self._origin_x
        y = self._origin_y - row * self._resolution
        return x, y
