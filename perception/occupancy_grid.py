"""
2D Occupancy Grid - Probabilistic Grid Mapping

Standard log-odds Bayesian occupancy grid with Bresenham raycasting.
Based on Thrun's probabilistic robotics approach.

Usage:
    from perception import OccupancyGrid2D

    grid = OccupancyGrid2D(width=20.0, height=20.0, resolution=0.2)
    grid.update(lidar_points)

    # Query
    prob = grid.get_cost(x, y)
    is_blocked = grid.is_occupied(x, y)

    # For planning
    obstacles = grid.get_obstacle_points()
"""

import time
import numpy as np
from threading import RLock
from typing import Optional, Tuple


class OccupancyGrid2D:
    """
    2D probabilistic occupancy grid in vehicle frame

    Coordinate System:
        Origin: Vehicle center
        X-axis: Forward (positive ahead)
        Y-axis: Left (positive left)

    Data Format:
        Internal: Log-odds (-5.0 to 5.0)
        External: Probability (0.0 to 1.0)
    """

    LOG_ODDS_MIN = -5.0
    LOG_ODDS_MAX = 5.0

    def __init__(
        self,
        width: float = 40.0,
        height: float = 40.0,
        resolution: float = 0.1,
        log_odds_occ: float = 0.4,
        log_odds_free: float = -0.2,
        decay_factor: float = 0.95,
        use_raycasting: bool = True
    ):
        """
        Args:
            width: Grid width in meters
            height: Grid height in meters
            resolution: Cell size in meters
            log_odds_occ: Log-odds increment for occupied cells
            log_odds_free: Log-odds increment for free cells
            decay_factor: Temporal decay per second (0-1)
            use_raycasting: Enable Bresenham raycasting for free space
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.log_odds_occ = log_odds_occ
        self.log_odds_free = log_odds_free
        self.decay_factor = decay_factor
        self.use_raycasting = use_raycasting

        self.cells_x = int(width / resolution)
        self.cells_y = int(height / resolution)
        self.origin_x = width / 2.0
        self.origin_y = height / 2.0

        self.grid = np.zeros((self.cells_y, self.cells_x), dtype=np.float32)
        self._last_update_time = time.time()
        self._lock = RLock()  # Thread safety for concurrent access

    # -------------------------------------------------------------------------
    # Core Methods
    # -------------------------------------------------------------------------

    def update(self, points: np.ndarray, sensor_origin: Optional[np.ndarray] = None) -> int:
        """
        Update grid from point cloud with Bayesian log-odds update

        Args:
            points: Point cloud (N, 2) or (N, 3) in vehicle frame [x, y] or [x, y, z]
            sensor_origin: Sensor position in vehicle frame. Default: [0, 0]

        Returns:
            Number of points processed
        """
        with self._lock:
            # Apply temporal decay
            current_time = time.time()
            dt = current_time - self._last_update_time
            if dt > 0:
                # Cap dt to prevent sudden decay on stalls
                dt = min(dt, 1.0)
                self.grid *= self.decay_factor ** dt
            self._last_update_time = current_time

            if points is None or len(points) == 0:
                return 0

            # Convert to grid indices
            indices = self._world_to_grid_batch(points[:, 0], points[:, 1])

            # Filter valid indices
            valid_mask = (indices[:, 0] >= 0) & (indices[:, 0] < self.cells_y) & \
                         (indices[:, 1] >= 0) & (indices[:, 1] < self.cells_x)
            valid_indices = indices[valid_mask]

            if len(valid_indices) == 0:
                return 0

            # Raycasting: mark free space between sensor and obstacles
            if self.use_raycasting:
                if sensor_origin is None:
                    sensor_origin = np.array([0.0, 0.0])

                origin_row, origin_col = self.world_to_grid(sensor_origin[0], sensor_origin[1])
                origin_row = np.clip(origin_row, 0, self.cells_y - 1)
                origin_col = np.clip(origin_col, 0, self.cells_x - 1)

                for end_row, end_col in valid_indices:
                    ray_cells = self._bresenham(origin_col, origin_row, end_col, end_row)

                    if len(ray_cells) > 1:
                        free_cells = ray_cells[:-1]
                        free_rows, free_cols = free_cells[:, 0], free_cells[:, 1]
                        valid_free = (free_rows >= 0) & (free_rows < self.cells_y) & \
                                     (free_cols >= 0) & (free_cols < self.cells_x)

                        if np.any(valid_free):
                            self.grid[free_rows[valid_free], free_cols[valid_free]] = np.clip(
                                self.grid[free_rows[valid_free], free_cols[valid_free]] + self.log_odds_free,
                                self.LOG_ODDS_MIN, self.LOG_ODDS_MAX
                            )

            # Mark endpoints as occupied
            rows, cols = valid_indices[:, 0], valid_indices[:, 1]
            self.grid[rows, cols] = np.clip(
                self.grid[rows, cols] + self.log_odds_occ,
                self.LOG_ODDS_MIN, self.LOG_ODDS_MAX
            )

            return len(valid_indices)

    def get_cost(self, x: float, y: float) -> float:
        """
        Query occupancy probability at world coordinates

        Args:
            x: X position in meters
            y: Y position in meters

        Returns:
            Occupancy probability [0.0, 1.0], or 0.0 if out of bounds
        """
        row, col = self.world_to_grid(x, y)
        if 0 <= row < self.cells_y and 0 <= col < self.cells_x:
            with self._lock:
                log_odds = self.grid[row, col]
            # Use scipy.special.expit equivalent for numerical stability
            return 1.0 / (1.0 + np.exp(-np.clip(log_odds, -500, 500)))
        return 0.0

    def is_occupied(self, x: float, y: float, threshold: float = 0.7) -> bool:
        """
        Check if world position is occupied

        Args:
            x: X position in meters
            y: Y position in meters
            threshold: Occupancy threshold (default 0.7)

        Returns:
            True if occupied, False otherwise
        """
        return self.get_cost(x, y) > threshold

    def to_probability_grid(self) -> np.ndarray:
        """
        Convert log-odds grid to probability grid

        Returns:
            Probability grid (cells_y, cells_x) [0.0, 1.0]
        """
        with self._lock:
            grid_copy = self.grid.copy()
        return 1.0 / (1.0 + np.exp(-grid_copy))

    def get_obstacle_points(self, threshold: float = 0.7) -> np.ndarray:
        """
        Extract obstacle positions as (N, 2) array for path planning

        Args:
            threshold: Occupancy probability threshold

        Returns:
            Array of [x, y] obstacle positions in world coordinates
        """
        prob = self.to_probability_grid()
        rows, cols = np.where(prob > threshold)

        if len(rows) == 0:
            return np.array([]).reshape(0, 2)

        points = np.array([self.grid_to_world(r, c) for r, c in zip(rows, cols)])
        return points

    def clear(self) -> None:
        """Reset grid to empty state"""
        with self._lock:
            self.grid.fill(0.0)

    # -------------------------------------------------------------------------
    # Coordinate Transforms
    # -------------------------------------------------------------------------

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices (row, col)"""
        col = int(np.floor((x + self.origin_x) / self.resolution))
        row = int(np.floor((self.origin_y - y) / self.resolution))
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (x, y)"""
        x = col * self.resolution - self.origin_x
        y = self.origin_y - row * self.resolution
        return x, y

    # -------------------------------------------------------------------------
    # Planning Utilities
    # -------------------------------------------------------------------------

    def get_closest_obstacle(
        self,
        max_range: float = 20.0,
        threshold: float = 0.7
    ) -> Optional[Tuple[float, float, float]]:
        """
        Find closest obstacle ahead of vehicle

        Args:
            max_range: Maximum search range (meters)
            threshold: Occupancy threshold

        Returns:
            (x, y, distance) or None if no obstacle found
        """
        threshold_log_odds = np.log(threshold / (1.0 - threshold))
        with self._lock:
            occupied = np.argwhere(self.grid > threshold_log_odds)

        if len(occupied) == 0:
            return None

        min_dist = float('inf')
        closest = None

        for row, col in occupied:
            x, y = self.grid_to_world(row, col)
            if 0 < x < max_range:
                dist = np.sqrt(x**2 + y**2)
                if dist < min_dist:
                    min_dist = dist
                    closest = (x, y, dist)

        return closest

    def check_path_collision(
        self,
        waypoints: np.ndarray,
        threshold: float = 0.7
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Check if path collides with obstacles

        Args:
            waypoints: Path waypoints (N, 2) [x, y]
            threshold: Occupancy threshold

        Returns:
            (collision_detected, collision_point)
        """
        if waypoints is None or len(waypoints) == 0:
            return False, None

        for wp in waypoints:
            if self.get_cost(wp[0], wp[1]) > threshold:
                return True, wp

        return False, None

    def inflate_obstacles(self, radius: float, threshold: float = 0.7) -> np.ndarray:
        """
        Inflate obstacles by radius for C-Space planning

        Args:
            radius: Inflation radius in meters
            threshold: Occupancy threshold

        Returns:
            Inflated binary grid (cells_y, cells_x)
        """
        try:
            from scipy.ndimage import binary_dilation
        except ImportError:
            raise ImportError("scipy required for inflate_obstacles")

        prob_grid = self.to_probability_grid()
        binary_obstacles = prob_grid > threshold

        if not binary_obstacles.any():
            return binary_obstacles.astype(np.float32)

        radius_cells = int(np.ceil(radius / self.resolution))
        y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
        disk = (x**2 + y**2 <= radius_cells**2)

        return binary_dilation(binary_obstacles, structure=disk).astype(np.float32)

    # -------------------------------------------------------------------------
    # Internal Methods
    # -------------------------------------------------------------------------

    def _world_to_grid_batch(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Batch convert world coordinates to grid indices"""
        cols = np.floor((x + self.origin_x) / self.resolution).astype(int)
        rows = np.floor((self.origin_y - y) / self.resolution).astype(int)
        return np.column_stack([rows, cols])

    def _bresenham(self, x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
        """Bresenham's line algorithm for raycasting"""
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cells = []
        x, y = x0, y0

        while True:
            cells.append((y, x))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return np.array(cells, dtype=np.int32)

    # -------------------------------------------------------------------------
    # Backward Compatibility Aliases
    # -------------------------------------------------------------------------

    update_from_lidar = update
    get_occupancy = get_cost
    _world_to_grid = world_to_grid
    _grid_to_world = grid_to_world
