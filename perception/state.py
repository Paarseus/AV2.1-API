"""
Thread-Safe State Management for Perception System

Provides thread-safe containers for inter-thread communication between
perception, control, and visualization threads.
"""

from threading import Lock
from typing import Optional, Tuple, List
import time

from .core_types import BEVRepresentation, YOLOPv2Output


class PerceptionState:
    """
    Thread-safe perception data container for control + visualization

    Used for communication between perception thread and main/control threads.
    Stores both raw data for visualization and processed data for control.

    Usage:
        # In perception thread
        perception_state = PerceptionState()
        while running:
            bev = ipm.process(frame)
            yolo = yolo_model.infer(frame)
            perception_state.update(bev=bev, yolo_output=yolo, ...)

        # In control thread
        perception = perception_state.get_control_data()
        steering_correction = compute_correction(perception['lane_offset'])

        # In visualization thread
        viz_data = perception_state.get_visualization_data()
        viewer.update_bev(viz_data['bev'])
    """

    def __init__(self):
        self._lock = Lock()

        # Visualization data
        self._bev_image: Optional[BEVRepresentation] = None
        self._yolo_raw: Optional[YOLOPv2Output] = None

        # CONTROL DATA (extracted from vision)
        self._lane_center_offset: float = 0.0      # Lateral offset from lane center (m, left+)
        self._lane_heading_error: float = 0.0      # Angle error from lane direction (rad)
        self._has_lane_detection: bool = False     # Confidence flag

        self._obstacles: List = []                  # List of [x, y, width, class, confidence]
        self._closest_obstacle_dist: float = float('inf')

        self._drivable_corridor: Optional[Tuple[float, float]] = None  # (left_bound, right_bound)

        # Occupancy grid (from LIDAR)
        self._occupancy_grid = None  # OccupancyGrid2D instance

        self._timestamp: float = 0.0
        self._processing_fps: float = 0.0

    def update(self, bev_repr: Optional[BEVRepresentation] = None,
               yolo_output: Optional[YOLOPv2Output] = None,
               lane_offset: Optional[float] = None,
               lane_heading_error: Optional[float] = None,
               obstacles: Optional[List] = None,
               drivable_corridor: Optional[Tuple[float, float]] = None,
               occupancy_grid = None,
               processing_fps: float = 0.0):
        """
        Update both visualization and control data (called by perception thread)

        Args:
            bev_repr: BEV representation for visualization
            yolo_output: YOLOPv2 output for visualization
            lane_offset: Lateral offset from lane center (m)
            lane_heading_error: Heading error from lane (rad)
            obstacles: List of detected obstacles
            drivable_corridor: Drivable area boundaries
            occupancy_grid: OccupancyGrid2D instance from LIDAR
            processing_fps: Perception processing FPS
        """
        with self._lock:
            # Visualization data
            if bev_repr is not None:
                self._bev_image = bev_repr
            if yolo_output is not None:
                self._yolo_raw = yolo_output

            # Control data
            if lane_offset is not None:
                self._lane_center_offset = lane_offset
                self._has_lane_detection = True
            else:
                self._has_lane_detection = False

            if lane_heading_error is not None:
                self._lane_heading_error = lane_heading_error

            if obstacles is not None:
                self._obstacles = obstacles
                # Find closest obstacle in front
                if len(obstacles) > 0:
                    front_obstacles = [obs for obs in obstacles if obs[0] > 0]  # x > 0
                    if front_obstacles:
                        self._closest_obstacle_dist = min(obs[0] for obs in front_obstacles)
                    else:
                        self._closest_obstacle_dist = float('inf')
                else:
                    self._closest_obstacle_dist = float('inf')

            if drivable_corridor is not None:
                self._drivable_corridor = drivable_corridor

            if occupancy_grid is not None:
                self._occupancy_grid = occupancy_grid

            self._processing_fps = processing_fps
            self._timestamp = time.time()

    def get_control_data(self) -> dict:
        """
        Get control-relevant data (called by control thread)

        Returns a thread-safe snapshot of all control data. The occupancy_grid
        is a deep copy, preventing race conditions between perception updates
        and control reads.

        Returns:
            Dictionary with lane_offset, obstacles, occupancy_grid, etc.
        """
        with self._lock:
            # Create thread-safe snapshot of occupancy grid
            occupancy_snapshot = self._occupancy_grid.copy() if self._occupancy_grid is not None else None

            return {
                'lane_offset': self._lane_center_offset,
                'lane_heading_error': self._lane_heading_error,
                'has_lane_detection': self._has_lane_detection,
                'obstacles': self._obstacles.copy() if self._obstacles else [],
                'closest_obstacle': self._closest_obstacle_dist,
                'drivable_corridor': self._drivable_corridor,
                'occupancy_grid': occupancy_snapshot,
                'timestamp': self._timestamp
            }

    def get_visualization_data(self) -> dict:
        """
        Get visualization data (called by main/visualization thread)

        Returns:
            Dictionary with bev, yolo, fps
        """
        with self._lock:
            return {
                'bev': self._bev_image,
                'yolo': self._yolo_raw,
                'processing_fps': self._processing_fps,
                'timestamp': self._timestamp
            }

    def get_age(self) -> float:
        """Get age of data in seconds"""
        with self._lock:
            return time.time() - self._timestamp

    def is_fresh(self, max_age: float = 1.0) -> bool:
        """Check if data is fresh (age < max_age seconds)"""
        return self.get_age() < max_age

    def get(self) -> Optional[BEVRepresentation]:
        """
        Get latest BEV (simplified access for visualization-only use)

        Returns:
            BEV representation or None
        """
        with self._lock:
            return self._bev_image
