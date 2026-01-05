"""
Perception Package
Modular perception pipeline for autonomous vehicle research
"""

__version__ = "1.0.0"

from .core_types import (
    CameraIntrinsics,
    CameraPose,
    BEVConfig,
    CameraFrame,
    BEVRepresentation
)

from .camera_adapter import CameraAdapter
from .state import PerceptionState
from .occupancy_grid import OccupancyGrid2D
from .costmap import Costmap

__all__ = [
    'CameraIntrinsics',
    'CameraPose',
    'BEVConfig',
    'CameraFrame',
    'BEVRepresentation',
    'CameraAdapter',
    'PerceptionState',
    'OccupancyGrid2D',
    'Costmap',
]
