"""
Core Data Types for Perception Pipeline
Standardized data structures for camera frames, BEV representations, etc.
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np


@dataclass
class CameraIntrinsics:
    """
    Camera intrinsic parameters
    Standard format used across all perception modules
    """
    fx: float          # Focal length x (pixels)
    fy: float          # Focal length y (pixels)
    cx: float          # Principal point x (pixels)
    cy: float          # Principal point y (pixels)
    width: int         # Image width (pixels)
    height: int        # Image height (pixels)

    def to_matrix(self) -> np.ndarray:
        """Convert to 3x3 intrinsic matrix"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)


@dataclass
class CameraPose:
    """
    Camera pose (extrinsics) relative to vehicle ground plane

    Standard Vehicle Frame (ISO 8855):
    - Origin: Vehicle center on ground
    - X-axis: Forward (ahead of vehicle)
    - Y-axis: Left (positive to left side)
    - Z-axis: Up (perpendicular to ground)

    Camera Frame (OpenCV convention):
    - X-axis: Right
    - Y-axis: Down
    - Z-axis: Forward (optical axis)
    """
    # Translation (meters)
    height: float = 1.2      # Height above ground
    forward: float = 0.0     # Forward offset from vehicle center
    lateral: float = 0.0     # Lateral offset (left positive)

    # Rotation (degrees)
    pitch: float = 5.0      # Tilt down (positive = looks down)
    roll: float = 0.0        # Roll angle
    yaw: float = 0.0         # Pan angle


@dataclass
class BEVConfig:
    """
    Bird's Eye View output configuration

    Standard BEV Image Layout (top-down view):
    - Rows: Row 0 (top) = far (x_max), Row height-1 (bottom) = near (x_min)
    - Cols: Col 0 (left) = left (y_max), Col width-1 (right) = right (y_min)

    Vehicle at origin (0, 0) on ground plane (z=0)
    """
    x_min: float = 0.0       # Near distance from vehicle (meters)
    x_max: float = 20.0      # Far distance from vehicle (meters)
    y_min: float = -5.0      # Right side extent (meters, negative)
    y_max: float = 5.0       # Left side extent (meters, positive)
    resolution: float = 0.01 # Meters per pixel (10mm default)

    def get_size(self) -> Tuple[int, int]:
        """Get output image size (width, height) in pixels"""
        width = int((self.y_max - self.y_min) / self.resolution)
        height = int((self.x_max - self.x_min) / self.resolution)
        return (width, height)


@dataclass
class CameraFrame:
    """
    Standardized camera output
    Common format for all camera types
    """
    timestamp: float
    image: np.ndarray              # RGB image (H×W×3)
    depth: Optional[np.ndarray]    # Depth map (H×W) in meters, optional
    intrinsics: CameraIntrinsics
    camera_id: str = "camera"

    def has_depth(self) -> bool:
        """Check if depth data is available"""
        return self.depth is not None


@dataclass
class BEVRepresentation:
    """
    Bird's Eye View representation output
    Result of IPM transformation
    """
    timestamp: float
    bev_image: np.ndarray                    # Top-down view (H×W×3)
    point_cloud: Optional[np.ndarray]        # 3D points in vehicle frame (N×3), optional
    grid_resolution: float                   # meters/pixel
    bounds: Tuple[float, float, float, float]  # (x_min, x_max, y_min, y_max)
    source_camera: str = "camera"


@dataclass
class YOLOPv2Output:
    """
    YOLOPv2 multi-task output: detection + segmentation
    """
    timestamp: float
    drivable_area: np.ndarray        # Binary mask (H, W) uint8: 0 or 255
    lane_lines: np.ndarray           # Binary mask (H, W) uint8: 0 or 255
    detections: np.ndarray           # Bounding boxes (N, 6): [x1, y1, x2, y2, conf, class]
    image_shape: Tuple[int, int]     # Original image (height, width)
    camera_id: str = "camera"


@dataclass
class LayerConfig:
    """
    Visualization layer configuration
    Defines how a data layer should be rendered in 3D viewer
    """
    name: str                        # Layer identifier
    layer_type: str                  # 'mesh', 'pointcloud', 'lineset', 'voxelgrid'
    visible: bool = True             # Show/hide layer
    color: Tuple[float, float, float] = (1.0, 1.0, 1.0)  # Default color RGB
    point_size: float = 2.0          # Point size for point clouds
    line_width: float = 1.0          # Line width for line sets
