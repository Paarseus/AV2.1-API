"""
IPM (Inverse Perspective Mapping) Processor
Transforms camera images to bird's eye view using proven algorithm from ipm_caliberation.py

Clean, reusable implementation that works with standardized data types
"""

import numpy as np
import cv2
from typing import Tuple, Optional
from ..core_types import CameraIntrinsics, CameraPose, BEVConfig, CameraFrame, BEVRepresentation


class IPMProcessor:
    """
    Inverse Perspective Mapping processor

    Transforms camera frames to bird's eye view representation
    Uses precomputed pixel mapping for efficient real-time processing

    Usage:
        processor = IPMProcessor(intrinsics, pose, bev_config)
        bev_repr = processor.process(camera_frame)
    """

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        pose: CameraPose,
        bev_config: BEVConfig
    ):
        """
        Initialize IPM processor

        Args:
            intrinsics: Camera intrinsic parameters
            pose: Camera pose relative to vehicle
            bev_config: BEV output configuration
        """
        self.intrinsics = intrinsics
        self.pose = pose
        self.config = bev_config

        # Build transformation matrices
        self.K = intrinsics.to_matrix()
        self.T_vehicle_to_camera = self._build_transform_matrix()

        # Precompute pixel mapping
        print(f"[IPMProcessor] Building transform for {bev_config.get_size()} BEV...")
        self.map_x, self.map_y = self._precompute_mapping()
        print(f"[IPMProcessor] ✓ Ready")

    def _build_transform_matrix(self) -> np.ndarray:
        """
        Build 4×4 transformation from vehicle frame to camera frame
        Uses proven approach from ipm_caliberation.py
        """
        # Convert to radians
        pitch_rad = np.radians(self.pose.pitch)
        roll_rad = np.radians(self.pose.roll)
        yaw_rad = np.radians(self.pose.yaw)

        # Base rotation: vehicle → camera frame
        R_base = np.array([
            [0, -1,  0],
            [0,  0, -1],
            [1,  0,  0]
        ], dtype=np.float32)

        # Pitch rotation (tilt down)
        R_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
            [0, np.sin(pitch_rad), np.cos(pitch_rad)]
        ], dtype=np.float32)

        # Roll rotation
        R_roll = np.array([
            [np.cos(roll_rad), -np.sin(roll_rad), 0],
            [np.sin(roll_rad), np.cos(roll_rad), 0],
            [0, 0, 1]
        ], dtype=np.float32)

        # Yaw rotation
        R_yaw = np.array([
            [np.cos(yaw_rad), 0, np.sin(yaw_rad)],
            [0, 1, 0],
            [-np.sin(yaw_rad), 0, np.cos(yaw_rad)]
        ], dtype=np.float32)

        # Combined rotation
        R = R_yaw @ R_roll @ R_pitch @ R_base

        # Translation
        t_vehicle = np.array([self.pose.forward, self.pose.lateral, self.pose.height])
        t_camera = -R @ t_vehicle

        # Build 4×4 matrix
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3] = t_camera

        return T

    def _precompute_mapping(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Precompute mapping from BEV pixels to camera pixels

        Returns:
            map_x, map_y: Pixel coordinate maps for cv2.remap
        """
        width, height = self.config.get_size()

        map_x = np.zeros((height, width), dtype=np.float32)
        map_y = np.zeros((height, width), dtype=np.float32)

        # For each BEV pixel
        for i in range(height):
            for j in range(width):
                # Convert BEV pixel to world coordinates (STANDARD)
                # Row 0 = far (x_max), Row height-1 = near (x_min)
                # Col 0 = left (y_max), Col width-1 = right (y_min)
                X = self.config.x_max - i * self.config.resolution
                Y = self.config.y_max - j * self.config.resolution

                # Project to camera image
                u, v = self._project_ground_point(X, Y)
                map_x[i, j] = u
                map_y[i, j] = v

        return map_x, map_y

    def _project_ground_point(self, X: float, Y: float) -> Tuple[float, float]:
        """
        Project ground point (X, Y, Z=0) to camera image

        Args:
            X: Forward distance (meters)
            Y: Lateral distance (meters)

        Returns:
            u, v: Pixel coordinates in camera image
        """
        # Ground point in vehicle frame
        point_vehicle = np.array([X, Y, 0.0, 1.0], dtype=np.float32)

        # Transform to camera frame
        point_camera = self.T_vehicle_to_camera @ point_vehicle
        Xc, Yc, Zc = point_camera[:3]

        # Check if in front of camera
        if Zc <= 0:
            return (-1, -1)

        # Project to image plane
        u = self.K[0, 0] * (Xc / Zc) + self.K[0, 2]
        v = self.K[1, 1] * (Yc / Zc) + self.K[1, 2]

        return (u, v)

    def process(self, camera_frame: CameraFrame, generate_pointcloud: bool = False) -> BEVRepresentation:
        """
        Transform camera frame to bird's eye view

        Args:
            camera_frame: Input camera frame
            generate_pointcloud: Generate 3D point cloud from depth (default: False)

        Returns:
            BEV representation
        """
        # Apply IPM transformation
        bev_image = cv2.remap(
            camera_frame.image,
            self.map_x,
            self.map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )

        # Optionally create 3D point cloud from depth
        point_cloud = None
        if generate_pointcloud and camera_frame.has_depth():
            point_cloud = self._depth_to_pointcloud(camera_frame)

        return BEVRepresentation(
            timestamp=camera_frame.timestamp,
            bev_image=bev_image,
            point_cloud=point_cloud,
            grid_resolution=self.config.resolution,
            bounds=(self.config.x_min, self.config.x_max,
                   self.config.y_min, self.config.y_max),
            source_camera=camera_frame.camera_id
        )

    def _depth_to_pointcloud(self, camera_frame: CameraFrame) -> np.ndarray:
        """
        Convert depth map to 3D point cloud in vehicle frame

        Args:
            camera_frame: Camera frame with depth data

        Returns:
            Point cloud (N×3) in vehicle frame
        """
        depth = camera_frame.depth
        K_inv = np.linalg.inv(self.K)
        T_camera_to_vehicle = np.linalg.inv(self.T_vehicle_to_camera)

        height, width = depth.shape
        points = []

        # Sample points (downsample for performance)
        step = 4  # Sample every 4th pixel

        for v in range(0, height, step):
            for u in range(0, width, step):
                d = depth[v, u]

                # Skip invalid depth
                if d <= 0 or not np.isfinite(d):
                    continue

                # Back-project to 3D in camera frame
                pixel = np.array([u, v, 1.0])
                ray = K_inv @ pixel
                point_camera = ray * d

                # Transform to vehicle frame
                point_camera_homo = np.append(point_camera, 1.0)
                point_vehicle = T_camera_to_vehicle @ point_camera_homo

                # Only keep points near ground (filter obstacles)
                if abs(point_vehicle[2]) < 0.5:  # Within 50cm of ground
                    points.append(point_vehicle[:3])

        return np.array(points) if points else np.zeros((0, 3))

    def add_grid(
        self,
        bev_image: np.ndarray,
        spacing: float = 1.0,
        color: Tuple[int, int, int] = (0, 255, 0),
        thickness: int = 1
    ) -> np.ndarray:
        """
        Draw metric grid overlay on BEV image

        Args:
            bev_image: BEV image
            spacing: Grid spacing in meters
            color: Line color (B, G, R)
            thickness: Line thickness

        Returns:
            Image with grid overlay
        """
        output = bev_image.copy()
        width, height = self.config.get_size()

        # Vertical lines (constant Y)
        Y = self.config.y_min
        while Y <= self.config.y_max:
            j = int((Y - self.config.y_min) / self.config.resolution)
            if 0 <= j < width:
                cv2.line(output, (j, 0), (j, height), color, thickness)
            Y += spacing

        # Horizontal lines (constant X)
        X = self.config.x_min
        while X <= self.config.x_max:
            # Convention: top = far, bottom = near
            i = int((self.config.x_max - X) / self.config.resolution)
            if 0 <= i < height:
                cv2.line(output, (0, i), (width, i), color, thickness)
            X += spacing

        return output

    def update_pose(self, new_pose: CameraPose):
        """
        Update camera pose and rebuild transformation
        Useful for live calibration

        Args:
            new_pose: New camera pose
        """
        self.pose = new_pose
        self.T_vehicle_to_camera = self._build_transform_matrix()
        print(f"[IPMProcessor] Rebuilding mapping for new pose...")
        self.map_x, self.map_y = self._precompute_mapping()
        print(f"[IPMProcessor] ✓ Updated")
