"""
Camera Adapter
Converts camera_interface.py camera output to standard perception data types

Bridges the gap between camera hardware abstraction and perception pipeline
"""

import time
from typing import Optional
import numpy as np
from .core_types import CameraIntrinsics, CameraFrame


class CameraAdapter:
    """
    Adapts camera_interface.py cameras to perception pipeline format

    Converts camera intrinsics and frames to standardized data structures
    Works with both RGBCamera and RGBDCamera
    """

    @staticmethod
    def convert_intrinsics(camera) -> CameraIntrinsics:
        """
        Convert camera_interface intrinsics to standard format

        Args:
            camera: RGBCamera or RGBDCamera instance

        Returns:
            CameraIntrinsics in standard format
        """
        intrinsics_dict = camera.get_intrinsics()
        width, height = camera.get_resolution()

        # Handle different formats
        if isinstance(intrinsics_dict, dict):
            # RGBDCamera format: {'color': {...}, 'depth': {...}}
            if 'color' in intrinsics_dict:
                color_params = intrinsics_dict['color']
                return CameraIntrinsics(
                    fx=color_params['fx'],
                    fy=color_params['fy'],
                    cx=color_params['cx'],
                    cy=color_params['cy'],
                    width=width,
                    height=height
                )
            else:
                # RGBCamera format: {'fx', 'fy', 'cx', 'cy', ...}
                return CameraIntrinsics(
                    fx=intrinsics_dict['fx'],
                    fy=intrinsics_dict['fy'],
                    cx=intrinsics_dict['cx'],
                    cy=intrinsics_dict['cy'],
                    width=width,
                    height=height
                )
        else:
            raise ValueError(f"Unknown intrinsics format: {type(intrinsics_dict)}")

    @staticmethod
    def get_frame_rgb(camera, camera_id: str = "camera") -> Optional[CameraFrame]:
        """
        Get frame from RGBCamera as standardized CameraFrame

        Args:
            camera: RGBCamera instance
            camera_id: Identifier for this camera

        Returns:
            CameraFrame or None if no frame available
        """
        frame = camera.get_frame()

        if frame is None:
            return None

        intrinsics = CameraAdapter.convert_intrinsics(camera)

        return CameraFrame(
            timestamp=time.time(),
            image=frame,  # BGR format from camera_interface
            depth=None,
            intrinsics=intrinsics,
            camera_id=camera_id
        )

    @staticmethod
    def get_frame_rgbd(camera, camera_id: str = "camera") -> Optional[CameraFrame]:
        """
        Get frame from RGBDCamera as standardized CameraFrame

        Args:
            camera: RGBDCamera instance
            camera_id: Identifier for this camera

        Returns:
            CameraFrame or None if no frame available
        """
        color = camera.get_color_frame()
        depth = camera.get_depth_frame()

        if color is None or depth is None:
            return None

        intrinsics = CameraAdapter.convert_intrinsics(camera)

        # Convert depth from millimeters to meters if needed
        if depth.dtype == np.uint16:
            depth_meters = depth.astype(np.float32) / 1000.0
        else:
            depth_meters = depth.astype(np.float32)

        return CameraFrame(
            timestamp=time.time(),
            image=color,
            depth=depth_meters,
            intrinsics=intrinsics,
            camera_id=camera_id
        )

    @staticmethod
    def get_frame_auto(camera, camera_id: str = "camera") -> Optional[CameraFrame]:
        """
        Automatically detect camera type and get frame

        Args:
            camera: RGBCamera or RGBDCamera instance
            camera_id: Identifier for this camera

        Returns:
            CameraFrame or None if no frame available
        """
        # Check if camera has RGB-D capability
        if hasattr(camera, 'get_color_frame') and hasattr(camera, 'get_depth_frame'):
            return CameraAdapter.get_frame_rgbd(camera, camera_id)
        else:
            return CameraAdapter.get_frame_rgb(camera, camera_id)
