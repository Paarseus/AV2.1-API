"""
Sensor Hardware Interfaces
Provides low-level hardware interfaces for all sensors

Available Interfaces:
    - SensorInterface: Base class for all sensors
    - CameraInterface, RGBCamera, RGBDCamera: Camera sensors
    - VelodyneLIDAR, LidarState: LIDAR sensors

Hardware-specific sensors (import explicitly if needed):
    - from sensors.xsens_receiver import XsensReceiver
"""

from .sensor_interface import SensorInterface
from .camera_interface import CameraInterface, RGBCamera, RGBDCamera
from .lidar_interface import VelodyneLIDAR, LidarState

__all__ = [
    'SensorInterface',
    'CameraInterface',
    'RGBCamera',
    'RGBDCamera',
    'VelodyneLIDAR',
    'LidarState',
]
