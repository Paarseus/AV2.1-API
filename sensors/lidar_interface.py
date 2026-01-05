"""
LIDAR Interface for Autonomous Vehicle Perception System
Provides abstract base class and implementations for LIDAR sensors

Classes:
    LIDARInterface: Abstract base class for LIDAR sensors
    VelodyneLIDAR: Velodyne Puck/VLP-16 LIDAR with UDP streaming
"""

import numpy as np
import socket
import threading
import time
import logging
from typing import Optional, Dict, Any, Tuple
from abc import abstractmethod

from .sensor_interface import SensorInterface

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    import velodyne_decoder as vd
    HAS_VELODYNE_DECODER = True
except ImportError:
    HAS_VELODYNE_DECODER = False
    logger.warning("velodyne_decoder not installed - pip install velodyne-decoder")


class LIDARInterface(SensorInterface):
    """
    Abstract base class for all LIDAR sensors
    Inherits lifecycle from SensorInterface, adds LIDAR-specific methods
    """

    def __init__(self, sensor_id: str = "lidar"):
        super().__init__(sensor_id)

        # LIDAR-specific data storage
        self._scan_lock = threading.Lock()
        self._latest_scan = None
        self._scan_timestamp = 0.0

    def get_scan(self) -> Optional[np.ndarray]:
        """
        Get latest LIDAR scan (thread-safe)
        Returns None if no scan available
        """
        with self._scan_lock:
            return self._latest_scan.copy() if self._latest_scan is not None else None

    def get_scan_timestamp(self) -> float:
        """Get timestamp of latest scan (seconds since epoch)"""
        with self._scan_lock:
            return self._scan_timestamp

    @abstractmethod
    def get_resolution(self) -> Tuple[int, int]:
        """Get scan resolution (horizontal_points, vertical_rings)"""
        pass

    @abstractmethod
    def get_intrinsics(self) -> Dict[str, Any]:
        """Get LIDAR specifications and calibration parameters"""
        pass


class VelodyneLIDAR(LIDARInterface):
    """
    Velodyne LIDAR implementation
    Provides real-time LIDAR scans via UDP streaming
    """

    def __init__(self, port: int = 2368, config=None):
        """
        Initialize Velodyne LIDAR

        Args:
            port: UDP port for LIDAR data (default: 2368)
            config: velodyne_decoder.Config object (optional)
        """
        if not HAS_VELODYNE_DECODER:
            raise ImportError("velodyne_decoder required: pip install velodyne-decoder")

        super().__init__(sensor_id=f"velodyne_{port}")

        self.port = port
        self.config = config if config else vd.Config()

        self._socket = None
        self._stream_decoder = None

    def connect(self) -> bool:
        """
        Initialize LIDAR connection and setup UDP socket
        Returns True if successful, False otherwise
        """
        if self._connected:
            logger.warning(f"{self.sensor_id}: Already connected")
            return True

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 26214400)
            self._socket.bind(('', self.port))
            self._socket.settimeout(0.1)

            self._stream_decoder = vd.StreamDecoder(self.config)
            self._connected = True
            logger.info(f"{self.sensor_id}: Connected on port {self.port}")
            return True

        except Exception as e:
            logger.error(f"{self.sensor_id}: Connection failed - {e}")
            if self._socket:
                self._socket.close()
                self._socket = None
            return False

    def _capture_loop(self):
        """Background thread for continuous UDP packet capture and decoding"""
        while self._running:
            try:
                data, addr = self._socket.recvfrom(2000)
                result = self._stream_decoder.decode(time.time(), data, False)

                if result is not None:
                    timestamp, points = result
                    if points is not None and len(points) > 0:
                        with self._scan_lock:
                            self._latest_scan = points
                            self._scan_timestamp = float(timestamp) if hasattr(timestamp, '__float__') else time.time()

                        # Notify callbacks
                        self._notify_callbacks(points, timestamp)

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    logger.error(f"{self.sensor_id}: Capture error - {e}")
                break

    def get_intrinsics(self) -> Dict[str, Any]:
        """Get LIDAR specifications"""
        return {
            'model': 'Velodyne VLP-16',
            'vertical_fov': (-15.0, 15.0),
            'horizontal_fov': (0.0, 360.0),
            'num_lasers': 16,
            'max_range': 100.0,
            'port': self.port
        }

    def get_resolution(self) -> Tuple[int, int]:
        """Get scan resolution (horizontal_points, vertical_rings)"""
        return (1800, 16)

    def disconnect(self):
        """Cleanup and close socket"""
        self.stop()
        if self._connected:
            if self._socket:
                self._socket.close()
                self._socket = None
            self._stream_decoder = None
            self._connected = False
            logger.info(f"{self.sensor_id}: Disconnected")


class LidarState:
    """
    Simplified thread-safe LIDAR state wrapper

    Provides a clean interface for main/visualization threads to access LIDAR data
    without needing to know about sensor lifecycle details.

    Usage:
        lidar = VelodyneLIDAR()
        lidar.connect()
        lidar.start()

        lidar_state = LidarState(lidar)

        # In visualization thread
        points = lidar_state.get_points()
        if points is not None:
            visualize(points)
    """

    def __init__(self, lidar: LIDARInterface):
        """
        Initialize LidarState wrapper

        Args:
            lidar: LIDARInterface instance (e.g., VelodyneLIDAR)
        """
        self.lidar = lidar

    def get_points(self) -> Optional[np.ndarray]:
        """
        Get latest LIDAR points as XYZ array

        Returns:
            Array of shape (N, 3) with [x, y, z] or None if no data
        """
        scan = self.lidar.get_scan()
        if scan is None:
            return None

        # Convert structured array to XYZ
        if scan.dtype.names:  # Structured array (x, y, z, intensity, ...)
            xyz = np.column_stack([scan['x'], scan['y'], scan['z']])
        else:  # Already flat array
            xyz = scan[:, :3]

        return xyz

    def get_scan_raw(self) -> Optional[np.ndarray]:
        """Get raw scan with all fields (intensity, time, etc.)"""
        return self.lidar.get_scan()

    def get_timestamp(self) -> float:
        """Get timestamp of latest scan"""
        return self.lidar.get_scan_timestamp()

    def get_age(self) -> float:
        """Get age of latest scan in seconds"""
        return time.time() - self.get_timestamp()

    def is_fresh(self, max_age: float = 1.0) -> bool:
        """Check if data is fresh (age < max_age seconds)"""
        return self.get_age() < max_age


if __name__ == "__main__":
    """Test LIDAR interface"""
    import sys

    print("VELODYNE LIDAR INTERFACE TEST")
    print("-" * 50)

    lidar = VelodyneLIDAR(port=2368)

    if not lidar.connect():
        print("ERROR: Failed to connect")
        sys.exit(1)

    print("Connected:", lidar.get_intrinsics())

    if not lidar.start():
        print("ERROR: Failed to start")
        lidar.disconnect()
        sys.exit(1)

    print("Waiting for data (Ctrl+C to stop)...\n")

    # Test with LidarState wrapper
    lidar_state = LidarState(lidar)

    try:
        scan_count = 0
        while True:
            xyz = lidar_state.get_points()
            if xyz is not None:
                scan_count += 1
                print(f"Scan {scan_count:4d} | Points: {len(xyz):6d} | "
                      f"Age: {lidar_state.get_age():.3f}s | "
                      f"Range: X[{xyz[:,0].min():.1f},{xyz[:,0].max():.1f}] "
                      f"Y[{xyz[:,1].min():.1f},{xyz[:,1].max():.1f}] "
                      f"Z[{xyz[:,2].min():.1f},{xyz[:,2].max():.1f}]")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        lidar.stop()
        lidar.disconnect()
        print("Done")
