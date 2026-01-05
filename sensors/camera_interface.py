"""
Camera Interface for Autonomous Vehicle Perception System
Provides abstract base class and implementations for RGB and RGB-D cameras.

Classes:
    CameraInterface: Abstract base class with lifecycle methods (connect, start, stop, disconnect)
    RGBCamera: Standard RGB camera using OpenCV (USB/CSI cameras)
    RGBDCamera: Intel RealSense RGB-D camera with aligned color+depth frames

Key Features:
    - Factory-calibrated intrinsics from RealSense (no manual calibration needed)
    - Estimated intrinsics for USB cameras (or load from calibration file)
    - Thread-safe frame capture with non-blocking getters
    - Aligned depth-to-color for accurate RGB-D correspondence

Usage Example:
    # RGB Camera
    rgb_cam = RGBCamera(camera_id=0)
    rgb_cam.connect()
    rgb_cam.start()
    frame = rgb_cam.get_frame()  # Returns BGR numpy array (OpenCV convention)
    intrinsics = rgb_cam.get_intrinsics()  # For IPM/calibration

    # RGB-D Camera
    rgbd_cam = RGBDCamera(width=1280, height=720, fps=30)
    rgbd_cam.connect()
    rgbd_cam.start()
    color = rgbd_cam.get_color_frame()  # BGR array (OpenCV convention)
    depth = rgbd_cam.get_depth_frame()  # Aligned depth in mm
    intrinsics = rgbd_cam.get_intrinsics()  # Color + depth intrinsics
"""

from abc import abstractmethod
import numpy as np
import threading
import time
from typing import Optional, Tuple, Dict, Any
import logging

from .sensor_interface import SensorInterface

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CameraInterface(SensorInterface):
    """
    Abstract base class for all camera sensors
    Inherits lifecycle from SensorInterface, adds camera-specific methods
    """

    def __init__(self, sensor_id: str = "camera"):
        super().__init__(sensor_id)

        # Camera-specific data storage
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_timestamp = 0.0

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get latest captured frame (thread-safe)
        Returns None if no frame available
        """
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def get_frame_timestamp(self) -> float:
        """Get timestamp of latest frame (seconds since epoch)"""
        with self._frame_lock:
            return self._frame_timestamp

    @abstractmethod
    def get_resolution(self) -> Tuple[int, int]:
        """Get camera resolution as (width, height)"""
        pass

    @abstractmethod
    def get_intrinsics(self) -> Optional[Dict[str, Any]]:
        """
        Get camera intrinsic calibration parameters

        Returns dict with keys:
            'fx', 'fy': Focal length (pixels)
            'cx', 'cy': Principal point (pixels)
            'distortion': [k1, k2, p1, p2, k3] distortion coefficients
            'camera_matrix': 3x3 numpy array
            'dist_coeffs': 1x5 numpy array

        Returns None if intrinsics not available
        """
        pass


class RGBCamera(CameraInterface):
    """
    RGB camera implementation using OpenCV
    Captures standard color frames from USB/CSI cameras
    """

    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480, fps: int = 30,
                 calibration_file: Optional[str] = None):
        """
        Initialize RGB camera

        Args:
            camera_id: Camera device ID (default 0)
            width: Frame width in pixels (default 640)
            height: Frame height in pixels (default 480)
            fps: Frames per second (default 30)
            calibration_file: Optional path to calibration file (numpy .npz format)
        """
        super().__init__(sensor_id=f"rgb_camera_{camera_id}")
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.calibration_file = calibration_file
        self._capture = None
        self._intrinsics = None

    def connect(self) -> bool:
        """Connect to RGB camera via OpenCV."""
        try:
            import cv2
            self._cv2 = cv2

            logger.info(f"RGBCamera: Connecting to camera {self.camera_id}...")
            self._capture = cv2.VideoCapture(self.camera_id)

            if not self._capture.isOpened():
                logger.error(f"RGBCamera: Failed to open camera {self.camera_id}")
                return False

            # Configure camera
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)

            # Verify settings
            actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self._capture.get(cv2.CAP_PROP_FPS))

            logger.info(f"RGBCamera: Connected - {actual_width}x{actual_height} @ {actual_fps}fps")

            # Load or estimate intrinsics
            self._load_intrinsics()

            self._connected = True
            return True

        except Exception as e:
            logger.error(f"RGBCamera: Connection failed - {e}")
            return False

    def _load_intrinsics(self):
        """Load camera intrinsics from file or estimate them."""
        # Try loading from calibration file
        if self.calibration_file:
            try:
                calib_data = np.load(self.calibration_file)
                camera_matrix = calib_data['camera_matrix']
                dist_coeffs = calib_data['dist_coeffs']

                self._intrinsics = {
                    'fx': camera_matrix[0, 0],
                    'fy': camera_matrix[1, 1],
                    'cx': camera_matrix[0, 2],
                    'cy': camera_matrix[1, 2],
                    'distortion': dist_coeffs.flatten().tolist(),
                    'camera_matrix': camera_matrix,
                    'dist_coeffs': dist_coeffs
                }
                logger.info(f"RGBCamera: Loaded calibration from {self.calibration_file}")
                return
            except Exception as e:
                logger.warning(f"RGBCamera: Failed to load calibration file - {e}")

        # Estimate intrinsics based on typical webcam FOV (60 degrees horizontal)
        # Formula: fx = width / (2 * tan(fov/2))
        width, height = self.get_resolution()
        fov_deg = 60  # Typical webcam horizontal FOV
        fx = width / (2 * np.tan(np.radians(fov_deg / 2)))
        fy = fx  # Assume square pixels
        cx = width / 2
        cy = height / 2

        camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)

        # Assume no distortion for uncalibrated camera
        dist_coeffs = np.zeros((1, 5), dtype=np.float32)

        self._intrinsics = {
            'fx': fx,
            'fy': fy,
            'cx': cx,
            'cy': cy,
            'distortion': [0, 0, 0, 0, 0],
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs
        }
        logger.warning(f"RGBCamera: Using estimated intrinsics (FOV={fov_deg}°). "
                      "For accurate results, calibrate camera and provide calibration_file.")

    def _capture_loop(self):
        """Continuous frame capture thread."""
        logger.info("RGBCamera: Capture loop started")
        frame_interval = 1.0 / self.fps

        while self._running:
            start_time = time.time()

            ret, frame = self._capture.read()

            if ret:
                # Keep BGR format (OpenCV convention, matches YOLOPv2 etc.)
                timestamp = time.time()

                with self._frame_lock:
                    self._latest_frame = frame
                    self._frame_timestamp = timestamp

                # Notify callbacks
                self._notify_callbacks(frame, timestamp)
            else:
                logger.warning("RGBCamera: Frame read failed")

            # Maintain consistent frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("RGBCamera: Capture loop ended")

    def get_resolution(self) -> Tuple[int, int]:
        """Get camera resolution as (width, height)."""
        if self._capture and self._capture.isOpened():
            width = int(self._capture.get(self._cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self._capture.get(self._cv2.CAP_PROP_FRAME_HEIGHT))
            return (width, height)
        return (self.width, self.height)

    def get_intrinsics(self) -> Optional[Dict[str, Any]]:
        """Get camera intrinsic parameters."""
        return self._intrinsics

    def disconnect(self):
        """Disconnect from camera."""
        self.stop()
        if self._capture:
            self._capture.release()
            self._capture = None
        self._connected = False
        logger.info("RGBCamera: Disconnected")


class RGBDCamera(CameraInterface):
    """
    Intel RealSense RGB-D camera implementation
    Captures aligned color and depth frames for RGB-D perception
    Depth values are in millimeters, aligned to color frame pixels
    """

    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30):
        """
        Initialize RealSense RGB-D camera

        Args:
            width: Frame width in pixels (default 640)
            height: Frame height in pixels (default 480)
            fps: Frames per second (default 30)
        """
        super().__init__(sensor_id="rgbd_camera")
        self.width = width
        self.height = height
        self.fps = fps
        self._pipeline = None
        self._config = None
        self._align = None
        self._intrinsics = None
        self._depth_scale = 1.0

        # Separate storage for color and depth
        self._latest_color_frame = None
        self._latest_depth_frame = None

    def connect(self) -> bool:
        """Connect to RealSense RGB-D camera."""
        try:
            import pyrealsense2 as rs
            self._rs = rs

            logger.info("RGBDCamera: Connecting to RealSense...")

            # Create pipeline and config
            self._pipeline = rs.pipeline()
            self._config = rs.config()

            # Enable both color and depth streams
            # Use BGR8 to match OpenCV convention (YOLOPv2, etc. expect BGR)
            self._config.enable_stream(rs.stream.color, self.width, self.height,
                                      rs.format.bgr8, self.fps)
            self._config.enable_stream(rs.stream.depth, self.width, self.height,
                                      rs.format.z16, self.fps)

            # Start pipeline
            profile = self._pipeline.start(self._config)

            # Create alignment object (align depth to color frame)
            self._align = rs.align(rs.stream.color)

            # Get depth sensor configuration
            depth_sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = depth_sensor.get_depth_scale()

            # Get intrinsics from color stream
            color_stream = profile.get_stream(rs.stream.color)
            color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

            depth_stream = profile.get_stream(rs.stream.depth)
            depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

            # Store intrinsics in standard format
            self._intrinsics = {
                'color': {
                    'fx': color_intrinsics.fx,
                    'fy': color_intrinsics.fy,
                    'cx': color_intrinsics.ppx,
                    'cy': color_intrinsics.ppy,
                    'distortion': color_intrinsics.coeffs,
                    'camera_matrix': np.array([
                        [color_intrinsics.fx, 0, color_intrinsics.ppx],
                        [0, color_intrinsics.fy, color_intrinsics.ppy],
                        [0, 0, 1]
                    ], dtype=np.float32),
                    'dist_coeffs': np.array([color_intrinsics.coeffs], dtype=np.float32)
                },
                'depth': {
                    'fx': depth_intrinsics.fx,
                    'fy': depth_intrinsics.fy,
                    'cx': depth_intrinsics.ppx,
                    'cy': depth_intrinsics.ppy,
                    'distortion': depth_intrinsics.coeffs,
                    'camera_matrix': np.array([
                        [depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                        [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                        [0, 0, 1]
                    ], dtype=np.float32),
                    'dist_coeffs': np.array([depth_intrinsics.coeffs], dtype=np.float32)
                },
                'depth_scale': self._depth_scale
            }

            logger.info(f"RGBDCamera: Connected - {self.width}x{self.height} @ {self.fps}fps")
            logger.info(f"RGBDCamera: Depth scale = {self._depth_scale:.6f} (meters = pixel_value * scale)")
            logger.info(f"RGBDCamera: Color intrinsics - fx={color_intrinsics.fx:.1f}, fy={color_intrinsics.fy:.1f}")

            self._connected = True
            return True

        except Exception as e:
            logger.error(f"RGBDCamera: Connection failed - {e}")
            logger.error("RGBDCamera: Make sure pyrealsense2 is installed: pip install pyrealsense2")
            return False

    def _capture_loop(self):
        """Continuous RGB-D frame capture thread."""
        logger.info("RGBDCamera: Capture loop started")
        frame_interval = 1.0 / self.fps

        while self._running:
            start_time = time.time()

            try:
                # Wait for frames (timeout in ms)
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)

                # Align depth frame to color frame
                aligned_frames = self._align.process(frames)

                # Get aligned frames
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if color_frame and depth_frame:
                    # Convert to numpy arrays
                    color_image = np.asanyarray(color_frame.get_data())  # BGR8 format
                    depth_image = np.asanyarray(depth_frame.get_data())  # uint16, millimeters
                    timestamp = time.time()

                    with self._frame_lock:
                        self._latest_color_frame = color_image
                        self._latest_depth_frame = depth_image
                        self._latest_frame = color_image  # For compatibility with base class
                        self._frame_timestamp = timestamp

                    # Notify callbacks with color frame (for compatibility)
                    self._notify_callbacks(color_image, timestamp)

            except Exception as e:
                if self._running:  # Only log if we're supposed to be running
                    logger.warning(f"RGBDCamera: Frame capture error - {e}")

            # Maintain consistent frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("RGBDCamera: Capture loop ended")

    def get_resolution(self) -> Tuple[int, int]:
        """Get camera resolution as (width, height)."""
        return (self.width, self.height)

    def get_color_frame(self) -> Optional[np.ndarray]:
        """
        Get latest color frame (thread-safe).
        Returns BGR numpy array (height, width, 3) uint8, or None if unavailable.
        """
        with self._frame_lock:
            return self._latest_color_frame.copy() if self._latest_color_frame is not None else None

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """
        Get latest depth frame aligned to color frame (thread-safe).
        Returns depth numpy array (height, width) uint16 in millimeters, or None if unavailable.
        """
        with self._frame_lock:
            return self._latest_depth_frame.copy() if self._latest_depth_frame is not None else None

    def get_intrinsics(self) -> Optional[Dict[str, Any]]:
        """
        Get camera intrinsics for both color and depth cameras.

        Returns dict with keys:
            'color': Color camera intrinsics dict (fx, fy, cx, cy, distortion, camera_matrix, dist_coeffs)
            'depth': Depth camera intrinsics dict (fx, fy, cx, cy, distortion, camera_matrix, dist_coeffs)
            'depth_scale': Conversion factor from depth pixels to meters
        """
        return self._intrinsics

    def disconnect(self):
        """Disconnect from RealSense camera."""
        self.stop()
        if self._pipeline:
            self._pipeline.stop()
            self._pipeline = None
        self._connected = False
        logger.info("RGBDCamera: Disconnected")


# Example usage
if __name__ == "__main__":
    print("Camera Interface Test")
    print("=" * 50)

    # Test RGB Camera
    print("\n1. Testing RGB Camera...")
    rgb_cam = RGBCamera(camera_id=0, width=640, height=480, fps=30)

    if rgb_cam.connect():
        print("   ✓ Connected")

        # Show intrinsics
        intrinsics = rgb_cam.get_intrinsics()
        if intrinsics:
            print(f"   ✓ Intrinsics: fx={intrinsics['fx']:.1f}, fy={intrinsics['fy']:.1f}, "
                  f"cx={intrinsics['cx']:.1f}, cy={intrinsics['cy']:.1f}")

        rgb_cam.start()
        print("   ✓ Started")

        # Capture a few frames
        time.sleep(1.0)  # Let camera warm up

        for i in range(5):
            frame = rgb_cam.get_frame()
            if frame is not None:
                print(f"   ✓ Frame {i+1}: shape={frame.shape}, dtype={frame.dtype}")
            time.sleep(0.2)

        rgb_cam.disconnect()
        print("   ✓ Disconnected")
    else:
        print("   ✗ Failed to connect")

    # Test RGB-D Camera
    print("\n2. Testing RGB-D Camera (RealSense)...")
    rgbd_cam = RGBDCamera(width=1280, height=720, fps=30)

    if rgbd_cam.connect():
        print("   ✓ Connected")

        # Show intrinsics
        intrinsics = rgbd_cam.get_intrinsics()
        if intrinsics:
            print(f"   ✓ Color intrinsics: fx={intrinsics['color']['fx']:.1f}, "
                  f"fy={intrinsics['color']['fy']:.1f}")
            print(f"   ✓ Depth scale: {intrinsics['depth_scale']:.6f}")

        rgbd_cam.start()
        print("   ✓ Started")

        # Capture a few frames
        time.sleep(1.0)  # Let camera warm up

        for i in range(5):
            color_frame = rgbd_cam.get_color_frame()
            depth_frame = rgbd_cam.get_depth_frame()

            if color_frame is not None and depth_frame is not None:
                min_depth = np.min(depth_frame[depth_frame > 0])  # Ignore zero values
                max_depth = np.max(depth_frame)
                print(f"   ✓ Frame {i+1}: color={color_frame.shape}, "
                      f"depth={depth_frame.shape}, depth_range={min_depth}-{max_depth}mm")
            time.sleep(0.2)

        rgbd_cam.disconnect()
        print("   ✓ Disconnected")
    else:
        print("   ✗ Failed to connect (RealSense may not be connected)")

    print("\n" + "=" * 50)
    print("Test complete")
