=======
Sensors
=======

Software interfaces for UTM Navigator sensors.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

Each sensor has a dedicated interface class that handles:

- Connection management
- Data acquisition
- Thread-safe access
- Error handling

Sensor Interface Base Class
===========================

All sensors implement this abstract interface:

.. code-block:: python

   from abc import ABC, abstractmethod

   class SensorInterface(ABC):
       @abstractmethod
       def connect(self) -> bool:
           """Connect to sensor. Returns True on success."""
           pass

       @abstractmethod
       def disconnect(self) -> None:
           """Disconnect from sensor."""
           pass

       @abstractmethod
       def is_connected(self) -> bool:
           """Check if sensor is connected."""
           pass

       @abstractmethod
       def get_data(self) -> Any:
           """Get latest sensor data."""
           pass

Xsens GPS/IMU Interface
=======================

The ``XsensReceiver`` class provides position, orientation, and velocity data.

Initialization
--------------

.. code-block:: python

   from xsens_class import XsensReceiver

   # Auto-detect Xsens device
   gps = XsensReceiver()

   # Or specify port
   gps = XsensReceiver(port='/dev/ttyUSB0')

Available Methods
-----------------

.. code-block:: python

   # Position (WGS84)
   lat, lon, alt = gps.get_position()

   # Position (UTM meters)
   x, y = gps.get_utm_position()

   # Heading (radians from North, clockwise positive)
   heading = gps.get_heading()

   # Velocity (m/s)
   speed = gps.get_speed()
   vx, vy, vz = gps.get_velocity_enu()

   # Orientation (Euler angles in degrees)
   roll, pitch, yaw = gps.get_orientation()

   # Orientation (quaternion)
   qw, qx, qy, qz = gps.get_quaternion()

   # Angular velocity (rad/s)
   wx, wy, wz = gps.get_angular_velocity()

   # Acceleration (m/s²)
   ax, ay, az = gps.get_acceleration()

   # RTK status
   status = gps.get_rtk_status()  # "None", "Float", "Fixed"

Threading Model
---------------

The XsensReceiver runs a background thread for packet retrieval:

.. code-block:: python

   class XsensReceiver:
       def __init__(self):
           self._lock = threading.Lock()
           self._data_buffer = deque(maxlen=5)
           self._running = True
           self._thread = threading.Thread(target=self._receive_loop)
           self._thread.start()

       def _receive_loop(self):
           while self._running:
               packet = self.device.read_packet()
               with self._lock:
                   self._data_buffer.append(packet)

       def get_position(self):
           with self._lock:
               if self._data_buffer:
                   return self._data_buffer[-1].position
               return None

RTK Wait
--------

Wait for RTK fix before starting navigation:

.. code-block:: python

   def wait_for_rtk(gps, timeout=60):
       start = time.time()
       while time.time() - start < timeout:
           status = gps.get_rtk_status()
           if status == "Fixed":
               return True
           time.sleep(1)
       return False

Velodyne LIDAR Interface
========================

The ``VelodyneLIDAR`` class captures 3D point clouds.

Initialization
--------------

.. code-block:: python

   from sensors.lidar_interface import VelodyneLIDAR

   lidar = VelodyneLIDAR(
       host='192.168.1.201',  # LIDAR IP
       port=2368              # Data port
   )
   lidar.connect()

Getting Point Cloud
-------------------

.. code-block:: python

   # Get latest scan (Nx3 numpy array)
   points = lidar.get_scan()

   # Points format: [[x, y, z], [x, y, z], ...]
   # Coordinates in LIDAR frame (meters)

   # Get points with intensity
   points, intensities = lidar.get_scan_with_intensity()

Implementation
--------------

.. code-block:: python

   class VelodyneLIDAR(SensorInterface):
       def __init__(self, host='192.168.1.201', port=2368):
           self.host = host
           self.port = port
           self._lock = threading.Lock()
           self._current_scan = None

       def connect(self):
           self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
           self.socket.bind(('', self.port))
           self._running = True
           self._thread = threading.Thread(target=self._receive_loop)
           self._thread.start()
           return True

       def _receive_loop(self):
           while self._running:
               data, _ = self.socket.recvfrom(1206)
               points = velodyne_decoder.decode(data)
               with self._lock:
                   self._current_scan = points

       def get_scan(self):
           with self._lock:
               return self._current_scan.copy() if self._current_scan else None

Camera Interfaces
=================

RGB Camera
----------

.. code-block:: python

   from sensors.camera_interface import RGBCamera

   camera = RGBCamera(device_id=0)  # /dev/video0
   camera.connect()

   # Get frame
   frame = camera.get_frame()  # numpy array (H, W, 3) BGR

   # Get with timestamp
   frame, timestamp = camera.get_frame_with_timestamp()

RGB-D Camera (RealSense)
------------------------

.. code-block:: python

   from sensors.camera_interface import RGBDCamera

   camera = RGBDCamera()
   camera.connect()

   # Get RGB frame
   color = camera.get_color_frame()

   # Get depth frame
   depth = camera.get_depth_frame()  # (H, W) uint16, mm

   # Get aligned frames
   color, depth = camera.get_aligned_frames()

   # Get intrinsics
   intrinsics = camera.get_intrinsics()
   # Returns: fx, fy, cx, cy

Camera Frame Dataclass
----------------------

.. code-block:: python

   @dataclass
   class CameraFrame:
       image: np.ndarray          # RGB image
       depth: Optional[np.ndarray]  # Depth map (if available)
       timestamp: float
       intrinsics: CameraIntrinsics

   @dataclass
   class CameraIntrinsics:
       fx: float  # Focal length x
       fy: float  # Focal length y
       cx: float  # Principal point x
       cy: float  # Principal point y
       width: int
       height: int

Camera Adapter
--------------

Standardizes output from different cameras:

.. code-block:: python

   from perception.camera_adapter import CameraAdapter

   adapter = CameraAdapter(camera)
   frame = adapter.get_standard_frame()
   # Returns CameraFrame with consistent format

Sensor Fusion Example
=====================

Combining data from multiple sensors:

.. code-block:: python

   class SensorManager:
       def __init__(self):
           self.gps = XsensReceiver()
           self.lidar = VelodyneLIDAR()
           self.camera = RGBDCamera()

       def connect_all(self):
           self.gps.connect()
           self.lidar.connect()
           self.camera.connect()

       def get_synchronized_data(self):
           """Get data from all sensors."""
           timestamp = time.time()

           position = self.gps.get_position()
           heading = self.gps.get_heading()
           points = self.lidar.get_scan()
           frame = self.camera.get_frame()

           return {
               'timestamp': timestamp,
               'position': position,
               'heading': heading,
               'lidar': points,
               'camera': frame
           }
