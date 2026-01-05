=======
Sensors
=======

Detailed specifications and setup for all sensors on the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Xsens MTi-630 AHRS/GPS
======================

Overview
--------

The Xsens MTi-630 is a high-performance Attitude and Heading Reference System (AHRS) with integrated GNSS receiver, providing precise position, orientation, and velocity data.

.. figure:: ../_static/xsens_mti630.png
   :alt: Xsens MTi-630
   :align: center
   :width: 300px

   Xsens MTi-630 AHRS/GPS Module

Specifications
--------------

**Positioning**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Position Accuracy (RTK) | 1 cm + 1 ppm                   |
+-------------------------+--------------------------------+
| Position Accuracy (GPS) | 2.5 m CEP                      |
+-------------------------+--------------------------------+
| Velocity Accuracy       | 0.05 m/s                       |
+-------------------------+--------------------------------+
| Update Rate             | Up to 400 Hz                   |
+-------------------------+--------------------------------+
| GNSS Constellations     | GPS, GLONASS, Galileo, BeiDou  |
+-------------------------+--------------------------------+

**Orientation**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Roll/Pitch Accuracy     | 0.2°                           |
+-------------------------+--------------------------------+
| Heading Accuracy        | 0.5° (with GNSS)               |
+-------------------------+--------------------------------+
| Angular Rate Range      | ±2000 °/s                      |
+-------------------------+--------------------------------+
| Angular Rate Noise      | 0.01 °/s/√Hz                   |
+-------------------------+--------------------------------+

**Physical**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Dimensions              | 57 × 42 × 23.5 mm              |
+-------------------------+--------------------------------+
| Weight                  | 55 g                           |
+-------------------------+--------------------------------+
| Operating Temperature   | -40°C to +85°C                 |
+-------------------------+--------------------------------+
| Power Consumption       | < 1 W                          |
+-------------------------+--------------------------------+

Connection
----------

- **Interface**: USB or RS-232
- **Default Baud Rate**: 115200
- **Linux Device**: ``/dev/ttyUSB0`` (typically)

Output Data
-----------

The Xsens provides the following data at each update:

.. code-block:: python

   # Position (WGS84)
   latitude: float    # degrees
   longitude: float   # degrees
   altitude: float    # meters above ellipsoid

   # Velocity (ENU frame)
   velocity_east: float   # m/s
   velocity_north: float  # m/s
   velocity_up: float     # m/s

   # Orientation (quaternion)
   q_w, q_x, q_y, q_z: float

   # Orientation (Euler angles)
   roll: float   # degrees
   pitch: float  # degrees
   yaw: float    # degrees (heading)

   # Angular velocity
   gyro_x, gyro_y, gyro_z: float  # rad/s

   # Acceleration
   acc_x, acc_y, acc_z: float  # m/s²

   # Status
   rtk_status: str  # "None", "Float", "Fixed"

RTK Setup
---------

For centimeter-level accuracy, RTK corrections are required:

1. **Base Station**: Set up a local base station or use NTRIP service
2. **NTRIP Client**: Configure to receive RTCM corrections
3. **Convergence**: Wait 30-60 seconds for RTK Fixed status

.. warning::

   RTK requires clear sky view. Performance degrades significantly under trees or near buildings.

Velodyne VLP-16 LIDAR
=====================

Overview
--------

The Velodyne VLP-16 (Puck) is a compact 3D LIDAR sensor providing 360° coverage with 16 channels.

.. figure:: ../_static/velodyne_vlp16.png
   :alt: Velodyne VLP-16
   :align: center
   :width: 250px

   Velodyne VLP-16 LIDAR

Specifications
--------------

**Performance**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Channels                | 16                             |
+-------------------------+--------------------------------+
| Range                   | 100 m                          |
+-------------------------+--------------------------------+
| Accuracy                | ±3 cm                          |
+-------------------------+--------------------------------+
| Points per Second       | ~300,000                       |
+-------------------------+--------------------------------+
| Rotation Rate           | 5-20 Hz (configurable)         |
+-------------------------+--------------------------------+
| Horizontal FOV          | 360°                           |
+-------------------------+--------------------------------+
| Vertical FOV            | 30° (+15° to -15°)             |
+-------------------------+--------------------------------+
| Angular Resolution (H)  | 0.1° - 0.4°                    |
+-------------------------+--------------------------------+
| Angular Resolution (V)  | 2°                             |
+-------------------------+--------------------------------+

**Physical**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Dimensions              | 103.3 mm (H) × 71.7 mm (D)     |
+-------------------------+--------------------------------+
| Weight                  | 830 g                          |
+-------------------------+--------------------------------+
| Power Consumption       | 8 W                            |
+-------------------------+--------------------------------+
| Operating Temperature   | -10°C to +60°C                 |
+-------------------------+--------------------------------+

Connection
----------

- **Interface**: Ethernet (100 Mbps)
- **Protocol**: UDP
- **Default IP**: 192.168.1.201
- **Data Port**: 2368
- **Position Port**: 8308

Network Configuration
---------------------

Configure the host computer's network interface:

.. code-block:: bash

   # Set static IP on the interface connected to LIDAR
   sudo ip addr add 192.168.1.100/24 dev eth0

   # Verify connectivity
   ping 192.168.1.201

Data Format
-----------

Each UDP packet contains:

- **Data Packets** (port 2368): Point cloud data
- **Position Packets** (port 8308): GPS timestamp and orientation

Point cloud output (per point):

.. code-block:: python

   x: float      # meters (forward)
   y: float      # meters (left)
   z: float      # meters (up)
   intensity: int  # 0-255
   ring: int     # channel number (0-15)

Intel RealSense D435
====================

Overview
--------

The Intel RealSense D435 is a stereo depth camera providing RGB and aligned depth images.

Specifications
--------------

**RGB Camera**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Resolution              | Up to 1920 × 1080              |
+-------------------------+--------------------------------+
| Frame Rate              | 30 fps @ 1080p, 60 fps @ 720p  |
+-------------------------+--------------------------------+
| Field of View           | 69° × 42°                      |
+-------------------------+--------------------------------+
| Shutter                 | Rolling shutter                |
+-------------------------+--------------------------------+

**Depth Camera**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Resolution              | Up to 1280 × 720               |
+-------------------------+--------------------------------+
| Frame Rate              | Up to 90 fps                   |
+-------------------------+--------------------------------+
| Range                   | 0.2 m - 10 m                   |
+-------------------------+--------------------------------+
| Field of View           | 87° × 58°                      |
+-------------------------+--------------------------------+
| Baseline                | 50 mm                          |
+-------------------------+--------------------------------+

**Physical**

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Dimensions              | 90 × 25 × 25 mm                |
+-------------------------+--------------------------------+
| Weight                  | 72 g                           |
+-------------------------+--------------------------------+
| Interface               | USB 3.0 Type-C                 |
+-------------------------+--------------------------------+

Connection
----------

.. code-block:: bash

   # Verify camera is detected
   rs-enumerate-devices

   # Test camera stream
   realsense-viewer

USB Cameras
===========

Overview
--------

Additional USB cameras can be used for supplementary visual input.

Supported Types
---------------

- Standard USB webcams (UVC compliant)
- Industrial USB cameras
- GMSL cameras (with adapter)

Configuration
-------------

.. code-block:: bash

   # List available cameras
   v4l2-ctl --list-devices

   # Check supported formats
   v4l2-ctl -d /dev/video0 --list-formats-ext

Typical settings:

.. code-block:: python

   resolution = (640, 480)  # or (1280, 720)
   frame_rate = 30
   format = "MJPG"  # or "YUYV"

Sensor Placement
================

See :doc:`../mechanical/sensor-mounting` for detailed mounting positions and calibration.

.. list-table::
   :header-rows: 1

   * - Sensor
     - Location
     - Orientation
   * - Xsens MTi-630
     - Vehicle center, roof
     - X forward, Y left, Z up
   * - Velodyne VLP-16
     - Roof, center-front
     - Level, X forward
   * - RealSense D435
     - Front bumper/windshield
     - Forward facing, slight down pitch
   * - USB Cameras
     - Variable
     - Application dependent
