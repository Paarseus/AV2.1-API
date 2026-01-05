===============
Sensor Mounting
===============

Sensor mounting positions and alignment for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

Accurate sensor mounting and calibration is critical for:

- Correct coordinate transformations
- Accurate obstacle localization
- Proper sensor fusion
- Safe autonomous operation

All sensor positions are defined relative to the **vehicle frame origin** (rear axle center at ground level).

Sensor Positions Summary
========================

.. code-block:: text

                         TOP VIEW
         ┌─────────────────────────────────────┐
         │             ROOF                    │
         │                                     │
         │        ┌─────────────┐              │
         │        │   LIDAR     │ ◄── Velodyne │
         │        │  (0.3, 0, 1.8)             │
         │        └─────────────┘              │
         │                                     │
         │        ┌─────────────┐              │
         │        │   GPS/IMU   │ ◄── Xsens    │
         │        │  (0.0, 0, 1.7)             │
         │        └─────────────┘              │
         │                                     │
         ├─────────────────────────────────────┤
         │                                     │
         │  ┌───────────────────────────────┐  │
         │  │        WINDSHIELD             │  │
         │  │                               │  │
         │  │      ┌───────┐                │  │
         │  │      │CAMERA │ ◄── RealSense  │  │
         │  │      │(1.5, 0, 1.2)           │  │
         │  │      └───────┘                │  │
         │  │                               │  │
         │  └───────────────────────────────┘  │
         │                                     │
         │                                     │
         │  ┌───────────────────────────────┐  │
         │  │           HOOD                │  │
         │  └───────────────────────────────┘  │
         │                                     │
         │  X ◄── Vehicle Frame Origin         │
         │       (Rear Axle Center)            │
         │                                     │
         └─────────────────────────────────────┘
                        FRONT

Position Format: (X, Y, Z) in meters
X = forward, Y = left, Z = up

Velodyne VLP-16 (LIDAR)
=======================

Mounting Position
-----------------

+-------------+----------------+------------------+
| Axis        | Value          | Notes            |
+=============+================+==================+
| X (forward) | 0.3 m          | Slightly forward |
+-------------+----------------+------------------+
| Y (left)    | 0.0 m          | Centered         |
+-------------+----------------+------------------+
| Z (up)      | 1.8 m          | Roof mounted     |
+-------------+----------------+------------------+

Orientation
-----------

+-------------+----------------+------------------+
| Angle       | Value          | Notes            |
+=============+================+==================+
| Roll        | 0°             | Level            |
+-------------+----------------+------------------+
| Pitch       | 0°             | Horizontal       |
+-------------+----------------+------------------+
| Yaw         | 0°             | X-axis forward   |
+-------------+----------------+------------------+

Mounting Requirements
---------------------

1. **Level**: Use bubble level to ensure horizontal
2. **Clear FOV**: No obstructions in 360° horizontal plane
3. **Vibration Isolation**: Use rubber mounts to reduce vibration
4. **Cable Routing**: Protect Ethernet and power cables

Field of View Clearance
-----------------------

The LIDAR has ±15° vertical FOV. Ensure no obstructions:

.. code-block:: text

   At 1.8m height with ±15° vertical FOV:

   Upper limit: tan(15°) × distance
   Lower limit: tan(-15°) × distance

   At 10m range:
   - Upper ray: 1.8 + 2.68 = 4.48m above ground
   - Lower ray: 1.8 - 2.68 = -0.88m (below ground = hits ground at ~6.7m)

Xsens MTi-630 (GPS/IMU)
=======================

Mounting Position
-----------------

+-------------+----------------+------------------+
| Axis        | Value          | Notes            |
+=============+================+==================+
| X (forward) | 0.0 m          | At rear axle     |
+-------------+----------------+------------------+
| Y (left)    | 0.0 m          | Centered         |
+-------------+----------------+------------------+
| Z (up)      | 1.7 m          | Roof mounted     |
+-------------+----------------+------------------+

.. note::

   Mounting at the rear axle center simplifies coordinate transformations since the vehicle frame origin is defined here.

Orientation
-----------

+-------------+----------------+------------------+
| Angle       | Value          | Notes            |
+=============+================+==================+
| Roll        | 0°             | Level            |
+-------------+----------------+------------------+
| Pitch       | 0°             | Horizontal       |
+-------------+----------------+------------------+
| Yaw         | 0°             | X-axis forward   |
+-------------+----------------+------------------+

Mounting Requirements
---------------------

1. **Rigid Mount**: No flexing or vibration
2. **Alignment**: X-axis must point exactly forward
3. **Level**: Critical for accurate orientation
4. **GPS Antenna**: Clear sky view required

GPS Antenna Placement
---------------------

If using external GPS antenna:

- Mount on highest point of vehicle
- Metal ground plane (roof) improves reception
- Keep away from other antennas
- Minimum 10cm from edges

Intel RealSense D435 (Camera)
=============================

Mounting Position
-----------------

+-------------+----------------+------------------+
| Axis        | Value          | Notes            |
+=============+================+==================+
| X (forward) | 1.5 m          | Front of vehicle |
+-------------+----------------+------------------+
| Y (left)    | 0.0 m          | Centered         |
+-------------+----------------+------------------+
| Z (up)      | 1.2 m          | Windshield level |
+-------------+----------------+------------------+

Orientation
-----------

+-------------+----------------+------------------+
| Angle       | Value          | Notes            |
+=============+================+==================+
| Roll        | 0°             | Level            |
+-------------+----------------+------------------+
| Pitch       | -5° to -15°    | Slight downward  |
+-------------+----------------+------------------+
| Yaw         | 0°             | Forward facing   |
+-------------+----------------+------------------+

.. note::

   A slight downward pitch (5-15°) helps capture the road surface while still seeing distant obstacles.

Mounting Requirements
---------------------

1. **Stable Mount**: Minimize vibration
2. **Clean View**: No reflections from windshield
3. **Avoid Sun**: Direct sunlight can blind IR sensors
4. **USB Cable**: High-quality USB 3.0, < 3m length

Calibration
===========

Extrinsic Calibration
---------------------

Sensor-to-vehicle transformations are defined in the configuration:

.. code-block:: python

   # Example: LIDAR to vehicle transform
   lidar_to_vehicle = {
       'translation': [0.3, 0.0, 1.8],  # [x, y, z] meters
       'rotation': [0, 0, 0]             # [roll, pitch, yaw] degrees
   }

   # Example: Camera to vehicle transform
   camera_to_vehicle = {
       'translation': [1.5, 0.0, 1.2],
       'rotation': [0, -10, 0]  # 10° down pitch
   }

Calibration Procedure
---------------------

1. **Measure Physical Positions**:
   - Use tape measure from rear axle center
   - Record X, Y, Z for each sensor

2. **Verify Alignment**:
   - Use laser level for roll/pitch
   - Use reference line for yaw

3. **Fine-Tune with Data**:
   - Collect sample data
   - Verify point cloud alignment
   - Adjust offsets as needed

Transformation Matrices
=======================

The full transformation from sensor frame to vehicle frame:

.. code-block:: python

   import numpy as np

   def sensor_to_vehicle_transform(translation, rotation_deg):
       """Create 4x4 homogeneous transformation matrix."""
       roll, pitch, yaw = np.radians(rotation_deg)

       # Rotation matrices
       Rx = rotation_matrix_x(roll)
       Ry = rotation_matrix_y(pitch)
       Rz = rotation_matrix_z(yaw)

       R = Rz @ Ry @ Rx

       T = np.eye(4)
       T[:3, :3] = R
       T[:3, 3] = translation

       return T

See :doc:`coordinate-systems` for detailed coordinate frame definitions.
