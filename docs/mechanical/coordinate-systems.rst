==================
Coordinate Systems
==================

Coordinate frame definitions for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The UTM Navigator uses several coordinate systems. Understanding these frames and their transformations is essential for correct operation.

Frame Summary
=============

+------------------+-------------------+-------------------+-------------------+
| Frame            | X-axis            | Y-axis            | Z-axis            |
+==================+===================+===================+===================+
| Vehicle          | Forward           | Left              | Up                |
+------------------+-------------------+-------------------+-------------------+
| World (UTM)      | East              | North             | Up                |
+------------------+-------------------+-------------------+-------------------+
| LIDAR            | Forward           | Left              | Up                |
+------------------+-------------------+-------------------+-------------------+
| Camera           | Right             | Down              | Forward           |
+------------------+-------------------+-------------------+-------------------+
| Grid             | Forward           | Left              | N/A (2D)          |
+------------------+-------------------+-------------------+-------------------+

Vehicle Frame
=============

The primary reference frame for the vehicle, following ISO 8855 convention.

Definition
----------

- **Origin**: Center of rear axle, at ground level
- **X-axis**: Points forward (positive ahead)
- **Y-axis**: Points left (positive to left)
- **Z-axis**: Points up (positive upward)

.. code-block:: text

                     Z (up)
                      │
                      │
                      │
            Y (left)  │
              ◄───────┼───────► -Y (right)
                      │
                      │
                      │
                      ▼ -Z (down)

                 (top view)

            Y (left)
              │
              │
              ▼
    ─────────────────────
   │                     │
   │                     │
   │         X           │──────► X (forward)
   │      (origin)       │
   │         ●           │
    ─────────────────────

Usage
-----

- Sensor mounting positions
- Point cloud output (after transformation)
- Control commands (steering angle, velocity)

World Frame (UTM)
=================

The global reference frame for navigation.

Definition
----------

- **Origin**: UTM zone reference point
- **X-axis**: East (positive eastward)
- **Y-axis**: North (positive northward)
- **Z-axis**: Up (positive upward)

This is an East-North-Up (ENU) coordinate system.

UTM Projection
--------------

GPS coordinates (WGS84 latitude/longitude) are converted to UTM meters:

.. code-block:: python

   from pyproj import Transformer

   # WGS84 to UTM Zone 11N (Southern California)
   transformer = Transformer.from_crs("EPSG:4326", "EPSG:32611")

   lat, lon = 34.0577, -117.8215  # Cal Poly Pomona
   east, north = transformer.transform(lat, lon)
   # Result: east=427859.2, north=3769432.1 meters

Vehicle Heading
---------------

Heading (yaw) is measured from North, clockwise positive:

- 0° = Facing North
- 90° = Facing East
- 180° = Facing South
- 270° = Facing West

.. code-block:: text

              North (0°)
                 │
                 │
    West (270°)──┼──► East (90°)
                 │
                 │
              South (180°)

LIDAR Frame
===========

The Velodyne VLP-16 sensor frame.

Definition
----------

- **Origin**: Center of LIDAR sensor
- **X-axis**: Forward (cable pointing backward)
- **Y-axis**: Left
- **Z-axis**: Up

.. note::

   The LIDAR frame is aligned with the vehicle frame when properly mounted.

Transformation
--------------

Points from LIDAR frame to vehicle frame:

.. code-block:: python

   # LIDAR mounted at (0.3, 0, 1.8) relative to vehicle origin
   def lidar_to_vehicle(points_lidar):
       # points_lidar: Nx3 array [x, y, z]
       translation = np.array([0.3, 0.0, 1.8])
       points_vehicle = points_lidar + translation
       return points_vehicle

Camera Frame
============

Standard computer vision (OpenCV) camera frame.

Definition
----------

- **Origin**: Camera optical center
- **X-axis**: Right (positive rightward in image)
- **Y-axis**: Down (positive downward in image)
- **Z-axis**: Forward (positive into scene)

.. code-block:: text

   Camera Frame:

       ──────────────────────
      │ ●───────► X (right)  │
      │ │                    │
      │ │                    │
      │ ▼ Y (down)           │
      │                      │
       ──────────────────────
            Image Plane

      Z points out of camera (into scene)

Transformation to Vehicle Frame
-------------------------------

.. code-block:: python

   # Camera frame to vehicle frame rotation
   # Camera: X-right, Y-down, Z-forward
   # Vehicle: X-forward, Y-left, Z-up

   R_cam_to_vehicle = np.array([
       [0, 0, 1],   # Vehicle X = Camera Z
       [-1, 0, 0],  # Vehicle Y = -Camera X
       [0, -1, 0]   # Vehicle Z = -Camera Y
   ])

   def camera_to_vehicle(points_camera, R, t):
       """Transform points from camera to vehicle frame."""
       points_vehicle = (R @ points_camera.T).T + t
       return points_vehicle

Occupancy Grid Frame
====================

The 2D grid used for obstacle mapping.

Definition
----------

- **Origin**: Vehicle center (can be offset)
- **X-axis**: Forward
- **Y-axis**: Left
- **Resolution**: Configurable (default 0.1 m/cell)

.. code-block:: text

   Grid Frame (top-down view):

         Y (columns, left)
         ▲
         │
         │
         │
         ├───────────────────►  X (rows, forward)
        (0,0)

   Grid indices:
   - Row 0 = rear of grid
   - Column 0 = right side of grid

Grid to World Transformation
----------------------------

.. code-block:: python

   def grid_to_world(grid_x, grid_y, vehicle_x, vehicle_y, vehicle_heading):
       """Convert grid coordinates to world coordinates."""
       # Grid to vehicle frame
       veh_x = (grid_x - grid_center_x) * resolution
       veh_y = (grid_y - grid_center_y) * resolution

       # Vehicle to world (rotate by heading)
       cos_h = np.cos(vehicle_heading)
       sin_h = np.sin(vehicle_heading)
       world_x = vehicle_x + veh_x * cos_h - veh_y * sin_h
       world_y = vehicle_y + veh_x * sin_h + veh_y * cos_h

       return world_x, world_y

Common Transformations
======================

GPS to Vehicle Relative
-----------------------

Convert a GPS waypoint to vehicle-relative coordinates:

.. code-block:: python

   def gps_to_vehicle_relative(waypoint_lat, waypoint_lon,
                                vehicle_lat, vehicle_lon,
                                vehicle_heading):
       # Convert both to UTM
       wp_x, wp_y = gps_to_utm(waypoint_lat, waypoint_lon)
       veh_x, veh_y = gps_to_utm(vehicle_lat, vehicle_lon)

       # Relative position in world frame
       dx = wp_x - veh_x
       dy = wp_y - veh_y

       # Rotate to vehicle frame
       cos_h = np.cos(-vehicle_heading)
       sin_h = np.sin(-vehicle_heading)
       rel_x = dx * cos_h - dy * sin_h  # Forward
       rel_y = dx * sin_h + dy * cos_h  # Left

       return rel_x, rel_y

LIDAR Point to World
--------------------

Transform a LIDAR point to world coordinates:

.. code-block:: python

   def lidar_to_world(point_lidar, vehicle_pose):
       """
       point_lidar: [x, y, z] in LIDAR frame
       vehicle_pose: [x, y, heading] in world frame
       """
       # LIDAR to vehicle
       lidar_offset = [0.3, 0.0, 1.8]
       point_vehicle = point_lidar + lidar_offset

       # Vehicle to world
       veh_x, veh_y, heading = vehicle_pose
       cos_h = np.cos(heading)
       sin_h = np.sin(heading)

       world_x = veh_x + point_vehicle[0] * cos_h - point_vehicle[1] * sin_h
       world_y = veh_y + point_vehicle[0] * sin_h + point_vehicle[1] * cos_h
       world_z = point_vehicle[2]

       return [world_x, world_y, world_z]

Reference Standards
===================

- **ISO 8855**: Road vehicles — Vehicle dynamics and road-holding ability — Vocabulary
- **REP 103**: ROS Standard Units of Measure and Coordinate Conventions
- **UTM**: Universal Transverse Mercator coordinate system
