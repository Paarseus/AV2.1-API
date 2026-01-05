==================
Sensor Calibration
==================

Calibration procedures for UTM Navigator sensors.

.. contents:: Contents
   :local:
   :depth: 2

Camera Intrinsic Calibration
============================

For USB cameras without factory calibration.

Equipment Needed
----------------

- Checkerboard pattern (8x6 or similar)
- Flat surface

Procedure
---------

1. Print checkerboard pattern at known size
2. Capture 15-20 images from various angles
3. Run calibration script:

.. code-block:: bash

   python scripts/calibrate_camera.py --images calibration_images/ --pattern 8x6 --square_size 0.025

4. Save calibration to ``config/camera_calibration.yaml``

Camera-LIDAR Extrinsic Calibration
==================================

Align camera and LIDAR coordinate frames.

Procedure
---------

1. Place calibration target visible to both sensors
2. Collect synchronized data
3. Run alignment script
4. Verify alignment visually

GPS Antenna Lever Arm
=====================

Configure offset from GPS antenna to vehicle origin.

Measurement
-----------

1. Measure X, Y, Z offset from rear axle center to GPS antenna
2. Enter in Xsens configuration or software

.. code-block:: python

   gps_lever_arm = {
       'x': 0.0,   # forward (m)
       'y': 0.0,   # left (m)
       'z': 1.7    # up (m)
   }

Steering Calibration
====================

Ensure steering center is correct.

Procedure
---------

1. Place vehicle on level ground
2. Set steering command to 0
3. Measure any wheel offset
4. Adjust center offset in configuration
