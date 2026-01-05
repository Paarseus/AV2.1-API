========
Hardware
========

Complete hardware documentation for the UTM Navigator autonomous vehicle platform.

.. contents:: In This Section
   :local:
   :depth: 2

Overview
========

The UTM Navigator integrates multiple hardware systems to enable autonomous operation:

.. figure:: ../_static/hardware_overview.png
   :alt: Hardware System Overview
   :align: center
   :width: 80%

   UTM Navigator hardware architecture

System Components
=================

Sensors
-------

The perception system relies on multiple sensor modalities:

- **Xsens MTi-630 AHRS/GPS** - Position, orientation, and velocity
- **Velodyne VLP-16 LIDAR** - 3D point cloud for obstacle detection
- **Intel RealSense D435** - RGB-D camera for visual perception
- **USB Cameras** - Additional visual inputs

Computing
---------

- **Main Computer** - High-performance PC for perception and planning
- **Teensy 4.1** - Real-time CAN bus interface to vehicle actuators

Actuators
---------

- **Steering System** - Electronic power steering control
- **Throttle** - Electronic throttle control
- **Brake** - Brake-by-wire system

Power
-----

- **Main Battery** - Vehicle propulsion
- **Auxiliary Battery** - Computing and sensor power
- **Power Distribution** - Fused distribution to all components

Hardware Documentation
======================

.. toctree::
   :maxdepth: 2

   system-overview
   sensors
   computing
   actuators
   power-system
   wiring
