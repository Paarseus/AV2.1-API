============
Power System
============

Power distribution and management for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The UTM Navigator uses a dual-battery system:

1. **Main Battery**: Powers vehicle propulsion
2. **Auxiliary Battery**: Powers computing and sensors

This separation ensures sensor/compute systems remain operational even under heavy propulsion loads.

Power Architecture
==================

.. code-block:: text

   ┌─────────────────────────────────────────────────────────────────┐
   │                       POWER DISTRIBUTION                       │
   ├─────────────────────────────────────────────────────────────────┤
   │                                                                 │
   │  ┌─────────────┐                    ┌─────────────────────────┐ │
   │  │    MAIN     │                    │      AUXILIARY          │ │
   │  │   BATTERY   │                    │       BATTERY           │ │
   │  │   (48V/72V) │                    │        (12V)            │ │
   │  └──────┬──────┘                    └───────────┬─────────────┘ │
   │         │                                       │               │
   │         ▼                                       ▼               │
   │  ┌─────────────┐                    ┌─────────────────────────┐ │
   │  │   Motor     │                    │   Power Distribution    │ │
   │  │ Controller  │                    │        Board            │ │
   │  └──────┬──────┘                    └───────────┬─────────────┘ │
   │         │                                       │               │
   │         ▼                           ┌───────────┼───────────┐   │
   │  ┌─────────────┐                    │           │           │   │
   │  │   Drive     │                    ▼           ▼           ▼   │
   │  │   Motor     │              ┌─────────┐ ┌─────────┐ ┌───────┐ │
   │  └─────────────┘              │Computer │ │ Sensors │ │Teensy │ │
   │                               │ (12/19V)│ │  (12V)  │ │ (5V)  │ │
   │                               └─────────┘ └─────────┘ └───────┘ │
   │                                                                 │
   └─────────────────────────────────────────────────────────────────┘

Main Battery
============

Specifications
--------------

.. note::

   Specifications depend on vehicle platform. Update with actual values.

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Type                    | Lithium-ion / Lead-acid        |
+-------------------------+--------------------------------+
| Voltage                 | 48V / 72V nominal              |
+-------------------------+--------------------------------+
| Capacity                | TBD Ah                         |
+-------------------------+--------------------------------+
| Max Discharge           | TBD A                          |
+-------------------------+--------------------------------+

Purpose
-------

- Vehicle propulsion motor
- Steering assist motor
- Brake booster (if electric)

Auxiliary Battery
=================

Specifications
--------------

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Type                    | AGM / Lithium-ion              |
+-------------------------+--------------------------------+
| Voltage                 | 12V nominal                    |
+-------------------------+--------------------------------+
| Capacity                | 20-50 Ah                       |
+-------------------------+--------------------------------+
| Max Discharge           | 30 A continuous                |
+-------------------------+--------------------------------+

Purpose
-------

- Main computer
- All sensors
- Teensy CAN master
- Status lights and indicators

Power Distribution Board
========================

The power distribution board provides:

1. **Fused Outputs**: Individual fuses for each subsystem
2. **Voltage Regulation**: 12V, 5V, 3.3V rails
3. **Monitoring**: Voltage and current sensing
4. **Switching**: Controlled power-on sequence

Fuse Ratings
------------

+-------------------+----------+----------+
| Circuit           | Voltage  | Fuse     |
+===================+==========+==========+
| Main Computer     | 12V/19V  | 15A      |
+-------------------+----------+----------+
| Velodyne LIDAR    | 12V      | 2A       |
+-------------------+----------+----------+
| Xsens GPS/IMU     | 5V       | 1A       |
+-------------------+----------+----------+
| RealSense Camera  | 5V (USB) | 1A       |
+-------------------+----------+----------+
| Teensy + CAN      | 5V       | 1A       |
+-------------------+----------+----------+
| USB Hub           | 5V       | 2A       |
+-------------------+----------+----------+
| Cooling Fans      | 12V      | 2A       |
+-------------------+----------+----------+
| Status LEDs       | 12V      | 1A       |
+-------------------+----------+----------+

Power Consumption
=================

Typical System
--------------

+-----------------------+----------+----------+----------+
| Component             | Voltage  | Typical  | Peak     |
+=======================+==========+==========+==========+
| Main Computer         | 12V      | 6 A      | 12 A     |
+-----------------------+----------+----------+----------+
| Velodyne VLP-16       | 12V      | 0.7 A    | 1 A      |
+-----------------------+----------+----------+----------+
| Xsens MTi-630         | 5V       | 0.2 A    | 0.3 A    |
+-----------------------+----------+----------+----------+
| RealSense D435        | 5V       | 0.7 A    | 1 A      |
+-----------------------+----------+----------+----------+
| Teensy 4.1            | 5V       | 0.1 A    | 0.2 A    |
+-----------------------+----------+----------+----------+
| Cooling Fans          | 12V      | 0.5 A    | 1 A      |
+-----------------------+----------+----------+----------+
| **Total (12V equiv)** |          | ~10 A    | ~18 A    |
+-----------------------+----------+----------+----------+

Runtime Estimation
------------------

With a 50 Ah auxiliary battery:

.. code-block:: text

   Runtime = Battery Capacity / Average Current
   Runtime = 50 Ah / 10 A = 5 hours (typical)

Safety Features
===============

Emergency Shutoff
-----------------

The main power can be cut via:

1. **Master Switch**: Manual disconnect
2. **E-Stop Circuit**: Opens relay to cut power
3. **BMS Cutoff**: Battery management system protection

Low Voltage Protection
----------------------

The power distribution board monitors battery voltage:

- **Warning**: < 11.5V → Reduce load
- **Cutoff**: < 10.5V → Shutdown non-essential systems

Charging
========

Auxiliary Battery
-----------------

1. Connect charger to auxiliary battery
2. Use appropriate charger (12V, 10A max recommended)
3. Monitor charging progress
4. Disconnect when fully charged

.. warning::

   Do not operate the vehicle while charging.

Wiring
======

See :doc:`wiring` for detailed power wiring diagrams.
