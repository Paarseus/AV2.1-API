===============
System Overview
===============

High-level overview of the UTM Navigator hardware architecture.

.. contents:: Contents
   :local:
   :depth: 2

Architecture Diagram
====================

.. code-block:: text

   ┌─────────────────────────────────────────────────────────────────────────────┐
   │                           UTM NAVIGATOR HARDWARE                            │
   ├─────────────────────────────────────────────────────────────────────────────┤
   │                                                                             │
   │  ┌─────────────────────────────────────────────────────────────────────┐   │
   │  │                         SENSOR SUITE                                │   │
   │  │  ┌──────────┐  ┌──────────────┐  ┌──────────┐  ┌──────────────┐    │   │
   │  │  │  Xsens   │  │   Velodyne   │  │RealSense │  │ USB Cameras  │    │   │
   │  │  │ MTi-630  │  │   VLP-16     │  │  D435    │  │              │    │   │
   │  │  │ GPS/IMU  │  │   LIDAR      │  │  RGB-D   │  │              │    │   │
   │  │  └────┬─────┘  └──────┬───────┘  └────┬─────┘  └──────┬───────┘    │   │
   │  │       │ USB          │ Ethernet       │ USB          │ USB         │   │
   │  └───────┼──────────────┼────────────────┼──────────────┼─────────────┘   │
   │          │              │                │              │                  │
   │  ┌───────┴──────────────┴────────────────┴──────────────┴─────────────┐   │
   │  │                       MAIN COMPUTER                                │   │
   │  │  ┌─────────────────────────────────────────────────────────────┐   │   │
   │  │  │  Ubuntu 22.04 LTS                                           │   │   │
   │  │  │  ┌─────────────┐ ┌─────────────┐ ┌───────────────────────┐  │   │   │
   │  │  │  │ Perception  │ │  Planning   │ │       Control         │  │   │   │
   │  │  │  │  Pipeline   │ │  (DWA)      │ │   (Pure Pursuit)      │  │   │   │
   │  │  │  └─────────────┘ └─────────────┘ └───────────────────────┘  │   │   │
   │  │  └─────────────────────────────────────────────────────────────┘   │   │
   │  └────────────────────────────────┬───────────────────────────────────┘   │
   │                                   │ USB Serial                            │
   │  ┌────────────────────────────────┴───────────────────────────────────┐   │
   │  │                      TEENSY 4.1 CAN MASTER                         │   │
   │  │              (Real-time actuator control interface)                │   │
   │  └────────────────────────────────┬───────────────────────────────────┘   │
   │                                   │ CAN Bus                               │
   │  ┌────────────────────────────────┴───────────────────────────────────┐   │
   │  │                         VEHICLE ACTUATORS                          │   │
   │  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │   │
   │  │  │   Steering   │  │   Throttle   │  │         Brake            │  │   │
   │  │  │   Actuator   │  │   Actuator   │  │        Actuator          │  │   │
   │  │  └──────────────┘  └──────────────┘  └──────────────────────────┘  │   │
   │  └────────────────────────────────────────────────────────────────────┘   │
   │                                                                             │
   │  ┌─────────────────────────────────────────────────────────────────────┐   │
   │  │                        POWER SYSTEM                                 │   │
   │  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │   │
   │  │  │ Main Battery │  │  Aux Battery │  │  Power Distribution      │  │   │
   │  │  │  (Vehicle)   │  │  (Computing) │  │        Board             │  │   │
   │  │  └──────────────┘  └──────────────┘  └──────────────────────────┘  │   │
   │  └─────────────────────────────────────────────────────────────────────┘   │
   │                                                                             │
   └─────────────────────────────────────────────────────────────────────────────┘

Data Flow
=========

Sensor Data Acquisition
-----------------------

1. **Xsens MTi-630** → USB Serial → Position, heading, velocity at 100 Hz
2. **Velodyne VLP-16** → Ethernet UDP → Point cloud at 10 Hz
3. **RealSense D435** → USB 3.0 → RGB + Depth at 30 Hz
4. **USB Cameras** → USB 2.0/3.0 → RGB at 30 Hz

Control Command Flow
--------------------

1. **Main Computer** computes steering and throttle commands
2. Commands sent via **USB Serial** to Teensy 4.1
3. **Teensy 4.1** translates to CAN messages
4. **CAN Bus** delivers commands to actuators

Communication Protocols
=======================

+-------------------+------------------+------------------+------------------+
| Interface         | Protocol         | Data Rate        | Latency          |
+===================+==================+==================+==================+
| Xsens             | USB Serial       | 115200 baud      | < 10 ms          |
+-------------------+------------------+------------------+------------------+
| Velodyne          | UDP              | ~100 Mbps        | < 5 ms           |
+-------------------+------------------+------------------+------------------+
| RealSense         | USB 3.0          | ~2 Gbps          | ~33 ms (30 fps)  |
+-------------------+------------------+------------------+------------------+
| Teensy            | USB Serial       | 115200 baud      | < 5 ms           |
+-------------------+------------------+------------------+------------------+
| Actuators         | CAN 2.0          | 500 kbps         | < 2 ms           |
+-------------------+------------------+------------------+------------------+

System Timing
=============

The control system operates at the following rates:

- **Perception Loop**: 10-30 Hz (sensor dependent)
- **Planning Loop**: 10-20 Hz
- **Control Loop**: 50-100 Hz
- **Actuator Commands**: 50 Hz

.. note::

   The control loop runs faster than perception to ensure smooth actuation even when sensor data is delayed.
