========
Software
========

Software architecture and components of the UTM Navigator.

.. contents:: In This Section
   :local:
   :depth: 2

Overview
========

The UTM Navigator software stack is a complete autonomous vehicle system implemented in Python. It provides:

- **Perception**: LIDAR and camera-based obstacle detection
- **Planning**: Route planning and local trajectory optimization
- **Control**: Steering and speed control
- **Sensors**: Interfaces for GPS, LIDAR, and cameras

Architecture Summary
====================

.. code-block:: text

   ┌─────────────────────────────────────────────────────────────────────┐
   │                      UTM NAVIGATOR SOFTWARE                         │
   ├─────────────────────────────────────────────────────────────────────┤
   │                                                                     │
   │  SENSORS              PERCEPTION           PLANNING        CONTROL  │
   │  ────────             ──────────           ────────        ──────── │
   │  ┌─────────┐         ┌───────────┐       ┌─────────┐    ┌─────────┐│
   │  │  Xsens  │────────►│Occupancy  │──────►│  DWA    │───►│  Pure   ││
   │  │ GPS/IMU │         │   Grid    │       │ Planner │    │ Pursuit ││
   │  └─────────┘         └───────────┘       └─────────┘    └────┬────┘│
   │                                                               │     │
   │  ┌─────────┐         ┌───────────┐       ┌─────────┐         │     │
   │  │Velodyne │────────►│  Point    │──────►│  Route  │         │     │
   │  │  LIDAR  │         │  Cloud    │       │Navigator│         │     │
   │  └─────────┘         └───────────┘       └─────────┘         │     │
   │                                                               │     │
   │  ┌─────────┐         ┌───────────┐                           ▼     │
   │  │RealSense│────────►│ YOLOPv2   │                    ┌───────────┐│
   │  │ Camera  │         │ Detection │                    │  Vehicle  ││
   │  └─────────┘         └───────────┘                    │  Actuator ││
   │                                                        └───────────┘│
   │                                                                     │
   │  ┌──────────────────────────────────────────────────────────────┐  │
   │  │                    MAIN CONTROL LOOP                         │  │
   │  │                      (runner.py)                             │  │
   │  │  Orchestrates all components at 50-100 Hz                    │  │
   │  └──────────────────────────────────────────────────────────────┘  │
   │                                                                     │
   └─────────────────────────────────────────────────────────────────────┘

Key Components
==============

+----------------------+----------------------+--------------------------------+
| Component            | File                 | Purpose                        |
+======================+======================+================================+
| Main Controller      | runner.py            | Orchestrates all systems       |
+----------------------+----------------------+--------------------------------+
| GPS/IMU Interface    | xsens_class.py       | Position, heading, velocity    |
+----------------------+----------------------+--------------------------------+
| LIDAR Interface      | lidar_interface.py   | Point cloud acquisition        |
+----------------------+----------------------+--------------------------------+
| Camera Interface     | camera_interface.py  | RGB/Depth image capture        |
+----------------------+----------------------+--------------------------------+
| Occupancy Grid       | occupancy_grid.py    | Bayesian obstacle mapping      |
+----------------------+----------------------+--------------------------------+
| YOLOPv2 Detector     | yolopv2_adapter.py   | Object/lane detection          |
+----------------------+----------------------+--------------------------------+
| Route Navigator      | osmnx_class.py       | OpenStreetMap routing          |
+----------------------+----------------------+--------------------------------+
| DWA Planner          | dwa_planner.py       | Local obstacle avoidance       |
+----------------------+----------------------+--------------------------------+
| Pure Pursuit         | pure_pursuit_*.py    | Path tracking controller       |
+----------------------+----------------------+--------------------------------+
| Vehicle Actuator     | vehicle_actuator.py  | CAN bus command interface      |
+----------------------+----------------------+--------------------------------+

Data Flow
=========

1. **Sensor Acquisition** (100 Hz)
   - Xsens provides position, heading, velocity
   - LIDAR provides 3D point cloud
   - Camera provides RGB/depth images

2. **Perception Processing** (10-30 Hz)
   - Point cloud → Occupancy grid
   - Images → YOLOPv2 → Detections

3. **Planning** (10-20 Hz)
   - Route navigation → Global waypoints
   - DWA → Local velocity commands

4. **Control** (50-100 Hz)
   - Pure pursuit → Steering angle
   - Velocity control → Throttle/brake

5. **Actuation** (50 Hz)
   - Commands sent to Teensy via serial
   - Teensy relays to vehicle CAN bus

Threading Model
===============

The system uses multiple threads for concurrent operation:

- **Main Thread**: Control loop and coordination
- **Sensor Threads**: Background data acquisition
- **Visualization Thread**: Optional real-time display

Thread-safe data structures (``VehicleState``, ``PerceptionState``) enable safe communication between threads.

Contents
========

.. toctree::
   :maxdepth: 2

   architecture
   perception
   planning
   control
   sensors
   visualization
