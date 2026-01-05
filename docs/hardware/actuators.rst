=========
Actuators
=========

Vehicle actuation systems for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The UTM Navigator uses drive-by-wire systems for autonomous control:

- **Steering**: Electronic power steering (EPS)
- **Throttle**: Electronic throttle control (ETC)
- **Brake**: Electro-hydraulic brake system

All actuators are controlled via CAN bus messages from the Teensy 4.1 interface.

Steering System
===============

Description
-----------

The steering system uses the vehicle's existing electronic power steering, modified to accept external commands.

Specifications
--------------

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Maximum Angle           | ±28° (at wheels)               |
+-------------------------+--------------------------------+
| Steering Ratio          | ~14:1                          |
+-------------------------+--------------------------------+
| Response Time           | < 100 ms                       |
+-------------------------+--------------------------------+
| Control Input           | CAN bus                        |
+-------------------------+--------------------------------+

Control Interface
-----------------

**CAN Message Format**:

.. code-block:: text

   CAN ID: 0x100
   Data Length: 8 bytes
   Byte 0-1: Steering angle (int16, 0.1° resolution)
   Byte 2: Control mode (0=manual, 1=auto)
   Byte 3-7: Reserved

**Software Command**:

.. code-block:: python

   # Steering command via vehicle_actuator.py
   actuator.set_steering(angle)  # -1.0 to 1.0 normalized
   # or
   actuator.set_steering_degrees(angle)  # -28 to +28 degrees

**Mapping**:

- Input: -1.0 to +1.0 (normalized)
- Output: -28° to +28° (wheel angle)
- Positive = Left, Negative = Right

Calibration
-----------

1. Center the steering wheel physically
2. Command zero steering and verify no drift
3. Adjust offset in configuration if needed:

.. code-block:: yaml

   steering:
     center_offset: 0.0  # degrees
     max_angle: 28.0
     invert: false

Throttle System
===============

Description
-----------

Electronic throttle control replaces the mechanical accelerator pedal linkage.

Specifications
--------------

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Input Range             | 0% to 100%                     |
+-------------------------+--------------------------------+
| Response Time           | < 50 ms                        |
+-------------------------+--------------------------------+
| Control Input           | CAN bus                        |
+-------------------------+--------------------------------+

Control Interface
-----------------

**CAN Message Format**:

.. code-block:: text

   CAN ID: 0x101
   Data Length: 8 bytes
   Byte 0: Throttle position (uint8, 0-255 = 0-100%)
   Byte 1: Control mode
   Byte 2-7: Reserved

**Software Command**:

.. code-block:: python

   # Throttle command
   actuator.set_throttle(value)  # 0.0 to 1.0

**Safety Limits**:

- Software limit: 50% for testing
- Hardware limit: 80% maximum

Brake System
============

Description
-----------

The brake system uses an electro-hydraulic actuator for brake-by-wire control.

Specifications
--------------

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Braking Force           | 0% to 100%                     |
+-------------------------+--------------------------------+
| Response Time           | < 100 ms                       |
+-------------------------+--------------------------------+
| Control Input           | CAN bus                        |
+-------------------------+--------------------------------+

Control Interface
-----------------

**CAN Message Format**:

.. code-block:: text

   CAN ID: 0x102
   Data Length: 8 bytes
   Byte 0: Brake pressure (uint8, 0-255 = 0-100%)
   Byte 1: Control mode
   Byte 2-7: Reserved

**Software Command**:

.. code-block:: python

   # Brake command
   actuator.set_brake(value)  # 0.0 to 1.0

.. warning::

   The brake system has a mechanical override. The physical brake pedal always takes priority.

Drive Mode Selection
====================

The transmission/drive mode can be commanded:

+------+------------------+
| Mode | Description      |
+======+==================+
| N    | Neutral          |
+------+------------------+
| D    | Drive            |
+------+------------------+
| S    | Sport            |
+------+------------------+
| R    | Reverse          |
+------+------------------+

**Command**:

.. code-block:: python

   actuator.set_drive_mode('D')  # Set to Drive

Emergency Stop
==============

The emergency stop system immediately disables all actuators:

**Activation Methods**:

1. Physical E-Stop button (hardwired)
2. Software command: ``actuator.emergency_stop()``
3. Watchdog timeout (no commands for 100ms)

**Behavior**:

1. Throttle set to 0%
2. Brakes applied at 100%
3. All CAN commands ignored until reset

**Reset**:

1. Ensure E-Stop button is released
2. Call ``actuator.reset_emergency_stop()``
3. Verify system status before resuming

Safety Considerations
=====================

.. danger::

   **Always ensure**:

   - An operator is ready to take manual control
   - The physical E-Stop is accessible
   - The test area is clear
   - All bystanders are at a safe distance

**Fail-Safe Behavior**:

+----------------------+---------------------------+
| Failure Mode         | Response                  |
+======================+===========================+
| CAN bus failure      | Apply brakes              |
+----------------------+---------------------------+
| Computer crash       | Watchdog applies brakes   |
+----------------------+---------------------------+
| Sensor failure       | Reduce speed, alert       |
+----------------------+---------------------------+
| Power loss           | Mechanical brakes engage  |
+----------------------+---------------------------+

Wiring
======

See :doc:`wiring` for detailed actuator wiring diagrams.
