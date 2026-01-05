=========
Computing
=========

Computing hardware for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Main Computer
=============

Overview
--------

The main computer handles all perception, planning, and high-level control tasks.

Recommended Specifications
--------------------------

+------------------+----------------------------+--------------------------------+
| Component        | Minimum                    | Recommended                    |
+==================+============================+================================+
| CPU              | Intel i5-8400 / Ryzen 5    | Intel i7-10700 / Ryzen 7       |
+------------------+----------------------------+--------------------------------+
| RAM              | 8 GB DDR4                  | 16-32 GB DDR4                  |
+------------------+----------------------------+--------------------------------+
| GPU              | NVIDIA GTX 1050 Ti (4GB)   | NVIDIA GTX 1060+ (6GB+)        |
+------------------+----------------------------+--------------------------------+
| Storage          | 256 GB SSD                 | 512 GB NVMe SSD                |
+------------------+----------------------------+--------------------------------+
| USB              | 3× USB 3.0                 | 4× USB 3.0, 2× USB 2.0         |
+------------------+----------------------------+--------------------------------+
| Ethernet         | 1× Gigabit                 | 2× Gigabit                     |
+------------------+----------------------------+--------------------------------+

Form Factor Options
-------------------

**Desktop/Tower**

- Pros: Best performance, easy cooling, expandable
- Cons: Large, requires secure mounting

**Mini PC (NUC-style)**

- Pros: Compact, low power
- Cons: Limited GPU options, thermal constraints

**Industrial PC**

- Pros: Ruggedized, wide temperature range, vibration resistant
- Cons: Higher cost

**Laptop**

- Pros: Integrated display, battery backup
- Cons: Difficult to mount, limited ports

Current Setup
-------------

The UTM Navigator currently uses:

- **Platform**: Custom desktop build
- **CPU**: Intel Core i7-10700K
- **RAM**: 32 GB DDR4-3200
- **GPU**: NVIDIA GeForce RTX 2060
- **Storage**: 512 GB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

Software Requirements
---------------------

.. code-block:: bash

   # Operating System
   Ubuntu 20.04 LTS or 22.04 LTS

   # NVIDIA Drivers
   nvidia-driver-525 (or newer)

   # CUDA Toolkit
   cuda-toolkit-11.8

   # Python
   Python 3.8+

Teensy 4.1 CAN Master
=====================

Overview
--------

The Teensy 4.1 serves as a real-time bridge between the main computer and the vehicle's CAN bus.

.. figure:: ../_static/teensy41.png
   :alt: Teensy 4.1
   :align: center
   :width: 300px

   Teensy 4.1 Microcontroller

Specifications
--------------

+-------------------------+--------------------------------+
| Parameter               | Value                          |
+=========================+================================+
| Processor               | ARM Cortex-M7 @ 600 MHz        |
+-------------------------+--------------------------------+
| RAM                     | 1024 KB                        |
+-------------------------+--------------------------------+
| Flash                   | 8 MB (+ microSD slot)          |
+-------------------------+--------------------------------+
| CAN Bus                 | 3× CAN 2.0B                    |
+-------------------------+--------------------------------+
| USB                     | USB 2.0 (480 Mbps)             |
+-------------------------+--------------------------------+
| GPIO                    | 55 digital pins                |
+-------------------------+--------------------------------+
| Dimensions              | 61 × 18 mm                     |
+-------------------------+--------------------------------+

Purpose
-------

1. **Protocol Translation**: Convert USB serial commands to CAN messages
2. **Real-time Control**: Ensure consistent timing for actuator commands
3. **Safety Watchdog**: Emergency stop if communication lost
4. **Sensor Aggregation**: Optional low-level sensor interfaces

Communication Protocol
----------------------

The Teensy communicates with the main computer via USB serial:

**Baud Rate**: 115200

**Command Format**: ASCII text with newline terminator

**Commands**:

+----------+------------------+---------------------------+
| Command  | Format           | Description               |
+==========+==================+===========================+
| Throttle | ``T <value>\n``  | value: 0.0 to 1.0         |
+----------+------------------+---------------------------+
| Brake    | ``B <value>\n``  | value: 0.0 to 1.0         |
+----------+------------------+---------------------------+
| Steering | ``S <value>\n``  | value: -1.0 to 1.0        |
+----------+------------------+---------------------------+
| Mode     | ``M <mode>\n``   | mode: N, D, S, R          |
+----------+------------------+---------------------------+
| E-Stop   | ``E <0/1>\n``    | 0: release, 1: engage     |
+----------+------------------+---------------------------+

**Examples**:

.. code-block:: text

   T 0.5       # Set throttle to 50%
   S -0.25     # Steer 25% right
   B 1.0       # Full brake
   M D         # Drive mode
   E 1         # Emergency stop

Firmware
--------

The Teensy runs custom firmware for:

- USB serial parsing
- CAN message construction
- Watchdog timer (100ms timeout)
- Status reporting

Firmware source and flashing instructions: ``firmware/teensy_can_master/``

CAN Bus Interface
-----------------

**Transceiver**: MCP2562 or SN65HVD230

**Bit Rate**: 500 kbps (standard automotive)

**Wiring**:

.. code-block:: text

   Teensy Pin 22 (CTX1) → CAN Transceiver TX
   Teensy Pin 23 (CRX1) → CAN Transceiver RX
   CAN_H, CAN_L → Vehicle CAN Bus
   120Ω termination resistor

Safety Features
---------------

1. **Watchdog Timer**: If no command received for 100ms, apply brakes
2. **Command Validation**: Reject out-of-range values
3. **Emergency Stop**: Hardware interrupt for immediate stop
4. **Status Heartbeat**: 10 Hz status messages to main computer

Flashing Firmware
-----------------

.. code-block:: bash

   # Install Teensy Loader
   sudo apt install teensy-loader-cli

   # Compile and flash
   cd firmware/teensy_can_master
   make
   teensy_loader_cli --mcu=TEENSY41 -w main.hex

Power Requirements
==================

+------------------+---------------+---------------+
| Component        | Voltage       | Current       |
+==================+===============+===============+
| Main Computer    | 12V / 19V     | 5-15 A        |
+------------------+---------------+---------------+
| Teensy 4.1       | 5V (USB)      | 100 mA        |
+------------------+---------------+---------------+
| CAN Transceiver  | 5V            | 50 mA         |
+------------------+---------------+---------------+

See :doc:`power-system` for complete power distribution details.
