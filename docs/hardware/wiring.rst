======
Wiring
======

Wiring diagrams and connector pinouts for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

System Wiring Overview
======================

.. code-block:: text

   ┌────────────────────────────────────────────────────────────────────────────┐
   │                         WIRING OVERVIEW                                    │
   ├────────────────────────────────────────────────────────────────────────────┤
   │                                                                            │
   │    ┌─────────────┐                                                         │
   │    │   XSENS     │◄─── USB Cable ──────────────┐                          │
   │    │   MTi-630   │                              │                          │
   │    └─────────────┘                              │                          │
   │                                                 │                          │
   │    ┌─────────────┐                              │    ┌─────────────────┐  │
   │    │  VELODYNE   │◄─── Ethernet Cat6 ──────────┼───►│                 │  │
   │    │   VLP-16    │◄─── 12V Power ──────┐       │    │      MAIN       │  │
   │    └─────────────┘                     │       │    │    COMPUTER     │  │
   │                                        │       │    │                 │  │
   │    ┌─────────────┐                     │       │    │  ┌───────────┐  │  │
   │    │  REALSENSE  │◄─── USB 3.0 ────────┼───────┼───►│  │ USB Hub   │  │  │
   │    │    D435     │                     │       │    │  └───────────┘  │  │
   │    └─────────────┘                     │       │    │                 │  │
   │                                        │       │    │  ┌───────────┐  │  │
   │    ┌─────────────┐                     │       └───►│  │  Ethernet │  │  │
   │    │ USB CAMERAS │◄─── USB 2.0 ────────┼───────────►│  │   Port    │  │  │
   │    └─────────────┘                     │            │  └───────────┘  │  │
   │                                        │            │                 │  │
   │    ┌─────────────┐                     │            └────────┬────────┘  │
   │    │   TEENSY    │◄─── USB Serial ─────┼─────────────────────┘           │
   │    │    4.1      │                     │                                  │
   │    │             │◄─── 5V Power ───────┤                                  │
   │    └──────┬──────┘                     │                                  │
   │           │ CAN                        │                                  │
   │           ▼                            │                                  │
   │    ┌─────────────┐                     │     ┌────────────────────────┐  │
   │    │     CAN     │                     │     │   POWER DISTRIBUTION   │  │
   │    │ TRANSCEIVER │                     │     │         BOARD          │  │
   │    └──────┬──────┘                     │     │                        │  │
   │           │                            └────►│  12V ──► Sensors       │  │
   │           ▼ CAN Bus                          │   5V ──► USB/Teensy    │  │
   │    ═══════════════                           │  19V ──► Computer      │  │
   │    ║  VEHICLE   ║                            │                        │  │
   │    ║ ACTUATORS  ║                            │         ▲              │  │
   │    ═══════════════                           │         │              │  │
   │                                              │    ┌────┴────┐         │  │
   │                                              │    │ AUX     │         │  │
   │                                              │    │ BATTERY │         │  │
   │                                              │    │  12V    │         │  │
   │                                              │    └─────────┘         │  │
   │                                              └────────────────────────┘  │
   │                                                                            │
   └────────────────────────────────────────────────────────────────────────────┘

Xsens MTi-630 Wiring
====================

**Connection**: USB Mini-B or 10-pin connector

USB Connection (Recommended)
----------------------------

.. code-block:: text

   Xsens USB Port ◄──── USB Mini-B Cable ────► Computer USB Port

10-Pin Connector
----------------

+------+--------+-------------+
| Pin  | Color  | Signal      |
+======+========+=============+
| 1    | Red    | VCC (5V)    |
+------+--------+-------------+
| 2    | Black  | GND         |
+------+--------+-------------+
| 3    | Green  | TX (out)    |
+------+--------+-------------+
| 4    | White  | RX (in)     |
+------+--------+-------------+
| 5-10 | -      | Reserved    |
+------+--------+-------------+

Velodyne VLP-16 Wiring
======================

Interface Box Connections
-------------------------

**Power Input**:

+--------+-------------+
| Wire   | Connection  |
+========+=============+
| Red    | +12V        |
+--------+-------------+
| Black  | GND         |
+--------+-------------+

**Ethernet**:

Standard Cat6 cable to computer Ethernet port.

**GPS Input** (Optional):

+------+-------------+
| Pin  | Signal      |
+======+=============+
| 1    | PPS Input   |
+------+-------------+
| 2    | GND         |
+------+-------------+
| 3    | GPS NMEA RX |
+------+-------------+

Intel RealSense Wiring
======================

**Connection**: USB 3.0 Type-C

.. code-block:: text

   RealSense USB-C ◄──── USB-C to USB-A 3.0 Cable ────► Computer USB 3.0 Port

.. note::

   Use a high-quality USB 3.0 cable. Poor cables cause bandwidth issues and dropped frames.

Teensy 4.1 Wiring
=================

USB Connection
--------------

.. code-block:: text

   Teensy Micro-USB ◄──── USB Cable ────► Computer USB Port

CAN Bus Interface
-----------------

Using MCP2562 transceiver:

.. code-block:: text

   Teensy 4.1                  MCP2562                Vehicle CAN
   ──────────                  ───────                ───────────
   Pin 22 (CTX1) ──────────► TXD (1)
   Pin 23 (CRX1) ◄────────── RXD (4)
                              CANH (7) ────────────► CAN_H
                              CANL (6) ────────────► CAN_L
   3.3V ──────────────────► VDD (3)
   GND ───────────────────► VSS (2), STBY (8)

**Termination**: 120Ω resistor between CAN_H and CAN_L at each end of the bus.

CAN Bus Wiring
==============

Topology
--------

.. code-block:: text

   ┌─────────┐      ┌─────────┐      ┌─────────┐      ┌─────────┐
   │ Teensy  │      │Steering │      │Throttle │      │ Brake   │
   │  CAN    │══════│   ECU   │══════│   ECU   │══════│   ECU   │
   └────┬────┘      └────┬────┘      └────┬────┘      └────┬────┘
        │                │                │                │
       120Ω             ─┴─              ─┴─              120Ω
        │                                                  │
       GND                                                GND

Cable Specifications
--------------------

- **Type**: Twisted pair, shielded
- **Gauge**: 22-24 AWG
- **Max Length**: 40 meters (500 kbps)
- **Termination**: 120Ω at each end

Power Distribution Wiring
=========================

From Auxiliary Battery
----------------------

.. code-block:: text

   ┌──────────────┐
   │  AUX BATTERY │
   │     12V      │
   └───────┬──────┘
           │
           ▼
   ┌──────────────────────────────────────────────────────────────┐
   │                    POWER DISTRIBUTION BOARD                  │
   │                                                              │
   │   ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐     │
   │   │ 15A │  │ 5A  │  │ 2A  │  │ 2A  │  │ 2A  │  │ 2A  │     │
   │   └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘  └──┬──┘     │
   │      │        │        │        │        │        │         │
   └──────┼────────┼────────┼────────┼────────┼────────┼─────────┘
          │        │        │        │        │        │
          ▼        ▼        ▼        ▼        ▼        ▼
       Computer  12V→5V   LIDAR    Fans    LEDs    Spare
                  Reg

Connector Reference
===================

Standard Connectors Used
------------------------

+----------------------+------------------+------------------+
| Component            | Connector Type   | Notes            |
+======================+==================+==================+
| Xsens                | USB Mini-B       | Or 10-pin        |
+----------------------+------------------+------------------+
| Velodyne             | Weatherpack      | Power + Ethernet |
+----------------------+------------------+------------------+
| RealSense            | USB-C            | USB 3.0 required |
+----------------------+------------------+------------------+
| Teensy               | Micro-USB        |                  |
+----------------------+------------------+------------------+
| CAN Bus              | Deutsch DT04     | Or bare wire     |
+----------------------+------------------+------------------+
| Power                | Anderson PP      | Or ring terminals|
+----------------------+------------------+------------------+

Wire Color Code
---------------

+--------+------------------+
| Color  | Signal           |
+========+==================+
| Red    | +12V / +5V       |
+--------+------------------+
| Black  | Ground           |
+--------+------------------+
| Yellow | CAN_H            |
+--------+------------------+
| Green  | CAN_L            |
+--------+------------------+
| Blue   | Signal / Data    |
+--------+------------------+
| White  | Signal / Data    |
+--------+------------------+
| Orange | Ignition / Enable|
+--------+------------------+

Best Practices
==============

1. **Strain Relief**: Secure all cables to prevent movement
2. **Shielding**: Ground shields at one end only
3. **Separation**: Keep power and signal cables apart
4. **Labeling**: Label all connectors and cables
5. **Fusing**: Fuse all power circuits appropriately
6. **Testing**: Verify connections before powering on
