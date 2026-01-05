.. UTM Navigator documentation master file

==========================================
UTM Navigator - Autonomous Vehicle Platform
==========================================

.. image:: _static/cpp_logo.png
   :alt: Cal Poly Pomona Logo
   :align: center
   :width: 300px

|

Welcome to the official documentation for the **UTM Navigator**, an autonomous vehicle research platform developed at the **Autonomous Vehicle Laboratory** at **California State Polytechnic University, Pomona (Cal Poly Pomona)**.

About the Vehicle
=================

The UTM Navigator is a full-scale autonomous vehicle platform designed for research and development in self-driving technology. Built as a comprehensive research tool, this vehicle serves as a testbed for exploring cutting-edge algorithms in perception, planning, and control for autonomous navigation.

The platform integrates multiple sensor modalities, advanced computing hardware, and custom actuation systems to enable fully autonomous operation in real-world environments. It provides students and researchers with hands-on experience in developing and testing autonomous vehicle systems.

Key Capabilities
----------------

- **Autonomous Navigation**: GPS-guided waypoint following with centimeter-level RTK precision
- **Obstacle Detection & Avoidance**: Real-time 3D LIDAR-based perception with Bayesian occupancy mapping
- **Multi-Task Perception**: Deep learning-based detection of vehicles, pedestrians, lanes, and drivable areas
- **Local Path Planning**: Dynamic Window Approach (DWA) for reactive obstacle avoidance
- **Path Tracking Control**: Pure Pursuit steering controller with speed-adaptive lookahead

About This Documentation
========================

This documentation provides a complete technical reference for the UTM Navigator platform, covering all aspects of the vehicle's design and operation:

**Hardware**
   Complete specifications and setup guides for all sensors, computing systems, actuators, and electrical components. Includes wiring diagrams, communication protocols, and integration details.

**Mechanical**
   Vehicle dimensions, physical specifications, sensor mounting positions, and coordinate system definitions. Reference material for understanding the vehicle's physical configuration.

**Software**
   In-depth documentation of the software architecture, including perception pipelines, planning algorithms, control systems, and sensor interfaces. Full API reference with usage examples.

Whether you are a new lab member getting started, a researcher extending the platform's capabilities, or an external collaborator seeking to understand the system, this documentation aims to provide all the information you need.

.. note::
   This documentation is actively maintained by the Autonomous Vehicle Laboratory at Cal Poly Pomona. For questions, contributions, or to report issues, please contact the lab directly.


Documentation Contents
======================

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   getting-started/index
   getting-started/quick-start
   getting-started/installation
   getting-started/dependencies
   getting-started/first-run

.. toctree::
   :maxdepth: 2
   :caption: Hardware

   hardware/index
   hardware/system-overview
   hardware/sensors
   hardware/computing
   hardware/actuators
   hardware/power-system
   hardware/wiring

.. toctree::
   :maxdepth: 2
   :caption: Mechanical

   mechanical/index
   mechanical/vehicle-specifications
   mechanical/sensor-mounting
   mechanical/coordinate-systems

.. toctree::
   :maxdepth: 2
   :caption: Software

   software/index
   software/architecture
   software/perception
   software/planning
   software/control
   software/sensors
   software/visualization

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/index

.. toctree::
   :maxdepth: 2
   :caption: Configuration

   configuration/index
   configuration/dwa-config
   configuration/sensor-calibration
   configuration/vehicle-parameters

.. toctree::
   :maxdepth: 2
   :caption: Tutorials

   tutorials/index
   tutorials/basic-navigation
   tutorials/obstacle-avoidance
   tutorials/custom-routes
   tutorials/tuning-controllers

.. toctree::
   :maxdepth: 2
   :caption: Theory & Background

   theory/index
   theory/pure-pursuit
   theory/dwa
   theory/occupancy-grids
   theory/coordinate-transforms

.. toctree::
   :maxdepth: 2
   :caption: Troubleshooting

   troubleshooting/index
   troubleshooting/common-issues
   troubleshooting/sensor-debugging

.. toctree::
   :maxdepth: 2
   :caption: Development

   development/index
   development/contributing
   development/code-style

.. toctree::
   :maxdepth: 1
   :caption: Appendix

   appendix/glossary
   appendix/references
   appendix/changelog


Quick Links
===========

* :ref:`genindex` - Index of all documented items
* :ref:`modindex` - Index of all modules
* :ref:`search` - Search the documentation


----

.. centered:: Autonomous Vehicle Laboratory | Cal Poly Pomona

.. centered:: College of Engineering | Department of Electrical and Computer Engineering
