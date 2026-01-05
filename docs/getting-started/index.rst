===============
Getting Started
===============

Welcome to the UTM Navigator! This section will guide you through setting up and running the autonomous vehicle platform for the first time.

.. contents:: In This Section
   :local:
   :depth: 2

Overview
========

Before operating the UTM Navigator, you'll need to:

1. **Install the software** - Set up Python dependencies and sensor SDKs
2. **Configure the hardware** - Connect and verify all sensors and actuators
3. **Calibrate the system** - Ensure sensors are properly calibrated
4. **Run initial tests** - Verify the system is working correctly

Prerequisites
=============

**Hardware Requirements**

- UTM Navigator vehicle platform (fully assembled)
- Computing unit with Ubuntu 20.04/22.04 LTS
- NVIDIA GPU (GTX 1060 or better) for perception
- All sensors connected and powered

**Software Requirements**

- Python 3.8 or higher
- CUDA 11.x (for GPU acceleration)
- Git for version control

**Knowledge Requirements**

- Basic familiarity with Linux command line
- Understanding of Python programming
- Familiarity with autonomous vehicle concepts (helpful but not required)

Next Steps
==========

.. toctree::
   :maxdepth: 1

   quick-start
   installation
   dependencies
   first-run

If you're eager to get started quickly, proceed to the :doc:`quick-start` guide. For a more thorough setup, follow the complete :doc:`installation` instructions.
