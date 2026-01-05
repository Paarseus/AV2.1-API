===========
Quick Start
===========

Get the UTM Navigator running in 5 minutes. This guide assumes all hardware is already set up and connected.

.. contents:: Steps
   :local:
   :depth: 1

1. Clone the Repository
=======================

.. code-block:: bash

   git clone https://github.com/cpp-avl/utm-navigator.git
   cd utm-navigator

2. Create Virtual Environment
=============================

.. code-block:: bash

   python3 -m venv venv
   source venv/bin/activate

3. Install Dependencies
=======================

.. code-block:: bash

   pip install -r requirements.txt

4. Verify Sensor Connections
============================

Run the sensor check script:

.. code-block:: bash

   python scripts/check_sensors.py

Expected output:

.. code-block:: text

   [OK] Xsens MTi-630 connected on /dev/ttyUSB0
   [OK] Velodyne VLP-16 receiving data on port 2368
   [OK] Camera detected at /dev/video0
   [OK] Teensy CAN master connected on /dev/ttyACM0

5. Run a Basic Test
===================

Start the system in manual mode to verify everything works:

.. code-block:: bash

   python manual_control.py

Use the keyboard controls:

- ``W/S`` - Throttle forward/reverse
- ``A/D`` - Steering left/right
- ``Space`` - Brake
- ``Q`` - Quit

6. Run Autonomous Navigation
============================

Once manual control works, try autonomous navigation:

.. code-block:: bash

   python runner.py --lat 34.0577 --lon -117.8215

.. warning::
   Always have an operator ready to take manual control. Ensure the test area is clear of obstacles and people.

Next Steps
==========

- :doc:`installation` - Complete installation guide with troubleshooting
- :doc:`../tutorials/basic-navigation` - Detailed navigation tutorial
- :doc:`../hardware/index` - Hardware documentation
