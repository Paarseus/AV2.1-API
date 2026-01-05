================
Basic Navigation
================

Your first autonomous navigation run with the UTM Navigator.

.. contents:: Steps
   :local:
   :depth: 1

Prerequisites
=============

Before starting:

- [ ] All hardware connected and powered
- [ ] Software installed and configured
- [ ] Test area clear and safe
- [ ] Emergency stop accessible

Step 1: Start the System
========================

Open a terminal and activate the environment:

.. code-block:: bash

   cd ~/utm-navigator
   source venv/bin/activate

Step 2: Verify GPS
==================

Check GPS status:

.. code-block:: bash

   python -c "from xsens_class import XsensReceiver; x = XsensReceiver(); print(x.get_rtk_status())"

Wait for "Fixed" status for best accuracy.

Step 3: Choose Destination
==========================

Select a destination within line of sight, 20-50 meters away.

Get coordinates using Google Maps or GPS app.

Step 4: Launch Navigation
=========================

.. code-block:: bash

   python runner.py --lat 34.0580 --lon -117.8210

Step 5: Monitor Progress
========================

Watch the visualization window showing:

- Blue dot: Current position
- Green line: Planned route
- Red marker: Target waypoint

Step 6: Safe Shutdown
=====================

When destination reached or to stop:

1. Press ``Ctrl+C`` in terminal
2. Vehicle will brake and stop

Troubleshooting
===============

**Vehicle doesn't move**

- Check drive mode is set to "D"
- Verify throttle commands in logs
- Check emergency stop is released

**Vehicle oscillates**

- Reduce controller gains
- Increase lookahead distance
- Check for GPS interference

Next Steps
==========

- :doc:`obstacle-avoidance` - Add obstacle detection
- :doc:`tuning-controllers` - Improve performance
