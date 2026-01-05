=========
First Run
=========

Guide to running the UTM Navigator for the first time after installation.

.. contents:: Contents
   :local:
   :depth: 2

Pre-Flight Checklist
====================

Before starting the system, verify:

Hardware
--------

.. rst-class:: checklist

- [ ] All sensors powered and connected
- [ ] Teensy CAN master connected via USB
- [ ] GPS antenna has clear sky view
- [ ] Emergency stop is accessible
- [ ] Test area is clear of obstacles and people
- [ ] Vehicle is on stable, level ground

Software
--------

.. rst-class:: checklist

- [ ] Virtual environment activated
- [ ] All dependencies installed
- [ ] Configuration files reviewed
- [ ] Destination coordinates verified

Step 1: Activate Environment
============================

.. code-block:: bash

   cd ~/utm-navigator
   source venv/bin/activate

Step 2: Sensor Verification
===========================

Run individual sensor tests:

GPS/IMU (Xsens)
---------------

.. code-block:: bash

   python -c "from xsens_class import XsensReceiver; x = XsensReceiver(); print(x.get_position())"

Expected: Latitude, longitude, altitude values (may take 30-60 seconds for GPS lock)

LIDAR (Velodyne)
----------------

.. code-block:: bash

   python -c "from sensors.lidar_interface import VelodyneLIDAR; l = VelodyneLIDAR(); print(f'Points: {len(l.get_scan())}')"

Expected: Point count > 0 (typically 20,000-30,000 points)

Vehicle Actuator (Teensy)
-------------------------

.. code-block:: bash

   python -c "from vehicle_actuator import VehicleActuator; v = VehicleActuator(); v.set_steering(0)"

Expected: No errors, steering centers

Step 3: Visualization Test
==========================

Test the GPS visualizer to verify positioning:

.. code-block:: bash

   python gps_visualizer.py

This opens a real-time plot showing:

- Current GPS position
- Heading indicator
- RTK status

Wait for RTK fix (status should show "RTK Fixed" for best accuracy).

Step 4: Manual Control Test
===========================

Test vehicle control manually before autonomous operation:

.. code-block:: bash

   python manual_control.py

Controls:

+----------+-------------------+
| Key      | Action            |
+==========+===================+
| W        | Throttle forward  |
+----------+-------------------+
| S        | Throttle reverse  |
+----------+-------------------+
| A        | Steer left        |
+----------+-------------------+
| D        | Steer right       |
+----------+-------------------+
| Space    | Brake             |
+----------+-------------------+
| E        | Emergency stop    |
+----------+-------------------+
| Q        | Quit              |
+----------+-------------------+

.. warning::

   Start with small inputs. Verify steering and throttle respond correctly before increasing speed.

Step 5: First Autonomous Run
============================

For your first autonomous test, choose a simple, short route:

1. **Get current position**:

   .. code-block:: bash

      python -c "from xsens_class import XsensReceiver; x = XsensReceiver(); print(x.get_position())"

2. **Choose a destination** ~20-50 meters away with clear path

3. **Start autonomous navigation**:

   .. code-block:: bash

      python runner.py --lat <dest_latitude> --lon <dest_longitude>

   Example (Cal Poly Pomona area):

   .. code-block:: bash

      python runner.py --lat 34.0580 --lon -117.8210

4. **Monitor the visualization** showing:

   - Current position (blue dot)
   - Planned route (green line)
   - Target waypoint (red marker)
   - Steering and speed commands

5. **Be ready to intervene** - Keep hands near emergency stop

Stopping the System
===================

Normal Shutdown
---------------

Press ``Ctrl+C`` in the terminal. The system will:

1. Send zero throttle command
2. Apply brakes
3. Close sensor connections
4. Save log files

Emergency Stop
--------------

Press the physical emergency stop button or:

.. code-block:: bash

   # In another terminal
   python -c "from vehicle_actuator import VehicleActuator; v = VehicleActuator(); v.emergency_stop()"

Reviewing Logs
==============

After each run, logs are saved to the ``logs/`` directory:

.. code-block:: bash

   ls -la logs/

Log files include:

- ``control_log_<timestamp>.csv`` - Timestamped control commands
- ``gps_log_<timestamp>.csv`` - GPS positions and headings
- ``error_log_<timestamp>.txt`` - Any errors or warnings

Analyze a log:

.. code-block:: bash

   python scripts/analyze_log.py logs/control_log_<timestamp>.csv

Common First-Run Issues
=======================

No GPS Fix
----------

**Symptom**: Position shows NaN or doesn't update

**Solutions**:

- Ensure clear sky view (no buildings/trees blocking)
- Wait longer (cold start can take 2-3 minutes)
- Check antenna connection
- Verify Xsens device is detected: ``ls /dev/ttyUSB*``

LIDAR No Data
-------------

**Symptom**: Point cloud is empty

**Solutions**:

- Check Ethernet connection
- Verify IP configuration (default: 192.168.1.201)
- Check firewall: ``sudo ufw allow 2368/udp``

Vehicle Doesn't Respond
-----------------------

**Symptom**: Steering/throttle commands have no effect

**Solutions**:

- Check Teensy USB connection: ``ls /dev/ttyACM*``
- Verify serial permissions: ``sudo usermod -a -G dialout $USER``
- Check vehicle is in correct mode (not in physical override)

Next Steps
==========

Congratulations on your first run! Next, explore:

- :doc:`../tutorials/basic-navigation` - Detailed navigation tutorial
- :doc:`../tutorials/obstacle-avoidance` - Enable obstacle avoidance
- :doc:`../configuration/index` - Tune system parameters
- :doc:`../troubleshooting/index` - Detailed troubleshooting guide
