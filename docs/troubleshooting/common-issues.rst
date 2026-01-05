=============
Common Issues
=============

Solutions for frequently encountered problems.

.. contents:: Issues
   :local:
   :depth: 2

GPS Issues
==========

No GPS Fix
----------

**Symptoms**: Position shows NaN or doesn't update

**Solutions**:

1. Ensure clear sky view
2. Wait 2-3 minutes for cold start
3. Check antenna connection
4. Verify device detected: ``ls /dev/ttyUSB*``

RTK Not Converging
------------------

**Symptoms**: RTK status stays "Float" or "None"

**Solutions**:

1. Check base station connection
2. Verify NTRIP credentials
3. Ensure baseline < 10 km
4. Wait up to 5 minutes for convergence

LIDAR Issues
============

No Point Cloud Data
-------------------

**Symptoms**: Empty scan array

**Solutions**:

1. Check Ethernet cable
2. Verify IP configuration
3. Allow UDP port: ``sudo ufw allow 2368/udp``
4. Ping LIDAR: ``ping 192.168.1.201``

Sparse Point Cloud
------------------

**Symptoms**: Fewer points than expected

**Solutions**:

1. Check for obstructions
2. Clean sensor lens
3. Verify rotation is active

Control Issues
==============

Vehicle Doesn't Move
--------------------

**Solutions**:

1. Check drive mode is "D"
2. Verify emergency stop released
3. Check Teensy connection
4. Review throttle commands in logs

Steering Oscillation
--------------------

**Solutions**:

1. Increase ``min_lookahead``
2. Decrease ``K_dd`` gain
3. Check for GPS noise

Software Issues
===============

Import Errors
-------------

**Solutions**:

1. Verify virtual environment active
2. Reinstall dependencies
3. Check Python version
