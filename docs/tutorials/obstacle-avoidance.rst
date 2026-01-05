==================
Obstacle Avoidance
==================

Enable and configure obstacle avoidance for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The UTM Navigator uses LIDAR-based obstacle detection with the Dynamic Window Approach (DWA) for local obstacle avoidance.

Enabling Obstacle Avoidance
===========================

Step 1: Verify LIDAR
--------------------

.. code-block:: bash

   python -c "from sensors.lidar_interface import VelodyneLIDAR; l = VelodyneLIDAR(); print(len(l.get_scan()))"

Should show > 0 points.

Step 2: Enable in Runner
------------------------

Edit ``runner.py`` or use command line flag:

.. code-block:: bash

   python runner.py --lat 34.0580 --lon -117.8210 --enable-dwa

Step 3: Configure Parameters
----------------------------

Edit ``config/dwa_config.yaml``:

.. code-block:: yaml

   costs:
     obstacle_cost_gain: 2.0  # Increase for more caution

Testing
=======

Static Obstacles
----------------

1. Place cardboard boxes in path
2. Run navigation
3. Vehicle should navigate around obstacles

Dynamic Obstacles
-----------------

1. Have person walk across path (carefully!)
2. Vehicle should detect and avoid

Tuning
======

If too conservative (large detours):

- Decrease ``obstacle_cost_gain``
- Decrease ``robot_radius``

If too aggressive (close passes):

- Increase ``obstacle_cost_gain``
- Increase ``robot_radius``
- Increase safety margin
