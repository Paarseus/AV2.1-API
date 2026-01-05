=================
DWA Configuration
=================

Configuration guide for the Dynamic Window Approach planner.

.. contents:: Contents
   :local:
   :depth: 2

Configuration File
==================

Location: ``config/dwa_config.yaml``

.. code-block:: yaml

   # DWA Planner Configuration

   vehicle:
     max_speed: 5.0          # Maximum forward speed (m/s)
     min_speed: 0.0          # Minimum speed (m/s)
     max_yaw_rate: 1.0       # Maximum rotation rate (rad/s)
     max_accel: 2.0          # Maximum acceleration (m/s²)
     max_yaw_rate_accel: 3.0 # Maximum rotation acceleration (rad/s²)
     robot_radius: 0.8       # Collision radius (m)

   sampling:
     v_resolution: 0.2       # Velocity sampling resolution (m/s)
     yaw_rate_resolution: 0.1 # Yaw rate sampling resolution (rad/s)

   prediction:
     dt: 0.1                 # Time step for trajectory prediction (s)
     predict_time: 2.0       # Prediction horizon (s)

   costs:
     goal_cost_gain: 1.0     # Weight for goal heading cost
     speed_cost_gain: 0.5    # Weight for speed cost
     obstacle_cost_gain: 2.0 # Weight for obstacle avoidance cost

Parameter Reference
===================

Vehicle Parameters
------------------

**max_speed** (float)
   Maximum forward velocity in meters per second.

   - Default: 5.0
   - Range: 0.0 - 10.0
   - Higher values allow faster travel but require more stopping distance

**robot_radius** (float)
   Radius used for collision checking in meters.

   - Default: 0.8
   - Should be >= actual vehicle radius + safety margin

Cost Weights
------------

**goal_cost_gain** (float)
   Weight for alignment with goal direction.

   - Higher: More aggressive goal seeking
   - Lower: More conservative path selection

**obstacle_cost_gain** (float)
   Weight for obstacle clearance.

   - Higher: Larger margins around obstacles
   - Lower: Closer passes permitted

Tuning Guide
============

For Narrow Spaces
-----------------

.. code-block:: yaml

   vehicle:
     max_speed: 2.0
     robot_radius: 0.5
   costs:
     obstacle_cost_gain: 3.0

For Open Areas
--------------

.. code-block:: yaml

   vehicle:
     max_speed: 5.0
   prediction:
     predict_time: 3.0
   costs:
     speed_cost_gain: 1.0
