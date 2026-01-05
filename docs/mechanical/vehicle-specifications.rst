======================
Vehicle Specifications
======================

Physical specifications of the UTM Navigator platform.

.. contents:: Contents
   :local:
   :depth: 2

Vehicle Overview
================

The UTM Navigator is built on a [vehicle platform base]. This section documents the key physical parameters used in the control algorithms.

.. figure:: ../_static/vehicle_dimensions.png
   :alt: Vehicle Dimensions
   :align: center
   :width: 80%

   UTM Navigator dimensions diagram

Physical Dimensions
===================

+-------------------------+----------------+------------------+
| Parameter               | Value          | Notes            |
+=========================+================+==================+
| Overall Length          | ~2.0-2.5 m     | Bumper to bumper |
+-------------------------+----------------+------------------+
| Overall Width           | ~1.0-1.8 m     | Mirror to mirror |
+-------------------------+----------------+------------------+
| Overall Height          | TBD            | To roof          |
+-------------------------+----------------+------------------+
| Wheel Base              | **1.23 m**     | Front to rear axle|
+-------------------------+----------------+------------------+
| Track Width (Front)     | TBD            | Center to center |
+-------------------------+----------------+------------------+
| Track Width (Rear)      | TBD            | Center to center |
+-------------------------+----------------+------------------+
| Ground Clearance        | TBD            | Minimum          |
+-------------------------+----------------+------------------+
| Curb Weight             | TBD            | Without payload  |
+-------------------------+----------------+------------------+

.. note::

   The wheel base of **1.23 m** is a critical parameter used in the steering controller. Verify this measurement on your specific vehicle.

Kinematic Parameters
====================

These parameters define the vehicle's motion capabilities and are used by the planning and control algorithms.

Speed Limits
------------

+-------------------------+----------------+------------------+
| Parameter               | Value          | Notes            |
+=========================+================+==================+
| Maximum Speed           | 5.0 m/s        | ~18 km/h         |
+-------------------------+----------------+------------------+
| Typical Operating Speed | 2.0-3.0 m/s    | ~7-11 km/h       |
+-------------------------+----------------+------------------+
| Minimum Speed (DWA)     | 0.0 m/s        | Can stop         |
+-------------------------+----------------+------------------+

Acceleration Limits
-------------------

+---------------------------+----------------+------------------+
| Parameter                 | Value          | Notes            |
+===========================+================+==================+
| Maximum Acceleration      | 2.0 m/s²       | Forward          |
+---------------------------+----------------+------------------+
| Maximum Deceleration      | 3.0 m/s²       | Braking          |
+---------------------------+----------------+------------------+
| Emergency Deceleration    | 5.0+ m/s²      | Full brake       |
+---------------------------+----------------+------------------+

Steering Geometry
-----------------

+---------------------------+----------------+------------------+
| Parameter                 | Value          | Notes            |
+===========================+================+==================+
| Maximum Steering Angle    | ±28°           | At wheels        |
+---------------------------+----------------+------------------+
| Steering Ratio            | ~14:1          | Wheel to road    |
+---------------------------+----------------+------------------+
| Maximum Yaw Rate          | 1.0 rad/s      | ~57°/s           |
+---------------------------+----------------+------------------+
| Minimum Turning Radius    | ~2.5 m         | At max steering  |
+---------------------------+----------------+------------------+

Turning Radius Calculation
--------------------------

The minimum turning radius is calculated from:

.. math::

   R_{min} = \frac{L}{\tan(\delta_{max})}

Where:

- :math:`L` = Wheel base (1.23 m)
- :math:`\delta_{max}` = Maximum steering angle (28°)

.. code-block:: python

   import math
   wheel_base = 1.23  # meters
   max_steering = 28  # degrees
   min_radius = wheel_base / math.tan(math.radians(max_steering))
   # Result: ~2.3 meters

Collision Geometry
==================

For obstacle avoidance, the vehicle is modeled with the following parameters:

+-------------------------+----------------+------------------+
| Parameter               | Value          | Notes            |
+=========================+================+==================+
| Robot Radius            | 0.8 m          | Circular approx  |
+-------------------------+----------------+------------------+
| Safety Margin           | 0.3 m          | Additional buffer|
+-------------------------+----------------+------------------+
| Effective Radius        | 1.1 m          | For planning     |
+-------------------------+----------------+------------------+

For more precise collision checking:

+-------------------------+----------------+
| Parameter               | Value          |
+=========================+================+
| Robot Length            | 2.0 m          |
+-------------------------+----------------+
| Robot Width             | 1.5 m          |
+-------------------------+----------------+

Weight Distribution
===================

+-------------------------+----------------+
| Location                | Weight         |
+=========================+================+
| Front Axle              | TBD kg         |
+-------------------------+----------------+
| Rear Axle               | TBD kg         |
+-------------------------+----------------+
| Total                   | TBD kg         |
+-------------------------+----------------+

Tire Specifications
===================

+-------------------------+----------------+
| Parameter               | Value          |
+=========================+================+
| Tire Size               | TBD            |
+-------------------------+----------------+
| Tire Pressure           | TBD psi        |
+-------------------------+----------------+
| Rolling Radius          | TBD m          |
+-------------------------+----------------+

Configuration in Code
=====================

These parameters are defined in the configuration files:

**DWA Planner** (``config/dwa_config.yaml``):

.. code-block:: yaml

   vehicle:
     max_speed: 5.0          # m/s
     max_yaw_rate: 1.0       # rad/s
     max_accel: 2.0          # m/s²
     max_yaw_rate_accel: 3.0 # rad/s²
     robot_radius: 0.8       # m

**Pure Pursuit Controller** (``pure_pursuit_controller.py``):

.. code-block:: python

   WHEEL_BASE = 1.23  # meters

Measurement Procedure
=====================

To verify or update these parameters:

Wheel Base
----------

1. Place vehicle on level ground
2. Mark center of front wheel contact patch
3. Mark center of rear wheel contact patch
4. Measure distance between marks

Turning Radius
--------------

1. Turn steering to full lock
2. Drive slowly in a circle
3. Measure radius to vehicle center

Weight
------

1. Use vehicle scales under each wheel
2. Sum front wheels for front axle weight
3. Sum rear wheels for rear axle weight
