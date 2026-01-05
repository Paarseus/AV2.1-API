==================
Vehicle Parameters
==================

Vehicle-specific parameters for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Core Parameters
===============

These parameters must be accurate for correct control behavior.

Wheel Base
----------

Distance from front to rear axle center.

**Current Value**: 1.23 m

**Location in Code**: ``pure_pursuit_controller.py``

.. code-block:: python

   WHEEL_BASE = 1.23  # meters

**How to Measure**:

1. Mark center of front wheel contact patch
2. Mark center of rear wheel contact patch
3. Measure distance between marks

Maximum Steering Angle
----------------------

Maximum wheel angle at full steering lock.

**Current Value**: ±28°

**How to Measure**:

1. Turn steering to full lock
2. Measure wheel angle from straight ahead

Speed Limits
============

+-------------------+-------+-------------+
| Parameter         | Value | Location    |
+===================+=======+=============+
| Max Speed         | 5.0   | dwa_config  |
+-------------------+-------+-------------+
| Test Speed Limit  | 2.0   | runner.py   |
+-------------------+-------+-------------+
| Max Reverse Speed | 2.0   | runner.py   |
+-------------------+-------+-------------+

Collision Geometry
==================

For obstacle avoidance calculations.

.. code-block:: yaml

   # config/vehicle.yaml
   collision:
     robot_radius: 0.8      # Circular approximation
     robot_length: 2.0      # For rectangular check
     robot_width: 1.5
     safety_margin: 0.3     # Additional buffer

Updating Parameters
===================

When modifying vehicle parameters:

1. Update configuration files
2. Update any hardcoded values in source
3. Test in simulation first
4. Test at low speed in safe environment
5. Document changes
