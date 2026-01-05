========================
Dynamic Window Approach
========================

Local obstacle avoidance through velocity space search.

.. contents:: Contents
   :local:
   :depth: 2

Introduction
============

The Dynamic Window Approach (DWA) is a local motion planning algorithm that selects optimal velocity commands by searching the velocity space.

Algorithm Overview
==================

1. Compute the dynamic window (reachable velocities)
2. Sample candidate velocities
3. Simulate trajectories for each candidate
4. Evaluate trajectories with objective function
5. Select best velocity

Dynamic Window
==============

The dynamic window constrains velocities to those reachable within one time step:

.. math::

   V_d = \{(v, \omega) | v \in [v_c - \dot{v}_{max} \cdot \Delta t, v_c + \dot{v}_{max} \cdot \Delta t]\}

Combined with velocity limits:

.. math::

   V_s = \{(v, \omega) | v \in [0, v_{max}], \omega \in [-\omega_{max}, \omega_{max}]\}

Trajectory Simulation
=====================

For each velocity sample, predict the trajectory:

.. math::

   x_{t+1} = x_t + v \cos(\theta_t) \cdot \Delta t

   y_{t+1} = y_t + v \sin(\theta_t) \cdot \Delta t

   \theta_{t+1} = \theta_t + \omega \cdot \Delta t

Objective Function
==================

.. math::

   G(v, \omega) = \alpha \cdot heading(v, \omega) + \beta \cdot clearance(v, \omega) + \gamma \cdot velocity(v, \omega)

- **heading**: Alignment with goal direction
- **clearance**: Distance to nearest obstacle
- **velocity**: Preference for higher speeds

Admissible Velocities
=====================

Velocities that allow stopping before collision:

.. math::

   v \leq \sqrt{2 \cdot dist(v, \omega) \cdot \dot{v}_b}

References
==========

- Fox, D., Burgard, W., & Thrun, S. (1997). "The dynamic window approach to collision avoidance."
