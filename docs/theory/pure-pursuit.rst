=======================
Pure Pursuit Algorithm
=======================

Geometric path tracking for autonomous vehicles.

.. contents:: Contents
   :local:
   :depth: 2

Introduction
============

Pure Pursuit is a path tracking algorithm that computes the steering angle needed to follow a path by "chasing" a point ahead on the path.

Geometric Derivation
====================

The algorithm finds the arc that connects the vehicle's rear axle to a lookahead point on the path.

.. math::

   \kappa = \frac{2 \sin(\alpha)}{L_d}

Where:

- :math:`\kappa` = curvature
- :math:`\alpha` = angle between vehicle heading and lookahead point
- :math:`L_d` = lookahead distance

The steering angle :math:`\delta` for a bicycle model:

.. math::

   \delta = \arctan(L \cdot \kappa) = \arctan\left(\frac{2 L \sin(\alpha)}{L_d}\right)

Where :math:`L` is the wheel base.

Lookahead Distance
==================

The lookahead distance affects tracking behavior:

- **Short lookahead**: Tight tracking, but oscillation prone
- **Long lookahead**: Smooth tracking, but cuts corners

Speed-adaptive lookahead:

.. math::

   L_d = K_{dd} \cdot v + L_{min}

Stability Analysis
==================

Pure Pursuit is stable when:

- Lookahead distance is greater than zero
- Path curvature is bounded
- Speed is within limits

Limitations
===========

- May cut corners on tight turns
- Cannot handle reverse driving
- Assumes bicycle model kinematics

References
==========

- Coulter, R. Craig. "Implementation of the Pure Pursuit Path Tracking Algorithm." Carnegie Mellon University, 1992.
