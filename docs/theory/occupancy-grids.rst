=========================
Bayesian Occupancy Grids
=========================

Probabilistic obstacle mapping.

.. contents:: Contents
   :local:
   :depth: 2

Introduction
============

Occupancy grids represent the environment as a 2D grid where each cell contains the probability of being occupied.

Probability Representation
==========================

Each cell stores :math:`P(m)` - the probability the cell is occupied.

- :math:`P(m) = 0`: Definitely free
- :math:`P(m) = 0.5`: Unknown
- :math:`P(m) = 1`: Definitely occupied

Log-Odds Representation
=======================

For numerical stability, we use log-odds:

.. math::

   l(m) = \log\frac{P(m)}{1 - P(m)}

Conversion:

.. math::

   P(m) = 1 - \frac{1}{1 + e^{l(m)}}

Bayesian Update
===============

Given a sensor measurement :math:`z`:

.. math::

   l(m|z_{1:t}) = l(m|z_{1:t-1}) + l(m|z_t) - l_0

Where :math:`l_0 = \log\frac{p_0}{1-p_0}` is the prior.

Simplified (with uniform prior):

.. math::

   l_{new} = l_{old} + l_{sensor}

Temporal Decay
==============

To handle dynamic environments, apply decay:

.. math::

   l_{t+1} = \gamma \cdot l_t

Where :math:`\gamma < 1` causes old observations to fade.

References
==========

- Elfes, A. (1989). "Using occupancy grids for mobile robot perception and navigation."
- Thrun, S. (2002). "Robotic mapping: A survey."
