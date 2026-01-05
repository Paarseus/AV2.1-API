===================
Tuning Controllers
===================

Optimize controller performance for your environment.

.. contents:: Contents
   :local:
   :depth: 2

Pure Pursuit Tuning
===================

Key Parameters
--------------

- **K_dd**: Lookahead distance gain
- **min_lookahead**: Minimum lookahead distance
- **max_lookahead**: Maximum lookahead distance

Symptoms and Solutions
----------------------

**Oscillation / Weaving**

- Increase ``min_lookahead``
- Decrease ``K_dd``

**Cutting Corners**

- Decrease ``min_lookahead``
- Increase ``K_dd``

**Slow Response**

- Decrease ``min_lookahead``

DWA Tuning
==========

Cost Weight Adjustment
----------------------

Start with default values and adjust one at a time:

1. **goal_cost_gain**: Adjust first for path following
2. **obstacle_cost_gain**: Tune for safety margins
3. **speed_cost_gain**: Final adjustment for smoothness

Logging for Analysis
====================

Enable detailed logging:

.. code-block:: python

   from control_logger import ControlLogger

   logger = ControlLogger('tuning_log.csv')

   while running:
       # ... control loop ...
       logger.log(state, steering, throttle)

Analyze with:

.. code-block:: bash

   python scripts/plot_control_log.py tuning_log.csv
