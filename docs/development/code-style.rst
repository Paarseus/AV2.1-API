==========
Code Style
==========

Coding standards for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Python Style
============

We follow PEP 8 with some modifications:

- Line length: 100 characters
- Use type hints for public functions
- Use docstrings for all public classes and methods

Formatting
----------

Use ``black`` for automatic formatting:

.. code-block:: bash

   black --line-length 100 .

Linting
-------

Use ``flake8`` for linting:

.. code-block:: bash

   flake8 --max-line-length 100 .

Docstrings
==========

Use NumPy style docstrings:

.. code-block:: python

   def compute_steering(self, waypoints, speed):
       """
       Compute steering angle using pure pursuit.

       Parameters
       ----------
       waypoints : list of tuple
           List of (x, y) waypoints in vehicle frame
       speed : float
           Current vehicle speed in m/s

       Returns
       -------
       float
           Steering angle in radians
       """

Type Hints
==========

.. code-block:: python

   from typing import List, Tuple, Optional

   def find_waypoint(
       waypoints: List[Tuple[float, float]],
       distance: float
   ) -> Optional[Tuple[float, float]]:
       ...

Naming Conventions
==================

- Classes: ``PascalCase``
- Functions/methods: ``snake_case``
- Constants: ``UPPER_SNAKE_CASE``
- Private members: ``_leading_underscore``
