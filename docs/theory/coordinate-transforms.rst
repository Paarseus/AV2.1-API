=======================
Coordinate Transforms
=======================

Coordinate system transformations for robotics.

.. contents:: Contents
   :local:
   :depth: 2

Homogeneous Coordinates
=======================

Points in 3D are represented as 4D vectors:

.. math::

   \mathbf{p} = \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}

Transformation Matrices
=======================

A 4×4 transformation matrix combines rotation and translation:

.. math::

   T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}

Where :math:`R` is a 3×3 rotation matrix and :math:`t` is translation.

Rotation Matrices
=================

Rotation about Z-axis (yaw):

.. math::

   R_z(\theta) = \begin{bmatrix}
   \cos\theta & -\sin\theta & 0 \\
   \sin\theta & \cos\theta & 0 \\
   0 & 0 & 1
   \end{bmatrix}

UTM Projection
==============

Convert WGS84 to UTM:

.. code-block:: python

   from pyproj import Transformer

   transformer = Transformer.from_crs("EPSG:4326", "EPSG:32611")
   x, y = transformer.transform(lat, lon)

Vehicle Frame Transform
=======================

Transform world coordinates to vehicle frame:

.. math::

   \mathbf{p}_{vehicle} = R(-\theta)(\mathbf{p}_{world} - \mathbf{p}_{vehicle\_origin})

Where :math:`\theta` is the vehicle heading.
