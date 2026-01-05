==========
Perception
==========

Perception system for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The perception system processes sensor data to understand the environment:

- **Occupancy Grid**: 2D probabilistic obstacle map from LIDAR
- **YOLOPv2**: Object detection and lane segmentation from camera
- **IPM**: Inverse perspective mapping for bird's-eye view

Occupancy Grid
==============

The occupancy grid maintains a 2D probabilistic map of obstacles around the vehicle.

Algorithm
---------

**Bayesian Log-Odds Update**

The grid uses log-odds representation for numerical stability:

.. math::

   l(m|z) = l(m) + \log\frac{p(m|z)}{1-p(m|z)}

Where:
- :math:`l(m)` = log-odds of cell m being occupied
- :math:`z` = sensor measurement

**Update Process**:

1. Receive LIDAR point cloud
2. Filter points by height (ground removal)
3. Transform to grid coordinates
4. Update log-odds for occupied cells
5. Apply temporal decay

Implementation
--------------

.. code-block:: python

   class OccupancyGrid:
       def __init__(self, size=40.0, resolution=0.1):
           self.size = size
           self.resolution = resolution
           self.grid_size = int(size / resolution)
           self.grid = np.zeros((self.grid_size, self.grid_size))

           # Log-odds parameters
           self.log_odds_hit = 0.4
           self.log_odds_miss = -0.2
           self.decay_factor = 0.95

       def update_from_lidar(self, points):
           """Update grid from LIDAR points."""
           # Filter by height
           mask = (points[:, 2] > -0.3) & (points[:, 2] < 0.5)
           filtered = points[mask]

           # Convert to grid coordinates
           grid_x = ((filtered[:, 0] / self.resolution) +
                     self.grid_size // 2).astype(int)
           grid_y = ((filtered[:, 1] / self.resolution) +
                     self.grid_size // 2).astype(int)

           # Clip to grid bounds
           valid = ((grid_x >= 0) & (grid_x < self.grid_size) &
                    (grid_y >= 0) & (grid_y < self.grid_size))
           grid_x = grid_x[valid]
           grid_y = grid_y[valid]

           # Update log-odds
           self.grid[grid_x, grid_y] += self.log_odds_hit

           # Apply decay
           self.grid *= self.decay_factor

           # Clip values
           np.clip(self.grid, -5.0, 5.0, out=self.grid)

       def to_probability(self):
           """Convert log-odds to probability."""
           return 1.0 / (1.0 + np.exp(-self.grid))

       def get_closest_obstacle(self, direction):
           """Find closest obstacle in given direction."""
           prob = self.to_probability()
           # Search along direction...

Parameters
----------

+-------------------+---------+--------------------------------+
| Parameter         | Default | Description                    |
+===================+=========+================================+
| size              | 40.0    | Grid size in meters            |
+-------------------+---------+--------------------------------+
| resolution        | 0.1     | Meters per cell                |
+-------------------+---------+--------------------------------+
| log_odds_hit      | 0.4     | Increment for occupied         |
+-------------------+---------+--------------------------------+
| log_odds_miss     | -0.2    | Decrement for free             |
+-------------------+---------+--------------------------------+
| decay_factor      | 0.95    | Temporal decay per frame       |
+-------------------+---------+--------------------------------+
| height_min        | -0.3    | Min height filter (meters)     |
+-------------------+---------+--------------------------------+
| height_max        | 0.5     | Max height filter (meters)     |
+-------------------+---------+--------------------------------+

YOLOPv2 Detection
=================

YOLOPv2 is a multi-task deep learning model for driving perception.

Tasks
-----

1. **Object Detection**: Vehicles, pedestrians, cyclists
2. **Drivable Area Segmentation**: Road surface mask
3. **Lane Line Segmentation**: Lane markings mask

Architecture
------------

.. code-block:: text

   Input Image (640×640×3)
         │
         ▼
   ┌─────────────────┐
   │    Backbone     │  (CSPDarknet)
   │   (Shared)      │
   └────────┬────────┘
            │
      ┌─────┼─────┐
      │     │     │
      ▼     ▼     ▼
   ┌─────┐ ┌─────┐ ┌─────┐
   │Det. │ │Drive│ │Lane │
   │Head │ │Head │ │Head │
   └──┬──┘ └──┬──┘ └──┬──┘
      │       │       │
      ▼       ▼       ▼
   Boxes   Mask    Mask

Usage
-----

.. code-block:: python

   from perception.preprocessing.yolopv2_adapter import YOLOPv2Adapter

   # Initialize
   detector = YOLOPv2Adapter(model_path='weights/YOLOPv2.pt')

   # Process frame
   frame = camera.get_frame()
   output = detector.process(frame)

   # Access results
   detections = output.detections      # List of Detection objects
   drivable_mask = output.drivable_mask  # Binary mask
   lane_mask = output.lane_mask        # Binary mask

Output Format
-------------

.. code-block:: python

   @dataclass
   class Detection:
       class_id: int
       class_name: str
       confidence: float
       bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2

   @dataclass
   class YOLOPv2Output:
       detections: List[Detection]
       drivable_mask: np.ndarray  # H×W binary
       lane_mask: np.ndarray      # H×W binary

Performance
-----------

- **Input Size**: 640×640
- **Inference Time**: ~50ms on GTX 1060 (GPU)
- **Inference Time**: ~500ms on CPU (not recommended)

Inverse Perspective Mapping
===========================

IPM transforms the camera image to a bird's-eye view (BEV).

Theory
------

Using camera intrinsics and extrinsics, we compute a homography matrix that maps image pixels to ground plane coordinates.

.. math::

   \begin{bmatrix} x_{bev} \\ y_{bev} \\ 1 \end{bmatrix} = H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}

Where H is the homography matrix.

Usage
-----

.. code-block:: python

   from perception.preprocessing.ipm_processor import IPMProcessor

   # Initialize with camera calibration
   ipm = IPMProcessor(
       camera_matrix=K,
       dist_coeffs=D,
       camera_height=1.2,
       camera_pitch=-10  # degrees
   )

   # Transform image
   bev_image = ipm.transform(image)

Configuration
-------------

.. code-block:: python

   @dataclass
   class BEVConfig:
       output_width: int = 400
       output_height: int = 400
       meters_per_pixel: float = 0.05
       roi_front: float = 20.0   # meters ahead
       roi_rear: float = 5.0     # meters behind
       roi_side: float = 10.0    # meters to each side

Perception Pipeline Integration
===============================

The complete perception pipeline:

.. code-block:: python

   class PerceptionPipeline:
       def __init__(self):
           self.occupancy_grid = OccupancyGrid()
           self.yolo = YOLOPv2Adapter()
           self.ipm = IPMProcessor()
           self.state = PerceptionState()

       def process(self, lidar_points, camera_frame):
           # Update occupancy grid from LIDAR
           self.occupancy_grid.update_from_lidar(lidar_points)

           # Run YOLOPv2 on camera image
           yolo_output = self.yolo.process(camera_frame)

           # Generate bird's-eye view
           bev = self.ipm.transform(camera_frame)

           # Update shared state
           self.state.update(
               grid=self.occupancy_grid,
               detections=yolo_output.detections,
               bev=bev
           )

           return self.state
