============
Architecture
============

Detailed software architecture of the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

System Overview
===============

The UTM Navigator is built as a modular, single-process Python application with multiple threads for sensor acquisition and visualization.

Design Principles
-----------------

1. **Separation of Concerns**: Each module handles one responsibility
2. **Thread Safety**: Shared state protected by locks
3. **Configurability**: Parameters in YAML/code, not hardcoded
4. **Testability**: Components can be tested in isolation

Module Hierarchy
================

.. code-block:: text

   utm-navigator/
   │
   ├── runner.py                 # Main entry point and control loop
   │
   ├── Sensors
   │   ├── xsens_class.py        # GPS/IMU interface
   │   └── sensors/
   │       ├── sensor_interface.py   # Abstract base class
   │       ├── lidar_interface.py    # Velodyne LIDAR
   │       └── camera_interface.py   # RGB/RGBD cameras
   │
   ├── Perception
   │   └── perception/
   │       ├── occupancy_grid.py     # Bayesian grid mapping
   │       ├── state.py              # Thread-safe state container
   │       ├── core_types.py         # Data structures
   │       ├── preprocessing/
   │       │   ├── ipm_processor.py  # Inverse perspective mapping
   │       │   └── yolopv2_adapter.py # Deep learning detector
   │       └── visualization/
   │           ├── occupancy_visualizer.py
   │           └── open3d_viewer.py
   │
   ├── Planning
   │   ├── osmnx_class.py        # Route planning
   │   └── planning/
   │       └── dwa_planner.py    # Dynamic Window Approach
   │
   ├── Control
   │   ├── pure_pursuit_controller.py
   │   └── vehicle_actuator.py
   │
   └── Configuration
       └── config/
           └── dwa_config.yaml

Main Control Loop
=================

The ``AutonomousRunner`` class in ``runner.py`` orchestrates all components:

.. code-block:: python

   class AutonomousRunner:
       def __init__(self):
           self.vehicle_state = VehicleState()
           self.gps = None
           self.navigator = None
           self.controller = None
           self.actuator = None

       def setup_gps(self, require_rtk=True, rtk_timeout=60):
           """Initialize GPS/IMU and wait for fix."""
           self.gps = XsensReceiver()
           # Wait for RTK fix...

       def setup_navigation(self, dest_lat, dest_lon):
           """Plan route to destination."""
           self.navigator = Navigator()
           self.waypoints = self.navigator.plan_route(
               start=(current_lat, current_lon),
               end=(dest_lat, dest_lon)
           )

       def setup_controller(self):
           """Initialize steering controller."""
           self.controller = PurePursuitController(wheel_base=1.23)
           self.actuator = VehicleActuator()

       def control_loop(self):
           """Main control iteration."""
           # 1. Get current state
           position = self.gps.get_position()
           heading = self.gps.get_heading()
           speed = self.gps.get_speed()

           # 2. Transform waypoints to vehicle frame
           local_waypoints = self.transform_waypoints(
               self.waypoints, position, heading
           )

           # 3. Compute steering
           steering = self.controller.compute_steering(
               local_waypoints, speed
           )

           # 4. Send commands
           self.actuator.set_steering(steering)
           self.actuator.set_throttle(target_throttle)

       def run(self, update_hz=50):
           """Run control loop at specified rate."""
           dt = 1.0 / update_hz
           while self.running:
               start = time.time()
               self.control_loop()
               elapsed = time.time() - start
               if elapsed < dt:
                   time.sleep(dt - elapsed)

Thread-Safe State
=================

VehicleState
------------

Thread-safe container for vehicle state:

.. code-block:: python

   class VehicleState:
       def __init__(self):
           self._lock = threading.Lock()
           self._position = (0.0, 0.0)  # (x, y) in UTM
           self._heading = 0.0          # radians from North
           self._speed = 0.0            # m/s

       def update(self, position, heading, speed):
           with self._lock:
               self._position = position
               self._heading = heading
               self._speed = speed

       def get_state(self):
           with self._lock:
               return (self._position, self._heading, self._speed)

PerceptionState
---------------

Thread-safe container for perception data:

.. code-block:: python

   class PerceptionState:
       def __init__(self):
           self._lock = threading.Lock()
           self._occupancy_grid = None
           self._detections = []
           self._lane_offset = 0.0

       def update_grid(self, grid):
           with self._lock:
               self._occupancy_grid = grid.copy()

       def get_grid(self):
           with self._lock:
               return self._occupancy_grid.copy() if self._occupancy_grid else None

Sensor Abstraction
==================

All sensors implement a common interface:

.. code-block:: python

   class SensorInterface(ABC):
       @abstractmethod
       def connect(self) -> bool:
           """Connect to sensor hardware."""
           pass

       @abstractmethod
       def disconnect(self) -> None:
           """Disconnect from sensor."""
           pass

       @abstractmethod
       def is_connected(self) -> bool:
           """Check connection status."""
           pass

       @abstractmethod
       def get_data(self) -> Any:
           """Get latest sensor data."""
           pass

This allows easy substitution of sensors (e.g., different LIDAR models).

Configuration System
====================

YAML Configuration
------------------

Complex parameters are stored in YAML files:

.. code-block:: yaml

   # config/dwa_config.yaml
   vehicle:
     max_speed: 5.0
     max_yaw_rate: 1.0
     robot_radius: 0.8

   costs:
     goal_cost_gain: 0.2
     obstacle_cost_gain: 1.0

Loading configuration:

.. code-block:: python

   import yaml

   with open('config/dwa_config.yaml') as f:
       config = yaml.safe_load(f)

   planner = DWAPlanner(
       max_speed=config['vehicle']['max_speed'],
       robot_radius=config['vehicle']['robot_radius']
   )

Dataclasses
-----------

Configuration containers use dataclasses:

.. code-block:: python

   from dataclasses import dataclass

   @dataclass
   class DWAConfig:
       max_speed: float = 5.0
       max_yaw_rate: float = 1.0
       max_accel: float = 2.0
       robot_radius: float = 0.8
       dt: float = 0.1
       predict_time: float = 3.0

Error Handling
==============

Sensor Failures
---------------

.. code-block:: python

   try:
       position = self.gps.get_position()
       if position is None:
           raise SensorError("GPS position unavailable")
   except SensorError as e:
       self.logger.warning(f"Sensor error: {e}")
       self.enter_safe_mode()

Safe Mode
---------

When errors occur, the system enters safe mode:

1. Reduce speed to minimum
2. Apply brakes gradually
3. Alert operator
4. Attempt sensor recovery

Logging
=======

The system uses Python's logging module:

.. code-block:: python

   import logging

   logging.basicConfig(level=logging.INFO)
   logger = logging.getLogger('utm_navigator')

   logger.info("Starting autonomous navigation")
   logger.warning("RTK fix lost, using standard GPS")
   logger.error("LIDAR communication timeout")

CSV data logging for analysis:

.. code-block:: python

   class ControlLogger:
       def __init__(self, filename):
           self.file = open(filename, 'w')
           self.writer = csv.writer(self.file)
           self.writer.writerow(['time', 'x', 'y', 'heading', 'speed', 'steering'])

       def log(self, state, steering):
           self.writer.writerow([
               time.time(),
               state.x, state.y,
               state.heading,
               state.speed,
               steering
           ])

Performance Considerations
==========================

Timing
------

- Control loop target: 50-100 Hz
- Perception processing: 10-30 Hz
- Visualization: 10-30 Hz (optional)

Bottlenecks
-----------

1. **YOLOPv2 inference**: GPU-bound, ~50ms per frame
2. **LIDAR processing**: CPU-bound point cloud operations
3. **Visualization**: Can be disabled for performance

Optimization Tips
-----------------

- Use NumPy vectorized operations
- Profile with ``cProfile`` to find hotspots
- Disable visualization during performance-critical tests
- Consider TensorRT for YOLOPv2 optimization
