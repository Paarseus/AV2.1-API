=======
Control
=======

Control systems for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The control system translates planned paths into actuator commands:

- **Pure Pursuit**: Geometric path tracking for steering
- **Velocity Control**: Speed and throttle management
- **Vehicle Actuator**: Hardware interface

Pure Pursuit Controller
=======================

Pure pursuit is a geometric path tracking algorithm that computes steering angle to follow a path.

Algorithm
---------

1. Find a lookahead point on the path ahead of the vehicle
2. Compute the curvature needed to reach that point
3. Convert curvature to steering angle

.. code-block:: text

                    Lookahead Point
                         ●
                        /|
                       / |
                      /  |
                     /   | L_d (lookahead distance)
                    /    |
                   /  α  |
                  ●──────┘
              Vehicle

   Steering = atan2(2 × L × sin(α), L_d)

   Where:
   - L = wheel base
   - α = angle to lookahead point
   - L_d = lookahead distance

Implementation
--------------

.. code-block:: python

   class PurePursuitController:
       def __init__(self, wheel_base=1.23):
           self.wheel_base = wheel_base
           self.K_dd = 0.5        # Lookahead gain
           self.min_lookahead = 3.0
           self.max_lookahead = 20.0

       def compute_steering(self, waypoints, speed):
           """
           Compute steering angle to follow waypoints.

           Args:
               waypoints: List of (x, y) in vehicle frame
               speed: Current vehicle speed (m/s)

           Returns:
               steering: Steering angle in radians
           """
           # Compute speed-adaptive lookahead
           lookahead = self.K_dd * speed + self.min_lookahead
           lookahead = np.clip(lookahead, self.min_lookahead, self.max_lookahead)

           # Find lookahead point
           target = self.find_lookahead_point(waypoints, lookahead)
           if target is None:
               return 0.0

           # Compute steering angle
           target_x, target_y = target
           alpha = np.arctan2(target_y, target_x)
           steering = np.arctan2(
               2 * self.wheel_base * np.sin(alpha),
               lookahead
           )

           return steering

       def find_lookahead_point(self, waypoints, lookahead):
           """Find point on path at lookahead distance."""
           for i, (x, y) in enumerate(waypoints):
               dist = np.sqrt(x**2 + y**2)
               if dist >= lookahead:
                   return (x, y)

           # Return last waypoint if none at lookahead distance
           return waypoints[-1] if waypoints else None

Parameters
----------

+------------------+---------+----------------------------------+
| Parameter        | Default | Description                      |
+==================+=========+==================================+
| wheel_base       | 1.23    | Vehicle wheel base (m)           |
+------------------+---------+----------------------------------+
| K_dd             | 0.5     | Lookahead distance gain          |
+------------------+---------+----------------------------------+
| min_lookahead    | 3.0     | Minimum lookahead distance (m)   |
+------------------+---------+----------------------------------+
| max_lookahead    | 20.0    | Maximum lookahead distance (m)   |
+------------------+---------+----------------------------------+

Tuning Guide
------------

**Lookahead Distance**:

- Too short: Oscillation, overshooting turns
- Too long: Cutting corners, slow response

**K_dd (Speed Gain)**:

- Higher: Smoother at high speeds
- Lower: More responsive at low speeds

Velocity Control
================

Speed management for throttle and braking.

Simple Proportional Control
---------------------------

.. code-block:: python

   class VelocityController:
       def __init__(self, Kp=0.5, max_throttle=0.5, max_brake=1.0):
           self.Kp = Kp
           self.max_throttle = max_throttle
           self.max_brake = max_brake

       def compute(self, target_speed, current_speed):
           """Compute throttle/brake command."""
           error = target_speed - current_speed

           if error > 0:
               # Need to accelerate
               throttle = min(self.Kp * error, self.max_throttle)
               brake = 0.0
           else:
               # Need to decelerate
               throttle = 0.0
               brake = min(-self.Kp * error, self.max_brake)

           return throttle, brake

PID Control
-----------

For smoother control:

.. code-block:: python

   class PIDVelocityController:
       def __init__(self, Kp=0.5, Ki=0.1, Kd=0.05):
           self.Kp = Kp
           self.Ki = Ki
           self.Kd = Kd
           self.integral = 0.0
           self.prev_error = 0.0

       def compute(self, target_speed, current_speed, dt):
           error = target_speed - current_speed

           self.integral += error * dt
           derivative = (error - self.prev_error) / dt
           self.prev_error = error

           output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

           if output > 0:
               return min(output, 1.0), 0.0  # throttle, brake
           else:
               return 0.0, min(-output, 1.0)

Vehicle Actuator
================

Interface to the vehicle's actuators via Teensy CAN master.

Connection
----------

.. code-block:: python

   class VehicleActuator:
       def __init__(self, port='/dev/ttyACM0', baudrate=115200):
           self.serial = serial.Serial(port, baudrate, timeout=0.1)
           self.connected = True

       def _send(self, command):
           """Send command to Teensy."""
           self.serial.write(f"{command}\n".encode())

Commands
--------

.. code-block:: python

   def set_throttle(self, value):
       """Set throttle (0.0 to 1.0)."""
       value = np.clip(value, 0.0, 1.0)
       self._send(f"T {value:.3f}")

   def set_brake(self, value):
       """Set brake (0.0 to 1.0)."""
       value = np.clip(value, 0.0, 1.0)
       self._send(f"B {value:.3f}")

   def set_steering(self, value):
       """Set steering (-1.0 to 1.0)."""
       value = np.clip(value, -1.0, 1.0)
       self._send(f"S {value:.3f}")

   def set_steering_degrees(self, degrees):
       """Set steering in degrees (-28 to 28)."""
       degrees = np.clip(degrees, -28, 28)
       normalized = degrees / 28.0
       self.set_steering(normalized)

   def set_drive_mode(self, mode):
       """Set drive mode (N, D, S, R)."""
       if mode in ['N', 'D', 'S', 'R']:
           self._send(f"M {mode}")

   def emergency_stop(self):
       """Activate emergency stop."""
       self._send("E 1")

   def reset_emergency_stop(self):
       """Reset emergency stop."""
       self._send("E 0")

Safety Features
---------------

.. code-block:: python

   class SafeVehicleActuator(VehicleActuator):
       def __init__(self, *args, **kwargs):
           super().__init__(*args, **kwargs)
           self.last_command_time = time.time()
           self.watchdog_timeout = 0.2  # seconds

       def check_watchdog(self):
           """Check if commands are being sent regularly."""
           if time.time() - self.last_command_time > self.watchdog_timeout:
               self.emergency_stop()

       def set_throttle(self, value):
           self.last_command_time = time.time()
           super().set_throttle(value)

Control Loop Integration
========================

Complete control loop:

.. code-block:: python

   class Controller:
       def __init__(self):
           self.path_tracker = PurePursuitController()
           self.velocity_ctrl = VelocityController()
           self.actuator = VehicleActuator()

       def control_step(self, waypoints, target_speed, current_speed):
           # Compute steering
           steering = self.path_tracker.compute_steering(
               waypoints, current_speed
           )

           # Compute throttle/brake
           throttle, brake = self.velocity_ctrl.compute(
               target_speed, current_speed
           )

           # Send commands
           self.actuator.set_steering_degrees(np.degrees(steering))

           if brake > 0:
               self.actuator.set_throttle(0)
               self.actuator.set_brake(brake)
           else:
               self.actuator.set_brake(0)
               self.actuator.set_throttle(throttle)
