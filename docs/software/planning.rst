========
Planning
========

Path planning systems for the UTM Navigator.

.. contents:: Contents
   :local:
   :depth: 2

Overview
========

The planning system operates at two levels:

1. **Global Planning**: Route from current position to destination (OSMnx)
2. **Local Planning**: Obstacle avoidance and trajectory optimization (DWA)

Route Planning (OSMnx)
======================

The Navigator class handles global route planning using OpenStreetMap data.

How It Works
------------

1. Download road network graph for the area
2. Find nearest graph nodes to start and end points
3. Compute shortest path using Dijkstra's algorithm
4. Interpolate waypoints along the path

Usage
-----

.. code-block:: python

   from osmnx_class import Navigator

   # Initialize
   nav = Navigator()

   # Plan route
   waypoints = nav.plan_route(
       start=(34.0577, -117.8215),  # lat, lon
       end=(34.0590, -117.8200),
       spacing=2.0  # meters between waypoints
   )

   # waypoints is list of (x, y) in UTM coordinates

Implementation
--------------

.. code-block:: python

   class Navigator:
       def __init__(self):
           self.transformer = Transformer.from_crs(
               "EPSG:4326",  # WGS84
               "EPSG:32611"  # UTM Zone 11N
           )

       def plan_route(self, start, end, spacing=2.0):
           # Get road network
           G = ox.graph_from_point(start, dist=1000, network_type='drive')

           # Find nearest nodes
           start_node = ox.nearest_nodes(G, start[1], start[0])
           end_node = ox.nearest_nodes(G, end[1], end[0])

           # Compute shortest path
           path = nx.shortest_path(G, start_node, end_node, weight='length')

           # Get coordinates
           coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in path]

           # Convert to UTM
           utm_coords = [self.transformer.transform(lat, lon)
                         for lat, lon in coords]

           # Interpolate waypoints
           waypoints = self.interpolate(utm_coords, spacing)

           return waypoints

Coordinate Systems
------------------

- **Input**: WGS84 (latitude, longitude)
- **Internal**: UTM meters (easting, northing)
- **Output**: UTM coordinates for vehicle control

Dynamic Window Approach (DWA)
=============================

DWA is a local planner that selects optimal velocity commands while avoiding obstacles.

Algorithm Overview
------------------

.. code-block:: text

   For each time step:
   1. Compute dynamic window (reachable velocities)
   2. Sample velocities in the window
   3. For each (v, ω) sample:
      a. Predict trajectory for T seconds
      b. Check for collisions
      c. Compute cost (goal + speed + obstacles)
   4. Select velocity with minimum cost

Dynamic Window
--------------

The dynamic window defines reachable velocities given current state and limits:

.. math::

   V_s = \{(v, \omega) | v \in [v_{min}, v_{max}], \omega \in [\omega_{min}, \omega_{max}]\}

   V_d = \{(v, \omega) | v \in [v_c - \dot{v}_{max} \cdot dt, v_c + \dot{v}_{max} \cdot dt]\}

   V = V_s \cap V_d

Cost Function
-------------

.. math::

   G(v, \omega) = \alpha \cdot heading(v, \omega) + \beta \cdot dist(v, \omega) + \gamma \cdot velocity(v, \omega)

Where:

- ``heading``: Alignment with goal direction
- ``dist``: Clearance to nearest obstacle
- ``velocity``: Preference for higher speeds

Implementation
--------------

.. code-block:: python

   class DWAPlanner:
       def __init__(self, config: DWAConfig):
           self.config = config

       def plan(self, state, goal, obstacles):
           """
           Args:
               state: [x, y, θ, v, ω]
               goal: (x, y) target position
               obstacles: Nx2 array of obstacle positions

           Returns:
               (v, ω): Best velocity command
           """
           best_v, best_omega = 0, 0
           min_cost = float('inf')

           # Get dynamic window
           v_min, v_max, omega_min, omega_max = self.calc_dynamic_window(state)

           # Sample velocities
           for v in np.arange(v_min, v_max, self.config.v_resolution):
               for omega in np.arange(omega_min, omega_max,
                                       self.config.omega_resolution):
                   # Predict trajectory
                   traj = self.predict_trajectory(state, v, omega)

                   # Check collision
                   if self.check_collision(traj, obstacles):
                       continue

                   # Compute cost
                   cost = self.calc_cost(traj, goal, obstacles)

                   if cost < min_cost:
                       min_cost = cost
                       best_v, best_omega = v, omega

           return best_v, best_omega

       def predict_trajectory(self, state, v, omega):
           """Predict trajectory for given velocity."""
           x, y, theta, _, _ = state
           trajectory = []

           for t in np.arange(0, self.config.predict_time, self.config.dt):
               x += v * np.cos(theta) * self.config.dt
               y += v * np.sin(theta) * self.config.dt
               theta += omega * self.config.dt
               trajectory.append([x, y, theta])

           return np.array(trajectory)

Configuration
-------------

.. code-block:: yaml

   # config/dwa_config.yaml
   vehicle:
     max_speed: 5.0          # m/s
     min_speed: 0.0          # m/s
     max_yaw_rate: 1.0       # rad/s
     max_accel: 2.0          # m/s²
     max_yaw_rate_accel: 3.0 # rad/s²
     robot_radius: 0.8       # m

   sampling:
     v_resolution: 0.2       # m/s
     yaw_rate_resolution: 0.1 # rad/s

   prediction:
     dt: 0.1                 # seconds
     predict_time: 2.0       # seconds

   costs:
     goal_cost_gain: 1.0
     speed_cost_gain: 0.5
     obstacle_cost_gain: 2.0

Tuning Guide
------------

**Goal Cost Gain**

- Increase: More aggressive goal seeking
- Decrease: More conservative, prioritizes safety

**Speed Cost Gain**

- Increase: Prefers higher speeds
- Decrease: More cautious speed selection

**Obstacle Cost Gain**

- Increase: Larger avoidance margins
- Decrease: Closer passes near obstacles

**Prediction Time**

- Increase: Plans further ahead, smoother but slower
- Decrease: More reactive, faster but may oscillate

Path Integration
================

Global and local planning are integrated:

.. code-block:: python

   class PathPlanner:
       def __init__(self):
           self.global_planner = Navigator()
           self.local_planner = DWAPlanner(DWAConfig())

       def plan(self, current_pos, current_heading, goal, obstacles):
           # Get global waypoints (cached/updated periodically)
           if self.needs_replan(current_pos):
               self.waypoints = self.global_planner.plan_route(
                   current_pos, goal
               )

           # Get local target (next waypoint)
           local_goal = self.get_next_waypoint(current_pos)

           # Run DWA for local planning
           state = [*current_pos, current_heading, self.current_v, self.current_omega]
           v, omega = self.local_planner.plan(state, local_goal, obstacles)

           return v, omega
