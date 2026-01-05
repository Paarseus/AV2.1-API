"""
Dynamic Window Approach for Ackermann Vehicles (Costmap Version)

Uses Costmap for collision checking instead of OccupancyGrid2D.
Simpler than the original - robot radius is baked into the costmap.

Usage:
    from perception import OccupancyGrid2D, Costmap
    from planning import AckermannDWACostmap

    grid = OccupancyGrid2D()
    costmap = Costmap(robot_radius=0.5, safety_margin=0.2)
    dwa = AckermannDWACostmap()

    # In control loop
    grid.update(lidar_points)
    costmap.update(grid)
    v, steering = dwa.compute_velocity(state, goal, costmap)
"""

import math
import numpy as np
from typing import Tuple, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from perception.costmap import Costmap


class AckermannDWACostmap:
    """
    DWA for Ackermann vehicles using Costmap.

    State: [x, y, yaw, v, steering]
    Control: [v, steering_angle]
    """

    def __init__(self, config: Optional[dict] = None):
        """
        Args:
            config: Configuration dict with keys:
                Motion:
                    - max_speed: Max forward speed (m/s), default 1.0
                    - min_speed: Min speed (m/s), default 0.0
                    - max_accel: Max acceleration (m/s^2), default 0.5
                    - wheelbase: Distance between axles (m), default 1.5
                    - max_steering: Max steering angle (rad), default 0.6
                    - max_steering_rate: Max steering rate (rad/s), default 1.0

                Sampling:
                    - v_resolution: Velocity step (m/s), default 0.02
                    - steering_resolution: Steering step (rad), default 0.035

                Prediction:
                    - dt: Time step (s), default 0.1
                    - predict_time: Horizon (s), default 3.0

                Cost weights:
                    - heading_cost_gain: default 0.01
                    - speed_cost_gain: default 1.0
                    - obstacle_cost_gain: default 2.0
                    - dist_cost_gain: default 0.3
        """
        cfg = config or {}

        # Motion limits
        self.max_speed = cfg.get('max_speed', 1.0)
        self.min_speed = cfg.get('min_speed', 0.0)
        self.max_accel = cfg.get('max_accel', 0.5)

        # Ackermann geometry
        self.wheelbase = cfg.get('wheelbase', 1.5)
        self.max_steering = cfg.get('max_steering', 0.6)
        self.max_steering_rate = cfg.get('max_steering_rate', 1.0)

        # Sampling
        self.v_resolution = cfg.get('v_resolution', 0.02)
        self.steering_resolution = cfg.get('steering_resolution', 0.035)

        # Prediction
        self.dt = cfg.get('dt', 0.1)
        self.predict_time = cfg.get('predict_time', 3.0)

        # Cost weights (matches AckermannDWA)
        self.heading_cost_gain = cfg.get('heading_cost_gain', 0.01)
        self.speed_cost_gain = cfg.get('speed_cost_gain', 1.0)
        self.obstacle_cost_gain = cfg.get('obstacle_cost_gain', 2.0)
        self.dist_cost_gain = cfg.get('dist_cost_gain', 0.3)

        self._stuck_count = 0

    def compute_velocity(
        self,
        state: Tuple[float, float, float, float, float],
        goal: Tuple[float, float],
        costmap: 'Costmap'
    ) -> Tuple[float, float]:
        """
        Compute velocity command using DWA.

        Args:
            state: (x, y, yaw, v, steering)
            goal: (x, y) target position
            costmap: Costmap for collision checking

        Returns:
            (v, steering_angle)
        """
        x = np.array(state, dtype=np.float64)
        goal_arr = np.array(goal)

        dw = self._calc_dynamic_window(x)
        best_u, _ = self._search_best_control(x, dw, goal_arr, costmap)

        return (best_u[0], best_u[1])

    def predict_trajectory(
        self,
        state: Tuple[float, float, float, float, float],
        v: float,
        steering: float
    ) -> np.ndarray:
        """Predict trajectory for visualization."""
        x = np.array(state, dtype=np.float64)
        return self._simulate(x, v, steering)

    def goal_reached(self, state, goal, threshold: float = 0.5) -> bool:
        """Check if robot reached goal."""
        return math.hypot(state[0] - goal[0], state[1] - goal[1]) < threshold

    # -------------------------------------------------------------------------
    # Internal
    # -------------------------------------------------------------------------

    def _motion(self, x: np.ndarray, v: float, steering: float) -> np.ndarray:
        """Bicycle model kinematics."""
        x = x.copy()

        if abs(v) > 1e-6:
            yaw_rate = v * math.tan(steering) / self.wheelbase
        else:
            yaw_rate = 0.0

        x[2] += yaw_rate * self.dt
        x[0] += v * math.cos(x[2]) * self.dt
        x[1] += v * math.sin(x[2]) * self.dt
        x[3] = v
        x[4] = steering

        return x

    def _calc_dynamic_window(self, x: np.ndarray) -> list:
        """Calculate reachable velocity window."""
        return [
            max(self.min_speed, x[3] - self.max_accel * self.dt),
            min(self.max_speed, x[3] + self.max_accel * self.dt),
            max(-self.max_steering, x[4] - self.max_steering_rate * self.dt),
            min(self.max_steering, x[4] + self.max_steering_rate * self.dt)
        ]

    def _simulate(self, x: np.ndarray, v: float, steering: float) -> np.ndarray:
        """Simulate trajectory with constant control."""
        trajectory = [x.copy()]
        t = 0.0

        while t <= self.predict_time:
            x = self._motion(x, v, steering)
            trajectory.append(x.copy())
            t += self.dt

        return np.array(trajectory)

    def _search_best_control(
        self,
        x: np.ndarray,
        dw: list,
        goal: np.ndarray,
        costmap: 'Costmap'
    ) -> Tuple[list, np.ndarray]:
        """Search velocity space for best control."""
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_traj = np.array([x])

        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for steering in np.arange(dw[2], dw[3], self.steering_resolution):
                traj = self._simulate(x.copy(), v, steering)

                # Compute obstacle cost (includes collision check)
                obstacle = self._obstacle_cost(traj, costmap)
                if obstacle == float("inf"):
                    continue  # Collision

                heading = self._heading_cost(traj, goal)
                speed = self.max_speed - v
                dist = self._dist_cost(traj, goal)

                cost = (self.heading_cost_gain * heading +
                        self.speed_cost_gain * speed +
                        self.obstacle_cost_gain * obstacle +
                        self.dist_cost_gain * dist)

                if cost < min_cost:
                    min_cost = cost
                    best_u = [v, steering]
                    best_traj = traj

        # Anti-stuck
        if abs(best_u[0]) < 1e-4 and abs(x[3]) < 1e-4:
            self._stuck_count += 1
            if self._stuck_count > 3:
                direction = -1 if self._stuck_count % 2 == 0 else 1
                best_u = [self.v_resolution * 3, direction * self.max_steering * 0.5]
        else:
            self._stuck_count = 0

        return best_u, best_traj

    def _heading_cost(self, traj: np.ndarray, goal: np.ndarray) -> float:
        """Angle between final heading and goal direction."""
        dx = goal[0] - traj[-1, 0]
        dy = goal[1] - traj[-1, 1]
        goal_angle = math.atan2(dy, dx)
        error = goal_angle - traj[-1, 2]
        return abs(math.atan2(math.sin(error), math.cos(error)))

    def _dist_cost(self, traj: np.ndarray, goal: np.ndarray) -> float:
        """Distance from trajectory end to goal."""
        return math.hypot(goal[0] - traj[-1, 0], goal[1] - traj[-1, 1])

    def _obstacle_cost(self, traj: np.ndarray, costmap: 'Costmap') -> float:
        """
        Obstacle cost using costmap.

        Collision check uses inflated costmap (robot radius baked in).
        Gradient cost uses raw obstacle positions for 1/min_distance.

        Returns:
            - inf if trajectory collides with inflated costmap
            - 1/min_distance to raw obstacles otherwise
        """
        # Check collision using inflated costmap
        for i in range(0, len(traj), 2):
            if costmap.check_collision(traj[i, 0], traj[i, 1]):
                return float("inf")

        # Get raw obstacle points for gradient cost
        obstacles = costmap.get_obstacle_points()
        if len(obstacles) == 0:
            return 0.0

        # Compute min distance from trajectory to raw obstacles
        dx = traj[:, 0] - obstacles[:, 0][:, np.newaxis]
        dy = traj[:, 1] - obstacles[:, 1][:, np.newaxis]
        distances = np.hypot(dx, dy)
        min_dist = np.min(distances)

        return 1.0 / max(min_dist, 0.1)
