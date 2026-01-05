"""
Dynamic Window Approach for Ackermann Vehicles

Uses kinematic bicycle model with (v, steering_angle) control.

Bicycle Model Equations:
    dx/dt = v * cos(theta)
    dy/dt = v * sin(theta)
    dtheta/dt = v * tan(delta) / L

Where:
    - (x, y): rear axle position
    - theta: heading angle
    - v: velocity
    - delta: front wheel steering angle
    - L: wheelbase

Reference: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
"""

import math
import numpy as np
from typing import Tuple, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from perception.occupancy_grid import OccupancyGrid2D


class AckermannDWA:
    """
    Dynamic Window Approach for Ackermann steering vehicles.

    State: [x, y, yaw, v, steering]
    Control: [v, steering_angle]

    Unlike differential drive DWA which uses (v, omega), this uses
    (v, steering_angle) and computes yaw rate from the bicycle model.
    """

    def __init__(self, config: Optional[dict] = None):
        """
        Initialize Ackermann DWA planner.

        Args:
            config: Configuration dict with keys:
                Motion limits:
                    - max_speed: Max forward speed (m/s), default 1.0
                    - min_speed: Min speed (m/s), default 0.0
                    - max_accel: Max acceleration (m/s^2), default 0.5

                Ackermann geometry:
                    - wheelbase: Distance between axles (m), default 1.5
                    - max_steering: Max steering angle (rad), default 0.6
                    - max_steering_rate: Max steering rate (rad/s), default 1.0

                Sampling:
                    - v_resolution: Velocity sampling step (m/s), default 0.02
                    - steering_resolution: Steering sampling step (rad), default 0.035

                Prediction:
                    - dt: Time step (s), default 0.1
                    - predict_time: Prediction horizon (s), default 3.0

                Cost weights:
                    - to_goal_cost_gain: Heading to goal weight, default 0.01
                    - speed_cost_gain: Speed maximization weight, default 1.0
                    - obstacle_cost_gain: Obstacle avoidance weight, default 2.0
                    - goal_dist_cost_gain: Distance to goal weight, default 0.3

                Collision:
                    - robot_radius: Collision check radius (m), default 0.5
                    - safety_margin: Extra clearance beyond robot_radius (m), default 0.5
        """
        cfg = config or {}

        # Motion limits
        self.max_speed = cfg.get('max_speed', 1.0)
        self.min_speed = cfg.get('min_speed', 0.0)
        self.max_accel = cfg.get('max_accel', 0.5)

        # Ackermann geometry
        self.wheelbase = cfg.get('wheelbase', 1.5)
        self.max_steering = cfg.get('max_steering', 0.6)  # ~35 deg
        self.max_steering_rate = cfg.get('max_steering_rate', 1.0)  # ~60 deg/s

        # Sampling resolution
        self.v_resolution = cfg.get('v_resolution', 0.02)
        self.steering_resolution = cfg.get('steering_resolution', 0.035)  # ~2 deg

        # Prediction
        self.dt = cfg.get('dt', 0.1)
        self.predict_time = cfg.get('predict_time', 3.0)

        # Cost weights (tuned for Ackermann - differs from differential drive)
        self.to_goal_cost_gain = cfg.get('to_goal_cost_gain', 0.01)
        self.speed_cost_gain = cfg.get('speed_cost_gain', 1.0)
        self.obstacle_cost_gain = cfg.get('obstacle_cost_gain', 2.0)
        self.goal_dist_cost_gain = cfg.get('goal_dist_cost_gain', 0.3)

        # Collision parameters
        self.robot_radius = cfg.get('robot_radius', 0.5)
        self.safety_margin = cfg.get('safety_margin', 0.5)

        # Internal
        self._obstacles = np.array([]).reshape(0, 2)
        self._stuck_count = 0

    def compute_velocity(
        self,
        state: Tuple[float, float, float, float, float],
        goal: Tuple[float, float],
        grid: Optional['OccupancyGrid2D'] = None,
        obstacle_threshold: float = 0.7
    ) -> Tuple[float, float]:
        """
        Compute velocity command using DWA.

        Args:
            state: (x, y, yaw, v, steering) - current robot state
            goal: (x, y) - target position
            grid: Optional OccupancyGrid2D for obstacle detection
            obstacle_threshold: Probability threshold for obstacles (0-1)

        Returns:
            (v, steering_angle) - velocity commands
        """
        x = np.array(state, dtype=np.float64)
        goal_arr = np.array(goal)

        # Get obstacles from grid or internal storage
        if grid is not None:
            obstacles = grid.get_obstacle_points(threshold=obstacle_threshold)
        else:
            obstacles = self._obstacles

        # Compute dynamic window and find best control
        dw = self._calc_dynamic_window(x)
        u, _ = self._calc_control_and_trajectory(x, dw, goal_arr, obstacles)

        return (u[0], u[1])

    def set_obstacles(self, obstacles: np.ndarray) -> None:
        """Set obstacle positions directly (alternative to grid)."""
        if obstacles is None or len(obstacles) == 0:
            self._obstacles = np.array([]).reshape(0, 2)
        else:
            self._obstacles = np.asarray(obstacles).reshape(-1, 2)

    def predict_trajectory(
        self,
        state: Tuple[float, float, float, float, float],
        v: float,
        steering: float
    ) -> np.ndarray:
        """Predict trajectory for given control input (for visualization)."""
        x = np.array(state, dtype=np.float64)
        return self._predict_trajectory(x, v, steering)

    def goal_reached(self, state, goal, threshold: float = 0.5) -> bool:
        """Check if robot is within threshold distance of goal."""
        return math.hypot(state[0] - goal[0], state[1] - goal[1]) < threshold

    # -------------------------------------------------------------------------
    # Internal methods
    # -------------------------------------------------------------------------

    def _motion(self, x: np.ndarray, u: list) -> np.ndarray:
        """
        Apply bicycle model kinematics.

        Equations:
            yaw_rate = v * tan(steering) / wheelbase
            x += v * cos(yaw) * dt
            y += v * sin(yaw) * dt
            yaw += yaw_rate * dt
        """
        x = x.copy()
        v, steering = u[0], u[1]

        # Compute yaw rate from bicycle model
        if abs(v) > 1e-6:
            yaw_rate = v * math.tan(steering) / self.wheelbase
        else:
            yaw_rate = 0.0

        # Update state (semi-implicit Euler integration)
        x[2] += yaw_rate * self.dt
        x[0] += v * math.cos(x[2]) * self.dt
        x[1] += v * math.sin(x[2]) * self.dt
        x[3] = v
        x[4] = steering

        return x

    def _calc_dynamic_window(self, x: np.ndarray) -> list:
        """
        Calculate reachable velocity window.

        Returns intersection of:
        - Robot limits: [min_speed, max_speed] x [-max_steering, max_steering]
        - Acceleration limits: reachable from current state in one dt
        """
        # Robot physical limits
        v_min_abs, v_max_abs = self.min_speed, self.max_speed
        s_min_abs, s_max_abs = -self.max_steering, self.max_steering

        # Acceleration-constrained limits
        v_min_dyn = x[3] - self.max_accel * self.dt
        v_max_dyn = x[3] + self.max_accel * self.dt
        s_min_dyn = x[4] - self.max_steering_rate * self.dt
        s_max_dyn = x[4] + self.max_steering_rate * self.dt

        # Return intersection
        return [
            max(v_min_abs, v_min_dyn),
            min(v_max_abs, v_max_dyn),
            max(s_min_abs, s_min_dyn),
            min(s_max_abs, s_max_dyn)
        ]

    def _predict_trajectory(self, x_init: np.ndarray, v: float, steering: float) -> np.ndarray:
        """Simulate trajectory with constant control over prediction horizon."""
        x = x_init.copy()
        trajectory = [x.copy()]
        time = 0.0

        while time <= self.predict_time:
            x = self._motion(x, [v, steering])
            trajectory.append(x.copy())
            time += self.dt

        return np.array(trajectory)

    def _calc_control_and_trajectory(
        self,
        x: np.ndarray,
        dw: list,
        goal: np.ndarray,
        ob: np.ndarray
    ) -> Tuple[list, np.ndarray]:
        """Search velocity space for optimal control."""
        x_init = x.copy()
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # Sample velocity space
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for steering in np.arange(dw[2], dw[3], self.steering_resolution):
                trajectory = self._predict_trajectory(x_init, v, steering)

                # Compute cost components
                heading_cost = self.to_goal_cost_gain * self._calc_heading_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - v)
                obstacle_cost = self.obstacle_cost_gain * self._calc_obstacle_cost(trajectory, ob)
                dist_cost = self.goal_dist_cost_gain * self._calc_dist_cost(trajectory, goal)

                total_cost = heading_cost + speed_cost + obstacle_cost + dist_cost

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [v, steering]
                    best_trajectory = trajectory

        # Anti-stuck: if no velocity found, try to escape
        if abs(best_u[0]) < 1e-4 and abs(x[3]) < 1e-4:
            self._stuck_count += 1
            if self._stuck_count > 3:
                # Alternate steering direction to escape
                direction = -1 if self._stuck_count % 2 == 0 else 1
                best_u = [self.v_resolution * 3, direction * self.max_steering * 0.5]
        else:
            self._stuck_count = 0

        return best_u, best_trajectory

    def _calc_obstacle_cost(self, trajectory: np.ndarray, ob: np.ndarray) -> float:
        """
        Compute obstacle cost.

        Returns:
            - inf if trajectory collides with obstacle
            - 1/min_distance otherwise (closer = higher cost)
        """
        if len(ob) == 0:
            return 0.0

        # Compute distances from all trajectory points to all obstacles
        # Shape: (N_obstacles, N_trajectory_points)
        dx = trajectory[:, 0] - ob[:, 0][:, np.newaxis]
        dy = trajectory[:, 1] - ob[:, 1][:, np.newaxis]
        distances = np.hypot(dx, dy)

        # Check collision with safety margin
        clearance = self.robot_radius + self.safety_margin
        if np.any(distances <= clearance):
            return float("inf")

        # Return inverse of minimum distance
        return 1.0 / np.min(distances)

    def _calc_heading_cost(self, trajectory: np.ndarray, goal: np.ndarray) -> float:
        """
        Compute heading cost (angle between final heading and goal direction).

        Lower cost when robot is pointing toward goal.
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        goal_angle = math.atan2(dy, dx)
        heading_error = goal_angle - trajectory[-1, 2]
        # Normalize angle to [-pi, pi]
        return abs(math.atan2(math.sin(heading_error), math.cos(heading_error)))

    def _calc_dist_cost(self, trajectory: np.ndarray, goal: np.ndarray) -> float:
        """
        Compute distance to goal cost.

        Lower cost when trajectory endpoint is closer to goal.
        This is critical for Ackermann vehicles to favor progress over heading.
        """
        return math.hypot(goal[0] - trajectory[-1, 0], goal[1] - trajectory[-1, 1])
