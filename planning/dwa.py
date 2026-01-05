"""
Dynamic Window Approach (DWA) Local Planner

Reactive local planner for obstacle avoidance using velocity space search.
Based on PythonRobotics by Atsushi Sakai.

Usage:
    from planning import DWA
    from perception import OccupancyGrid2D

    dwa = DWA()
    grid = OccupancyGrid2D()

    # Update grid with sensor data
    grid.update(lidar_points)

    # Compute velocity command
    state = (x, y, yaw, v, omega)
    goal = (goal_x, goal_y)
    v_cmd, omega_cmd = dwa.compute_velocity(state, goal, grid)
"""

import math
import numpy as np
from typing import Tuple, Optional, Union, TYPE_CHECKING

if TYPE_CHECKING:
    from perception.occupancy_grid import OccupancyGrid2D


class DWA:
    """
    Dynamic Window Approach local planner

    Uses differential drive motion model (v, omega) which works well
    for low-speed Ackermann vehicles.

    Attributes:
        max_speed: Maximum forward velocity (m/s)
        min_speed: Minimum velocity, negative for reverse (m/s)
        max_yaw_rate: Maximum angular velocity (rad/s)
        robot_radius: Robot collision radius (m)
    """

    def __init__(self, config: Optional[dict] = None):
        """
        Initialize DWA planner

        Args:
            config: Optional configuration dict with keys:
                - max_speed: Max forward speed (m/s), default 1.0
                - min_speed: Min speed (m/s), default 0.0
                - max_accel: Max acceleration (m/s²), default 0.5
                - max_yaw_rate: Max angular velocity (rad/s), default 1.0
                - max_delta_yaw_rate: Max angular acceleration (rad/s²), default 2.0
                - v_resolution: Velocity sampling resolution (m/s), default 0.05
                - yaw_rate_resolution: Angular velocity resolution (rad/s), default 0.05
                - dt: Time step (s), default 0.1
                - predict_time: Trajectory prediction horizon (s), default 2.0
                - to_goal_cost_gain: Weight for goal heading cost, default 0.15
                - speed_cost_gain: Weight for speed cost, default 1.0
                - obstacle_cost_gain: Weight for obstacle cost, default 1.0
                - robot_radius: Collision radius (m), default 0.5
        """
        cfg = config or {}

        # Motion limits
        self.max_speed = cfg.get('max_speed', 1.0)
        self.min_speed = cfg.get('min_speed', 0.0)
        self.max_accel = cfg.get('max_accel', 0.5)
        self.max_yaw_rate = cfg.get('max_yaw_rate', 1.0)
        self.max_delta_yaw_rate = cfg.get('max_delta_yaw_rate', 2.0)

        # Sampling resolution
        self.v_resolution = cfg.get('v_resolution', 0.05)
        self.yaw_rate_resolution = cfg.get('yaw_rate_resolution', 0.05)

        # Prediction
        self.dt = cfg.get('dt', 0.1)
        self.predict_time = cfg.get('predict_time', 2.0)

        # Cost weights
        self.to_goal_cost_gain = cfg.get('to_goal_cost_gain', 0.15)
        self.speed_cost_gain = cfg.get('speed_cost_gain', 1.0)
        self.obstacle_cost_gain = cfg.get('obstacle_cost_gain', 1.0)

        # Collision
        self.robot_radius = cfg.get('robot_radius', 0.5)
        self._stuck_threshold = 0.001

        # Internal obstacle storage
        self._obstacles = np.array([]).reshape(0, 2)

    def compute_velocity(
        self,
        state: Tuple[float, float, float, float, float],
        goal: Tuple[float, float],
        grid: Optional['OccupancyGrid2D'] = None,
        obstacle_threshold: float = 0.7
    ) -> Tuple[float, float]:
        """
        Compute velocity command for obstacle avoidance

        Args:
            state: Robot state (x, y, yaw, v, omega)
                - x, y: Position in meters
                - yaw: Heading in radians
                - v: Current velocity in m/s
                - omega: Current angular velocity in rad/s
            goal: Goal position (x, y) in meters
            grid: Optional OccupancyGrid2D for obstacle detection
            obstacle_threshold: Occupancy threshold for obstacles (0-1)

        Returns:
            Tuple of (velocity, angular_velocity) commands
        """
        x = np.array(state, dtype=np.float64)
        goal_arr = np.array(goal)

        # Get obstacles from grid if provided
        if grid is not None:
            obstacles = grid.get_obstacle_points(threshold=obstacle_threshold)
        else:
            obstacles = self._obstacles

        # Compute dynamic window and find best control
        dw = self._calc_dynamic_window(x)
        u, _ = self._calc_control_and_trajectory(x, dw, goal_arr, obstacles)

        return (u[0], u[1])

    def set_obstacles(self, obstacles: np.ndarray) -> None:
        """
        Set obstacle positions directly (alternative to using grid)

        Args:
            obstacles: Obstacle positions as (N, 2) array of [x, y] points
        """
        if obstacles is None or len(obstacles) == 0:
            self._obstacles = np.array([]).reshape(0, 2)
        else:
            self._obstacles = np.asarray(obstacles).reshape(-1, 2)

    def predict_trajectory(
        self,
        state: Tuple[float, float, float, float, float],
        v: float,
        omega: float
    ) -> np.ndarray:
        """
        Predict future trajectory for visualization

        Args:
            state: Current state (x, y, yaw, v, omega)
            v: Velocity command
            omega: Angular velocity command

        Returns:
            Trajectory as (N, 5) array of states
        """
        x = np.array(state, dtype=np.float64)
        return self._predict_trajectory(x, v, omega)

    def goal_reached(self, state, goal, threshold: float = 0.5) -> bool:
        """Check if robot is within threshold distance of goal"""
        return math.hypot(state[0] - goal[0], state[1] - goal[1]) < threshold

    # -------------------------------------------------------------------------
    # Internal methods
    # -------------------------------------------------------------------------

    def _motion(self, x: np.ndarray, u: list) -> np.ndarray:
        """Apply motion model: differential drive kinematics"""
        x = x.copy()
        x[2] += u[1] * self.dt  # yaw
        x[0] += u[0] * math.cos(x[2]) * self.dt  # x
        x[1] += u[0] * math.sin(x[2]) * self.dt  # y
        x[3] = u[0]  # v
        x[4] = u[1]  # omega
        return x

    def _calc_dynamic_window(self, x: np.ndarray) -> list:
        """Calculate velocity search space based on limits and current state"""
        # Robot limits
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]

        # Acceleration-constrained limits
        Vd = [x[3] - self.max_accel * self.dt,
              x[3] + self.max_accel * self.dt,
              x[4] - self.max_delta_yaw_rate * self.dt,
              x[4] + self.max_delta_yaw_rate * self.dt]

        # Intersection
        return [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
                max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    def _predict_trajectory(self, x_init: np.ndarray, v: float, omega: float) -> np.ndarray:
        """Simulate trajectory with constant control input"""
        x = x_init.copy()
        trajectory = [x.copy()]
        time = 0

        while time <= self.predict_time:
            x = self._motion(x, [v, omega])
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
        """Search velocity space for best control input"""
        x_init = x.copy()
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self._predict_trajectory(x_init, v, omega)

                # Cost function
                to_goal_cost = self.to_goal_cost_gain * self._calc_to_goal_cost(trajectory, goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self._calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, omega]
                    best_trajectory = trajectory

                    # Anti-stuck: force turn when stopped
                    if abs(best_u[0]) < self._stuck_threshold \
                            and abs(x[3]) < self._stuck_threshold:
                        best_u[1] = -self.max_delta_yaw_rate

        return best_u, best_trajectory

    def _calc_obstacle_cost(self, trajectory: np.ndarray, ob: np.ndarray) -> float:
        """Obstacle cost: infinity if collision, else inverse of min distance"""
        if len(ob) == 0:
            return 0.0

        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if np.any(r <= self.robot_radius):
            return float("inf")

        return 1.0 / np.min(r)

    def _calc_to_goal_cost(self, trajectory: np.ndarray, goal: np.ndarray) -> float:
        """Goal cost: angle between heading and goal direction"""
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        return abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    # Backward compatibility alias
    def _simulate(self, x, y, theta, v, omega):
        """Deprecated: use predict_trajectory() instead"""
        return self.predict_trajectory((x, y, theta, v, omega), v, omega)
