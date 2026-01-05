#!/usr/bin/env python3
"""
DWA Example: Synthetic Obstacles

Demonstrates DWA obstacle avoidance with synthetic circular obstacles.
Uses the occupancy grid integration.
"""

import sys
import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, '..')
from perception import OccupancyGrid2D
from planning import DWA


def add_obstacle(grid: OccupancyGrid2D, cx: float, cy: float, radius: float):
    """Add circular obstacle to grid"""
    for row in range(grid.cells_y):
        for col in range(grid.cells_x):
            x, y = grid.grid_to_world(row, col)
            if np.hypot(x - cx, y - cy) < radius:
                grid.grid[row, col] = 5.0  # High log-odds = occupied


def main():
    # Create occupancy grid (20m x 20m, 0.2m resolution)
    grid = OccupancyGrid2D(
        width=20.0,
        height=20.0,
        resolution=0.2,
        use_raycasting=False
    )

    # Add synthetic obstacles
    add_obstacle(grid, 4.0, 0.5, 1.0)   # First obstacle
    add_obstacle(grid, 6.0, -1.0, 0.8)  # Second obstacle
    add_obstacle(grid, 8.0, 1.0, 0.6)   # Third obstacle

    # Create DWA planner
    dwa = DWA(config={
        'max_speed': 1.0,
        'min_speed': -0.5,
        'max_accel': 0.2,
        'max_yaw_rate': 40.0 * math.pi / 180.0,      # 40 deg/s
        'max_delta_yaw_rate': 40.0 * math.pi / 180.0,
        'v_resolution': 0.01,
        'yaw_rate_resolution': 0.1 * math.pi / 180.0,
        'dt': 0.1,
        'predict_time': 4.0,
        'to_goal_cost_gain': 0.05,
        'speed_cost_gain': 0.5,
        'obstacle_cost_gain': 2.0,
        'robot_radius': 0.5,
    })

    print(f"Obstacles: {len(grid.get_obstacle_points())} grid cells")

    # Initial state [x, y, yaw, v, omega]
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([10.0, 0.0])

    # Trajectory history
    trajectory = [state.copy()]

    # Setup plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))

    # Precompute grid visualization
    prob = grid.to_probability_grid()
    extent = [-grid.origin_x, grid.width - grid.origin_x,
              -grid.origin_y, grid.height - grid.origin_y]

    print("Running DWA simulation... (close window to exit)")

    while plt.fignum_exists(fig.number):
        # Compute velocity command (pass grid directly)
        v_cmd, omega_cmd = dwa.compute_velocity(
            state=tuple(state),
            goal=tuple(goal),
            grid=grid,
            obstacle_threshold=0.7
        )

        # Update state with motion model
        state[2] += omega_cmd * dwa.dt
        state[0] += v_cmd * math.cos(state[2]) * dwa.dt
        state[1] += v_cmd * math.sin(state[2]) * dwa.dt
        state[3] = v_cmd
        state[4] = omega_cmd

        trajectory.append(state.copy())

        # Get predicted trajectory for visualization
        pred_traj = dwa.predict_trajectory(tuple(state), v_cmd, omega_cmd)

        # Plot
        ax.clear()
        ax.imshow(prob, origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=1)

        # Path taken
        traj_arr = np.array(trajectory)
        ax.plot(traj_arr[:, 0], traj_arr[:, 1], 'b-', linewidth=2, label='Path')

        # Predicted trajectory
        ax.plot(pred_traj[:, 0], pred_traj[:, 1], 'g-', linewidth=2, label='Predicted')

        # Robot
        circle = plt.Circle((state[0], state[1]), dwa.robot_radius,
                             color='b', fill=False, linewidth=2)
        ax.add_patch(circle)
        ax.arrow(state[0], state[1], math.cos(state[2]) * 0.8, math.sin(state[2]) * 0.8,
                 head_width=0.2, color='blue')

        # Goal
        ax.plot(goal[0], goal[1], 'g*', markersize=15, label='Goal')

        ax.set_xlim(-2, 12)
        ax.set_ylim(-5, 5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left')
        ax.set_title(f'v={v_cmd:.2f} m/s  omega={math.degrees(omega_cmd):.1f} deg/s')

        plt.pause(0.05)

        # Check goal reached
        dist = math.hypot(state[0] - goal[0], state[1] - goal[1])
        if dist < 1.0:
            print(f"Goal reached! Distance: {dist:.2f}m")
            break

    print("Done")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
