#!/usr/bin/env python3
"""
Ackermann DWA Example: Synthetic Obstacles
"""

import sys
import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, '..')
from perception import OccupancyGrid2D
from planning import AckermannDWA


def add_obstacle(grid, cx, cy, radius):
    """Add circular obstacle to grid"""
    for row in range(grid.cells_y):
        for col in range(grid.cells_x):
            x, y = grid.grid_to_world(row, col)
            if np.hypot(x - cx, y - cy) < radius:
                grid.grid[row, col] = 5.0


def main():
    # Grid
    grid = OccupancyGrid2D(width=20.0, height=20.0, resolution=0.2, use_raycasting=False)

    # Obstacles
    add_obstacle(grid, 4.0, 0.5, 1.0)
    add_obstacle(grid, 6.0, -1.0, 0.8)
    add_obstacle(grid, 8.0, 1.0, 0.6)

    # Ackermann DWA - uses bicycle model kinematics
    dwa = AckermannDWA(config={
        'max_speed': 1.0,
        'wheelbase': 1.5,
        'max_steering': math.radians(35),
        'max_steering_rate': math.radians(60),
        'safety_margin': 0.1,
    })

    # State [x, y, yaw, v, steering]
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([10.0, 0.0])

    trajectory = [state.copy()]

    # Plot setup
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))
    prob = grid.to_probability_grid()
    extent = [-grid.origin_x, grid.width - grid.origin_x,
              -grid.origin_y, grid.height - grid.origin_y]

    print(f"Wheelbase: {dwa.wheelbase}m")
    print(f"Max steering: {math.degrees(dwa.max_steering):.1f} deg")
    print(f"Goal: {goal}")
    print("Running... (close window to exit)")

    while plt.fignum_exists(fig.number):
        # Ackermann DWA
        v, steering = dwa.compute_velocity(tuple(state), tuple(goal), grid=grid)

        # Bicycle model update
        if abs(v) > 0.001:
            yaw_rate = v * math.tan(steering) / dwa.wheelbase
        else:
            yaw_rate = 0.0

        state[2] += yaw_rate * dwa.dt
        state[0] += v * math.cos(state[2]) * dwa.dt
        state[1] += v * math.sin(state[2]) * dwa.dt
        state[3] = v
        state[4] = steering

        trajectory.append(state.copy())

        # Predicted path
        pred = dwa.predict_trajectory(tuple(state), v, steering)

        # Plot
        ax.clear()
        ax.imshow(prob, origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=1)

        traj_arr = np.array(trajectory)
        ax.plot(traj_arr[:, 0], traj_arr[:, 1], 'b-', lw=2, label='Path')
        ax.plot(pred[:, 0], pred[:, 1], 'g-', lw=2, label='Predicted')

        # Robot
        circle = plt.Circle((state[0], state[1]), dwa.robot_radius, color='b', fill=False, lw=2)
        ax.add_patch(circle)
        ax.arrow(state[0], state[1], math.cos(state[2]) * 0.8, math.sin(state[2]) * 0.8,
                 head_width=0.2, color='blue')

        # Goal
        ax.plot(goal[0], goal[1], 'g*', ms=15, label='Goal')

        ax.set_xlim(-2, 12)
        ax.set_ylim(-5, 5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper left')
        ax.set_title(f'v={v:.2f} m/s  steering={math.degrees(steering):.1f} deg')

        plt.pause(0.05)

        # Goal check
        if dwa.goal_reached(tuple(state), tuple(goal), threshold=1.0):
            print("Goal reached!")
            break

    print("Done")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
