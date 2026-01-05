#!/usr/bin/env python3
"""
DWA Example: Real LIDAR Integration

Demonstrates DWA with live Velodyne LIDAR data.
Updates occupancy grid in real-time and computes velocity commands.
"""

import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, '..')
from sensors.lidar_interface import VelodyneLIDAR, LidarState
from perception import OccupancyGrid2D
from planning import DWA


def main():
    # Connect to LIDAR
    print("Connecting to LIDAR...")
    lidar = VelodyneLIDAR(port=2368)
    if not lidar.connect() or not lidar.start():
        print("LIDAR connection failed")
        return 1

    lidar_state = LidarState(lidar)

    # Wait for data
    print("Waiting for LIDAR data...")
    for _ in range(25):
        if lidar_state.get_points() is not None:
            print("Data received")
            break
        time.sleep(0.2)
    else:
        print("No data received")
        lidar.stop()
        lidar.disconnect()
        return 1

    # Create occupancy grid
    grid = OccupancyGrid2D(
        width=20.0,
        height=20.0,
        resolution=0.2,
        decay_factor=0.5,
        use_raycasting=False
    )

    # Create DWA planner
    dwa = DWA(config={
        'max_speed': 1.0,
        'min_speed': 0.0,
        'max_accel': 0.5,
        'max_yaw_rate': 1.0,
        'max_delta_yaw_rate': 2.0,
        'v_resolution': 0.05,
        'yaw_rate_resolution': 0.1,
        'dt': 0.1,
        'predict_time': 2.0,
        'to_goal_cost_gain': 0.15,
        'speed_cost_gain': 1.0,
        'obstacle_cost_gain': 1.0,
        'robot_radius': 1.0,
    })

    # State [x, y, yaw, v, omega] - robot at origin
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([8.0, 0.0])  # 8m ahead

    # Setup plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))

    print(f"Running DWA with LIDAR (goal: {goal})")
    print("Close window to exit")

    try:
        while plt.fignum_exists(fig.number):
            # Update grid from LIDAR
            points = lidar_state.get_points()
            if points is not None:
                grid.update(points[:, :2])

            # Compute velocity (grid passed directly)
            v_cmd, omega_cmd = dwa.compute_velocity(
                state=tuple(state),
                goal=tuple(goal),
                grid=grid,
                obstacle_threshold=0.6
            )

            # Get predicted trajectory for visualization
            pred_traj = dwa.predict_trajectory(tuple(state), v_cmd, omega_cmd)

            # Plot
            ax.clear()

            # Occupancy grid
            prob = grid.to_probability_grid()
            extent = [-grid.origin_x, grid.width - grid.origin_x,
                      -grid.origin_y, grid.height - grid.origin_y]
            ax.imshow(prob, origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=1)

            # Predicted trajectory
            ax.plot(pred_traj[:, 0], pred_traj[:, 1], 'g-', linewidth=2, label='Predicted')

            # Robot
            circle = plt.Circle((state[0], state[1]), dwa.robot_radius,
                                 color='b', fill=False)
            ax.add_patch(circle)
            ax.arrow(state[0], state[1], math.cos(state[2]), math.sin(state[2]),
                     head_width=0.2, color='blue')

            # Goal
            ax.plot(goal[0], goal[1], 'g*', markersize=15, label='Goal')

            obstacles = grid.get_obstacle_points(threshold=0.6)
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.set_title(f'v={v_cmd:.2f} m/s  omega={math.degrees(omega_cmd):.1f} deg/s  obstacles={len(obstacles)}')

            plt.pause(0.1)

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        lidar.stop()
        lidar.disconnect()
        plt.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
