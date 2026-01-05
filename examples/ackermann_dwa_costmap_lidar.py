#!/usr/bin/env python3
"""
Ackermann DWA Costmap Example: Live LIDAR

Demonstrates AckermannDWA with Costmap using live Velodyne LIDAR data.
Vehicle navigates toward a waypoint while avoiding detected obstacles.
"""

import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, '..')
from sensors.lidar_interface import VelodyneLIDAR, LidarState
from perception import OccupancyGrid2D, Costmap
from planning import AckermannDWACostmap


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

    # Occupancy grid
    grid = OccupancyGrid2D(
        width=20.0,
        height=20.0,
        resolution=0.2,
        decay_factor=0.5,
        use_raycasting=False
    )

    # Costmap with robot dimensions
    costmap = Costmap(robot_radius=0.3, safety_margin=0.1)

    # Ackermann DWA planner
    dwa = AckermannDWACostmap(config={
        'max_speed': 1.0,
        'min_speed': 0.0,
        'max_accel': 0.5,
        'wheelbase': 1.5,
        'max_steering': math.radians(35),
        'max_steering_rate': math.radians(60),
        'predict_time': 3.0,
    })

    # State [x, y, yaw, v, steering]
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([8.0, 0.0])  # Waypoint 8m ahead

    # Plot setup
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))
    extent = [-grid.origin_x, grid.width - grid.origin_x,
              -grid.origin_y, grid.height - grid.origin_y]

    # Height filter for LIDAR points (ignore floor/ceiling)
    z_min = 0.1   # Ignore ground (below 10cm)
    z_max = 0.7   # Ignore ceiling (above 70cm)

    print("=== Ackermann DWA + Costmap + LIDAR ===")
    print(f"Wheelbase: {dwa.wheelbase}m")
    print(f"Robot radius: {costmap.robot_radius}m")
    print(f"Inflation: {costmap.inflation_radius}m")
    print(f"Height filter: {z_min}m to {z_max}m")
    print(f"Goal: {goal}")
    print("Close window to exit")

    try:
        while plt.fignum_exists(fig.number):
            # Update grid from LIDAR (filter by height)
            points = lidar_state.get_points()
            if points is not None and len(points) > 0:
                # Filter: keep points between z_min and z_max
                mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
                filtered = points[mask]
                if len(filtered) > 0:
                    grid.update(filtered[:, :2])

            # Update costmap
            costmap.update(grid, threshold=0.55)

            # Compute velocity
            v, steering = dwa.compute_velocity(tuple(state), tuple(goal), costmap)

            # Bicycle model update (simulation - in real use, get from vehicle)
            if abs(v) > 0.001:
                yaw_rate = v * math.tan(steering) / dwa.wheelbase
            else:
                yaw_rate = 0.0

            state[2] += yaw_rate * dwa.dt
            state[0] += v * math.cos(state[2]) * dwa.dt
            state[1] += v * math.sin(state[2]) * dwa.dt
            state[3] = v
            state[4] = steering

            # Predicted trajectory
            pred = dwa.predict_trajectory(tuple(state), v, steering)

            # Plot
            ax.clear()

            # Costmap (red=obstacle, orange=inflation)
            rgb = costmap.to_rgb()
            if rgb is not None:
                ax.imshow(rgb, origin='upper', extent=extent)

            # Predicted trajectory
            ax.plot(pred[:, 0], pred[:, 1], 'c-', lw=2, label='Predicted')

            # Robot with inflation radius
            circle = plt.Circle((state[0], state[1]), costmap.inflation_radius,
                                color='b', fill=False, lw=2, linestyle='--')
            ax.add_patch(circle)
            ax.arrow(state[0], state[1], math.cos(state[2]) * 0.8, math.sin(state[2]) * 0.8,
                     head_width=0.2, color='blue')

            # Goal
            ax.plot(goal[0], goal[1], 'm*', ms=15, label='Goal')

            ax.set_xlim(-2, 12)
            ax.set_ylim(-5, 5)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper left')
            ax.set_title(f'v={v:.2f} m/s  steering={math.degrees(steering):.1f} deg  '
                        f'obstacles={len(costmap.get_obstacle_points())}')

            plt.pause(0.05)

            # Goal check
            if dwa.goal_reached(tuple(state), tuple(goal), threshold=1.0):
                print("Goal reached!")
                break

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        lidar.stop()
        lidar.disconnect()
        plt.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
