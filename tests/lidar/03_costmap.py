#!/usr/bin/env python3
"""
03 - Costmap Visualization

Tests Costmap class with live LIDAR data.
Shows inflated obstacles for collision checking.

Tune parameters in config.yaml:
  - robot_radius: Collision radius
  - safety_margin: Extra clearance

Usage:
    python tests/lidar/03_costmap.py
"""

import sys
import os
import time
import yaml
import numpy as np
from pathlib import Path

import matplotlib.pyplot as plt

from sensors.lidar_interface import VelodyneLIDAR, LidarState
from perception import OccupancyGrid2D, Costmap


def load_config():
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config()

    print("=" * 60)
    print("03 - COSTMAP VISUALIZATION")
    print("=" * 60)
    print()

    # Connect LIDAR
    print(f"Connecting to LIDAR (port {cfg['lidar']['port']})...")
    lidar = VelodyneLIDAR(port=cfg['lidar']['port'])

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
    grid_cfg = cfg['occupancy_grid']
    grid = OccupancyGrid2D(
        width=grid_cfg['width'],
        height=grid_cfg['height'],
        resolution=grid_cfg['resolution'],
        decay_factor=grid_cfg['decay_factor'],
        use_raycasting=grid_cfg['use_raycasting']
    )

    # Create costmap
    costmap_cfg = cfg['costmap']
    costmap = Costmap(
        robot_radius=costmap_cfg['robot_radius'],
        safety_margin=costmap_cfg['safety_margin']
    )

    # Plot setup
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 10))
    extent = [-grid.origin_x, grid.width - grid.origin_x,
              -grid.origin_y, grid.height - grid.origin_y]

    # Height filter
    z_min = cfg['pointcloud']['z_min']
    z_max = cfg['pointcloud']['z_max']

    print()
    print(f"Grid: {grid.cells_x}x{grid.cells_y} @ {grid.resolution}m")
    print(f"Robot radius: {costmap.robot_radius}m")
    print(f"Inflation radius: {costmap.inflation_radius}m")
    print(f"Height filter: {z_min}m to {z_max}m")
    print()
    print("Close window to exit")
    print("=" * 60)

    frame_count = 0
    start_time = time.time()

    try:
        while plt.fignum_exists(fig.number):
            # Update grid from LIDAR
            points = lidar_state.get_points()
            if points is not None and len(points) > 0:
                # Filter by height
                mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
                filtered = points[mask]
                if len(filtered) > 0:
                    grid.update(filtered[:, :2])

            # Update costmap
            costmap.update(grid, threshold=0.55)

            # Plot
            ax.clear()

            # Costmap RGB (red=obstacle, orange=inflation)
            rgb = costmap.to_rgb()
            if rgb is not None:
                ax.imshow(rgb, origin='upper', extent=extent)

            # Robot at origin with inflation radius
            robot_circle = plt.Circle((0, 0), costmap.robot_radius,
                                       color='cyan', fill=False, lw=2, label='Robot')
            inflation_circle = plt.Circle((0, 0), costmap.inflation_radius,
                                           color='yellow', fill=False, lw=2,
                                           linestyle='--', label='Inflation')
            ax.add_patch(robot_circle)
            ax.add_patch(inflation_circle)

            # Vehicle direction arrow
            ax.arrow(0, 0, 1.5, 0, head_width=0.3, color='blue', zorder=10)

            # Stats
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0

            collision = costmap.check_collision(0, 0)
            obstacle_points = costmap.get_obstacle_points()
            n_obstacles = len(obstacle_points) if obstacle_points is not None else 0

            ax.set_xlim(-grid.width/2, grid.width/2)
            ax.set_ylim(-grid.height/2, grid.height/2)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right')
            ax.set_xlabel('X (m) - Forward')
            ax.set_ylabel('Y (m) - Left')
            ax.set_title(
                f'FPS: {fps:.1f}  |  '
                f'Obstacles: {n_obstacles}  |  '
                f'Collision: {"YES" if collision else "NO"}'
            )

            plt.pause(1.0 / cfg['visualization']['update_hz'])

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        plt.close()
        lidar.stop()
        lidar.disconnect()
        print("Done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
