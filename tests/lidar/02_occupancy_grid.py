#!/usr/bin/env python3
"""
02 - Occupancy Grid Visualization

Tests OccupancyGrid2D class with live LIDAR data.
Shows probability grid as heatmap.

Tune parameters in config.yaml:
  - resolution: Cell size (smaller = more detail)
  - log_odds_occ: How fast cells become occupied
  - decay_factor: How fast old data fades

Usage:
    python tests/lidar/02_occupancy_grid.py
"""

import sys
import time
import yaml
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Interactive backend
import matplotlib.pyplot as plt
from pathlib import Path

from sensors.lidar_interface import VelodyneLIDAR, LidarState
from perception import OccupancyGrid2D


def load_config():
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config()

    print("=" * 60)
    print("02 - OCCUPANCY GRID VISUALIZATION")
    print("=" * 60)
    print()

    # Connect LIDAR
    print(f"[1/3] Connecting to LIDAR (port {cfg['lidar']['port']})...")
    lidar = VelodyneLIDAR(port=cfg['lidar']['port'])

    if not lidar.connect():
        print("ERROR: LIDAR connection failed")
        return 1

    lidar.start()
    state = LidarState(lidar)
    print("Connected")

    # Create occupancy grid
    print("[2/3] Creating occupancy grid...")
    grid_cfg = cfg['occupancy_grid']
    grid = OccupancyGrid2D(
        width=grid_cfg['width'],
        height=grid_cfg['height'],
        resolution=grid_cfg['resolution'],
        log_odds_occ=grid_cfg['log_odds_occ'],
        log_odds_free=grid_cfg.get('log_odds_free', 0.1),
        decay_factor=grid_cfg['decay_factor'],
        use_raycasting=grid_cfg['use_raycasting']
    )
    print(f"Grid: {grid.cells_x}x{grid.cells_y} cells @ {grid.resolution}m")

    # Wait for data
    print("[3/3] Waiting for data...")
    for _ in range(30):
        if state.get_points() is not None:
            break
        time.sleep(0.1)
    else:
        print("ERROR: No data received")
        lidar.disconnect()
        return 1

    print()
    print("Visualizing... Close window to exit.")
    print("=" * 60)

    # Setup plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))

    # Initial grid
    prob_grid = grid.to_probability_grid()
    extent = [-grid.height/2, grid.height/2, -grid.width/2, grid.width/2]
    img = ax.imshow(prob_grid, cmap=cfg['visualization']['colormap'],
                    vmin=0, vmax=1, origin='lower', extent=extent)

    ax.set_xlabel('Y (m) - Left/Right')
    ax.set_ylabel('X (m) - Forward/Back')
    ax.set_title('Occupancy Grid (Red=Occupied, Green=Free)')
    ax.set_aspect('equal')

    # Vehicle marker
    ax.plot(0, 0, 'b^', markersize=15, label='Vehicle')
    ax.legend(loc='upper right')

    # Colorbar
    cbar = plt.colorbar(img, ax=ax)
    cbar.set_label('Occupancy Probability')

    # Stats text
    stats_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                         verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    frame_count = 0
    start_time = time.time()

    try:
        while plt.fignum_exists(fig.number):
            points = state.get_points()

            if points is not None and len(points) > 0:
                # Filter by height
                z = points[:, 2]
                mask = (z >= cfg['pointcloud']['z_min']) & (z <= cfg['pointcloud']['z_max'])
                filtered = points[mask]

                # Update grid with XY points
                n_processed = grid.update(filtered[:, :2])

                # Get probability grid
                prob_grid = grid.to_probability_grid()

                # Update image
                img.set_data(prob_grid)

                # Update stats
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0

                occupied = np.sum(prob_grid > 0.7)
                free = np.sum(prob_grid < 0.3)

                stats = (
                    f"FPS: {fps:.1f}\n"
                    f"Points: {len(filtered):,}\n"
                    f"Processed: {n_processed:,}\n"
                    f"Occupied cells: {occupied:,}\n"
                    f"Free cells: {free:,}"
                )
                stats_text.set_text(stats)

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
