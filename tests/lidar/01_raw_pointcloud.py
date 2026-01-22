#!/usr/bin/env python3
"""
01 - Raw Point Cloud Visualization

Verifies LIDAR is working and shows raw XY point cloud.
Use this to confirm data is flowing before testing perception.

Usage:
    python tests/lidar/01_raw_pointcloud.py
"""

import sys
import os
import time
import yaml
import numpy as np
import matplotlib
# Try interactive backends in order of preference
for backend in ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'WebAgg']:
    try:
        matplotlib.use(backend)
        break
    except ImportError:
        continue
import matplotlib.pyplot as plt
from pathlib import Path

from sensors.lidar_interface import VelodyneLIDAR, LidarState


def load_config():
    config_path = Path(__file__).parent / "config.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config()

    print("=" * 60)
    print("01 - RAW POINT CLOUD VISUALIZATION")
    print("=" * 60)
    print()

    # Connect LIDAR
    print(f"[1/2] Connecting to LIDAR (port {cfg['lidar']['port']})...")
    lidar = VelodyneLIDAR(port=cfg['lidar']['port'])

    if not lidar.connect():
        print("ERROR: LIDAR connection failed")
        return 1

    lidar.start()
    state = LidarState(lidar)
    print("Connected")

    # Wait for data
    print("[2/2] Waiting for data...")
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
    scatter = ax.scatter([], [], s=1, c='blue')

    ax.set_xlim(-cfg['pointcloud']['range_max'], cfg['pointcloud']['range_max'])
    ax.set_ylim(-cfg['pointcloud']['range_max'], cfg['pointcloud']['range_max'])
    ax.set_xlabel('X (m) - Forward')
    ax.set_ylabel('Y (m) - Left')
    ax.set_title('Raw LIDAR Point Cloud (Top-Down)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Draw vehicle marker
    ax.plot(0, 0, 'r^', markersize=15, label='Vehicle')
    ax.legend()

    # Stats text
    stats_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                         verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    try:
        while plt.fignum_exists(fig.number):
            points = state.get_points()

            if points is not None and points.ndim == 2 and len(points) > 0:
                # Filter by height and range
                z = points[:, 2]
                r = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
                mask = ((z >= cfg['pointcloud']['z_min']) &
                        (z <= cfg['pointcloud']['z_max']) &
                        (r <= cfg['pointcloud']['range_max']))
                filtered = points[mask]

                if len(filtered) > 0:
                    scatter.set_offsets(filtered[:, :2])

                stats = f"Total: {len(points):,}  Filtered: {len(filtered):,}  Z: [{z.min():.2f}, {z.max():.2f}]"
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
