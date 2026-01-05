#!/usr/bin/env python3
"""
LIDAR Occupancy Grid - Matplotlib Visualization

Real-time occupancy grid display with Velodyne LIDAR.
"""

import sys
import time

sys.path.insert(0, '..')
from sensors.lidar_interface import VelodyneLIDAR, LidarState
from perception import OccupancyGrid2D
from perception.visualization import OccupancyGridVisualizer


def main():
    """Main entry point"""
    print("=" * 70)
    print("LIDAR OCCUPANCY GRID VISUALIZATION")
    print("=" * 70)
    print()

    # Connect LIDAR
    print("[1/3] Connecting to LIDAR (port 2368)...")
    lidar = VelodyneLIDAR(port=2368)

    if not lidar.connect():
        print("ERROR: LIDAR connection failed")
        print()
        print("Troubleshooting:")
        print("  1. Check LIDAR power")
        print("  2. Verify network: ping 192.168.1.201")
        print("  3. Check firewall: sudo ufw allow 2368/udp")
        return 1

    if not lidar.start():
        print("ERROR: LIDAR start failed")
        lidar.disconnect()
        return 1

    lidar_state = LidarState(lidar)
    print("Connected")
    print()

    # Create grid
    print("[2/3] Initializing occupancy grid...")
    grid = OccupancyGrid2D(
        width=20.0,
        height=20.0,
        resolution=0.2,
        log_odds_occ=0.4,
        decay_factor=0.5,
        use_raycasting=False
    )
    print(f"Grid: {grid.cells_x}x{grid.cells_y} @ {grid.resolution}m")
    print()

    # Wait for data
    print("[3/3] Waiting for LIDAR data...")
    max_wait = 5.0
    start = time.time()

    while time.time() - start < max_wait:
        if lidar_state.get_points() is not None:
            print("Data received")
            break
        time.sleep(0.2)
    else:
        print("ERROR: No data after 5 seconds")
        print("Check: sudo tcpdump -i <interface> port 2368")
        lidar.stop()
        lidar.disconnect()
        return 1

    print()
    print("=" * 70)
    print("Starting visualization (close window to exit)")
    print("=" * 70)
    print()

    # Create visualizer
    viz = OccupancyGridVisualizer(grid, update_hz=10, title='LIDAR Occupancy Grid')

    def update_callback():
        """Called each frame - update grid and return stats"""
        points = lidar_state.get_points()
        if points is not None:
            n_proc = grid.update(points[:, :2])
            return f"Points: {len(points):5d}\nProc:   {n_proc:5d}"
        return None

    try:
        viz.run(update_callback=update_callback)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        lidar.stop()
        lidar.disconnect()
        print("Disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
