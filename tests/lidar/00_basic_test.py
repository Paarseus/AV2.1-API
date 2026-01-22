#!/usr/bin/env python3
"""
Basic LIDAR test - check connection, data flow, and delay.
No filtering, just raw points.
"""

import sys
import time
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, '../..')
from sensors.lidar_interface import VelodyneLIDAR, LidarState

PORT = 2368


def main():
    print("Basic LIDAR Test")
    print("-" * 30)

    lidar = VelodyneLIDAR(port=PORT)
    if not lidar.connect():
        print("FAIL: Connection")
        return 1
    print("OK: Connected")

    lidar.start()
    state = LidarState(lidar)

    for _ in range(30):
        if state.get_points() is not None:
            print("OK: Data received")
            break
        time.sleep(0.1)
    else:
        print("FAIL: No data")
        lidar.stop()
        lidar.disconnect()
        return 1

    # Plot raw points
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    scatter = ax.scatter([], [], s=1)
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.plot(0, 0, 'r^', ms=10)

    try:
        while plt.fignum_exists(fig.number):
            t0 = time.time()
            points = state.get_points()
            dt = (time.time() - t0) * 1000

            if points is not None and points.ndim == 2:
                scatter.set_offsets(points[:, :2])
                z = points[:, 2]
                ax.set_title(f"pts={len(points)}  z=[{z.min():.1f}, {z.max():.1f}]  get={dt:.1f}ms")

            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        plt.close()
        lidar.stop()
        lidar.disconnect()
        print("Done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
