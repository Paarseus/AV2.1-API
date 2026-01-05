#!/usr/bin/env python3
"""
Synthetic BEV + LIDAR Visualization Test
No hardware required - generates fake data for testing
"""

import numpy as np
import time
from threading import Thread, Event, Lock

from perception.visualization.open3d_viewer import MultiLayerViewer
from perception.core_types import BEVRepresentation


class SyntheticPerceptionState:
    """Fake perception state with synthetic BEV"""

    def __init__(self):
        self._lock = Lock()
        self._bev = None

    def update(self, bev):
        with self._lock:
            self._bev = bev

    def get(self):
        with self._lock:
            return self._bev


class SyntheticLidarState:
    """Fake LIDAR state with synthetic point cloud"""

    def __init__(self):
        self._lock = Lock()
        self._points = None

    def update(self, points):
        with self._lock:
            self._points = points.copy() if points is not None else None

    def get_points(self):
        with self._lock:
            return self._points.copy() if self._points is not None else None


def generate_synthetic_bev(t):
    """
    Generate synthetic BEV representation

    Args:
        t: Time parameter for animation

    Returns:
        BEVRepresentation with synthetic road texture
    """
    # BEV bounds: 0-20m ahead, -5 to +5m lateral
    bounds = (0.0, 20.0, -5.0, 5.0)
    resolution = 0.02  # 2cm per pixel

    # Calculate dimensions
    x_range = bounds[1] - bounds[0]  # 20m
    y_range = bounds[3] - bounds[2]  # 10m
    height = int(x_range / resolution)  # 1000 pixels
    width = int(y_range / resolution)   # 500 pixels

    # Create synthetic road texture
    bev_image = np.zeros((height, width, 3), dtype=np.uint8)

    # Gray road surface
    bev_image[:, :] = [80, 80, 80]

    # Lane markings (dashed white lines)
    lane_positions = [width // 4, width // 2, 3 * width // 4]  # 3 lanes
    dash_length = 40
    gap_length = 30

    for y in range(0, height, dash_length + gap_length):
        for lane_x in lane_positions:
            bev_image[y:y+dash_length, lane_x-2:lane_x+2] = [255, 255, 255]

    # Add moving "vehicle" shadow (animated with time)
    vehicle_x = int((width // 2) + 80 * np.sin(t * 0.5))
    vehicle_y = int(200 + 100 * np.sin(t * 0.3))
    bev_image[vehicle_y:vehicle_y+60, vehicle_x-20:vehicle_x+20] = [60, 60, 100]

    return BEVRepresentation(
        timestamp=time.time(),
        bev_image=bev_image,
        point_cloud=None,
        grid_resolution=resolution,
        bounds=bounds
    )


def generate_synthetic_lidar(t, num_points=10000):
    """
    Generate synthetic LIDAR point cloud

    Args:
        t: Time parameter for animation
        num_points: Number of points to generate

    Returns:
        Point cloud array (N, 3) with [x, y, z]
    """
    points = []

    # Ground plane scatter
    ground_points = num_points // 3
    x = np.random.uniform(0, 20, ground_points)
    y = np.random.uniform(-5, 5, ground_points)
    z = np.random.uniform(-0.5, 0.1, ground_points)
    points.append(np.column_stack([x, y, z]))

    # "Building" on left side (animated)
    building_points = num_points // 3
    x = np.random.uniform(5, 15, building_points)
    y = np.random.uniform(-6, -4.5, building_points) + 0.5 * np.sin(t * 0.2)
    z = np.random.uniform(0, 3, building_points)
    points.append(np.column_stack([x, y, z]))

    # "Vehicle" ahead (moving)
    vehicle_points = num_points // 3
    vehicle_x = 10 + 3 * np.sin(t * 0.5)
    vehicle_y = 1 + np.sin(t * 0.3)
    x = np.random.uniform(vehicle_x - 1, vehicle_x + 1, vehicle_points)
    y = np.random.uniform(vehicle_y - 0.8, vehicle_y + 0.8, vehicle_points)
    z = np.random.uniform(0, 1.5, vehicle_points)
    points.append(np.column_stack([x, y, z]))

    return np.vstack(points)


def synthetic_perception_thread(state, stop_event, fps=20):
    """Thread that generates synthetic BEV data"""
    print("[SYNTHETIC PERCEPTION] Starting...")

    dt = 1.0 / fps
    t = 0.0

    while not stop_event.is_set():
        loop_start = time.time()

        # Generate synthetic BEV
        bev = generate_synthetic_bev(t)
        state.update(bev)

        t += dt
        elapsed = time.time() - loop_start
        time.sleep(max(0, dt - elapsed))

    print("[SYNTHETIC PERCEPTION] Stopped")


def synthetic_lidar_thread(state, stop_event, fps=10):
    """Thread that generates synthetic LIDAR data"""
    print("[SYNTHETIC LIDAR] Starting...")

    dt = 1.0 / fps
    t = 0.0

    while not stop_event.is_set():
        loop_start = time.time()

        # Generate synthetic LIDAR
        points = generate_synthetic_lidar(t)
        state.update(points)

        t += dt
        elapsed = time.time() - loop_start
        time.sleep(max(0, dt - elapsed))

    print("[SYNTHETIC LIDAR] Stopped")


def main():
    print("\n" + "="*70)
    print("SYNTHETIC BEV + LIDAR TEST")
    print("No hardware required - generates fake data")
    print("="*70 + "\n")

    # Create states
    perception_state = SyntheticPerceptionState()
    lidar_state = SyntheticLidarState()

    # Create threads
    perception_stop = Event()
    lidar_stop = Event()

    perception_thread = Thread(
        target=synthetic_perception_thread,
        args=(perception_state, perception_stop, 20),
        daemon=False
    )

    lidar_thread = Thread(
        target=synthetic_lidar_thread,
        args=(lidar_state, lidar_stop, 10),
        daemon=False
    )

    # Start threads
    perception_thread.start()
    lidar_thread.start()
    print("[THREADS] Started")

    # Create viewer
    viewer = MultiLayerViewer(
        window_name="Synthetic BEV + LIDAR Test",
        width=1600,
        height=900,
        show_grid=True,
        show_origin=True
    )

    print("[VIEWER] Ready")
    print("\n" + "="*70)
    print("What you should see:")
    print("  - Gray road with white dashed lane markings (BEV mesh)")
    print("  - Animated blue rectangle (fake vehicle on BEV)")
    print("  - Red point cloud (synthetic LIDAR):")
    print("    • Ground scatter")
    print("    • Building on left (moving)")
    print("    • Vehicle ahead (moving)")
    print("\nControls:")
    print("  Mouse:  Rotate/Pan/Zoom")
    print("  Q/ESC:  Quit")
    print("="*70 + "\n")

    running = True
    frame_count = 0
    fps_time = time.time()

    try:
        while running:
            # Get data
            bev = perception_state.get()
            lidar_points = lidar_state.get_points()

            # Update layers
            if bev is not None:
                viewer.update_bev(bev)

            if lidar_points is not None and len(lidar_points) > 0:
                viewer.update_lidar(lidar_points, name="lidar", color=(1, 0, 0))

            # Render
            if not viewer.render():
                print("\n[VIEWER] Window closed")
                running = False
                break

            # FPS
            frame_count += 1
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                print(f"\r[VIEWER] Render FPS: {fps:.1f}", end='', flush=True)
                frame_count = 0
                fps_time = time.time()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[MAIN] Interrupted")

    finally:
        # Cleanup
        print("\n[MAIN] Shutting down...")
        perception_stop.set()
        lidar_stop.set()
        perception_thread.join(timeout=2.0)
        lidar_thread.join(timeout=2.0)
        viewer.close()
        print("✓ Done")


if __name__ == "__main__":
    main()
