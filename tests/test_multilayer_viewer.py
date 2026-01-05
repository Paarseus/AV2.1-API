"""
Test MultiLayerViewer
Verify multi-layer visualization with BEV, LIDAR, and path data
"""

import numpy as np
import time
from sensors.camera_interface import RGBDCamera
from perception.camera_adapter import CameraAdapter
from perception.preprocessing.ipm_processor import IPMProcessor
from perception.visualization.open3d_viewer import MultiLayerViewer
from perception.core_types import CameraPose, BEVConfig, LayerConfig


def generate_test_lidar(num_points: int = 500) -> np.ndarray:
    """Generate synthetic LIDAR point cloud"""
    # Random points in front of vehicle
    x = np.random.uniform(5, 20, num_points)
    y = np.random.uniform(-5, 5, num_points)
    z = np.random.uniform(-1, 2, num_points)
    return np.stack([x, y, z], axis=1)


def generate_test_path(num_waypoints: int = 20) -> np.ndarray:
    """Generate synthetic path"""
    # Straight path with slight curve
    x = np.linspace(0, 20, num_waypoints)
    y = np.sin(x * 0.3) * 2
    z = np.zeros(num_waypoints)
    return np.stack([x, y, z], axis=1)


def main():
    print("=" * 60)
    print("MULTI-LAYER VIEWER TEST")
    print("=" * 60)

    # Initialize camera
    print("\n[1/3] Connecting to camera...")
    camera = RGBDCamera(width=1280, height=720, fps=30)
    if not camera.connect() or not camera.start():
        print("ERROR: Camera failed")
        return 1

    # Initialize IPM
    print("[2/3] Setting up IPM processor...")
    intrinsics = CameraAdapter.convert_intrinsics(camera)
    pose = CameraPose(height=1.2, forward=0.0, lateral=0.0, pitch=5.0, roll=0.0, yaw=0.0)
    bev_config = BEVConfig(x_min=0.0, x_max=20.0, y_min=-5.0, y_max=5.0, resolution=0.01)
    ipm = IPMProcessor(intrinsics, pose, bev_config)

    # Initialize multi-layer viewer
    print("[3/3] Creating multi-layer viewer...")
    viewer = MultiLayerViewer(
        window_name="Multi-Layer Test | BEV + LIDAR + Path",
        width=1280,
        height=720,
        show_grid=True,
        show_origin=True
    )

    # Register layers
    viewer.add_layer("bev", LayerConfig("bev", "mesh", visible=True))
    viewer.add_layer("lidar", LayerConfig("lidar", "pointcloud", visible=True, color=(1, 0, 0)))
    viewer.add_layer("path", LayerConfig("path", "lineset", visible=True, color=(0, 1, 0)))

    print("\n" + "=" * 60)
    print("Controls:")
    print("  - Rotate: Left mouse button + drag")
    print("  - Pan: Middle mouse button + drag")
    print("  - Zoom: Scroll wheel")
    print("  - Close window to exit")
    print("=" * 60 + "\n")

    # Main loop
    frame_count = 0
    lidar_update_interval = 10  # Update LIDAR every 10 frames
    path_update_interval = 30   # Update path every 30 frames

    try:
        while True:
            # Get camera frame
            frame = CameraAdapter.get_frame_auto(camera, "rgbd_camera")
            if frame is None:
                time.sleep(0.01)
                continue

            # Generate BEV
            bev_repr = ipm.process(frame)
            viewer.update_bev(bev_repr)

            # Update LIDAR (less frequently for demo)
            if frame_count % lidar_update_interval == 0:
                lidar_points = generate_test_lidar(num_points=500)
                viewer.update_lidar(lidar_points, name="lidar", color=(1, 0, 0))

            # Update path (even less frequently)
            if frame_count % path_update_interval == 0:
                path_points = generate_test_path(num_waypoints=20)
                viewer.update_path(path_points, name="path", color=(0, 1, 0))

            # Render
            if not viewer.render():
                break

            frame_count += 1

            # Print status
            if frame_count % 30 == 0:
                layers = [name for name in viewer.layers.keys() if not name.startswith("_")]
                print(f"Frame {frame_count} | Layers: {', '.join(layers)}")

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    finally:
        viewer.close()
        camera.stop()
        camera.disconnect()
        print("Done")

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
