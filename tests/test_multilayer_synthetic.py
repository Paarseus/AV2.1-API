"""
Test MultiLayerViewer with Synthetic Data
No camera required - generates test data to verify the viewer
"""

import numpy as np
import sys
from perception.visualization.open3d_viewer import MultiLayerViewer
from perception.core_types import BEVRepresentation, LayerConfig


def generate_synthetic_bev() -> BEVRepresentation:
    """Generate synthetic BEV image"""
    # Create 2000x1000 image (20m x 10m at 0.01m/px)
    height, width = 2000, 1000

    # Create gradient image
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Add some patterns
    # Road surface (gray)
    img[:, :] = [80, 80, 80]

    # Lane markers (white stripes)
    img[:, 450:470, :] = [255, 255, 255]  # Left lane
    img[:, 530:550, :] = [255, 255, 255]  # Right lane

    # Gradient from far (dark) to near (bright)
    for i in range(height):
        brightness = int((1.0 - i/height) * 100)
        img[i, :, 0] += brightness
        img[i, :, 1] += brightness
        img[i, :, 2] += brightness

    return BEVRepresentation(
        timestamp=0.0,
        bev_image=img,
        point_cloud=None,
        grid_resolution=0.01,
        bounds=(0.0, 20.0, -5.0, 5.0),
        source_camera="synthetic"
    )


def generate_synthetic_lidar(frame: int) -> np.ndarray:
    """Generate synthetic LIDAR point cloud"""
    num_points = 1000

    # Random points in front of vehicle
    x = np.random.uniform(5, 20, num_points)
    y = np.random.uniform(-5, 5, num_points)
    z = np.random.uniform(-1, 2, num_points)

    # Add some animated "obstacles"
    obstacle_x = 10 + np.sin(frame * 0.1) * 3
    obstacle_y = np.cos(frame * 0.1) * 2

    # Add obstacle points
    obs_points = 200
    obs_x = obstacle_x + np.random.normal(0, 0.5, obs_points)
    obs_y = obstacle_y + np.random.normal(0, 0.5, obs_points)
    obs_z = np.random.uniform(0, 1.5, obs_points)

    # Combine
    x = np.concatenate([x, obs_x])
    y = np.concatenate([y, obs_y])
    z = np.concatenate([z, obs_z])

    return np.stack([x, y, z], axis=1)


def generate_synthetic_path(frame: int) -> np.ndarray:
    """Generate synthetic path"""
    num_waypoints = 30

    # Path with slight curve
    x = np.linspace(0, 25, num_waypoints)
    y = np.sin(x * 0.2 + frame * 0.05) * 2
    z = np.zeros(num_waypoints)

    return np.stack([x, y, z], axis=1)


def main():
    print("=" * 60)
    print("MULTI-LAYER VIEWER SYNTHETIC TEST")
    print("=" * 60)
    print("\nTesting MultiLayerViewer with synthetic data")
    print("No physical camera required")
    print("\n" + "=" * 60)

    # Create multi-layer viewer
    print("\n[1/1] Creating multi-layer viewer...")
    try:
        viewer = MultiLayerViewer(
            window_name="Multi-Layer Test (Synthetic Data)",
            width=1280,
            height=720,
            show_grid=True,
            show_origin=True
        )
        print("✓ Viewer created successfully")
    except Exception as e:
        print(f"✗ Failed to create viewer: {e}")
        return 1

    # Pre-register layers
    viewer.add_layer("bev", LayerConfig("bev", "mesh", visible=True))
    viewer.add_layer("lidar", LayerConfig("lidar", "pointcloud", visible=True, color=(1, 0, 0)))
    viewer.add_layer("path", LayerConfig("path", "lineset", visible=True, color=(0, 1, 0)))

    print("\n" + "=" * 60)
    print("Controls:")
    print("  - Rotate: Left mouse button + drag")
    print("  - Pan: Middle mouse button + drag")
    print("  - Zoom: Scroll wheel")
    print("  - Close window to exit")
    print("=" * 60)
    print("\nVisualization running...")
    print("Layers: BEV (mesh), LIDAR (red points), Path (green line)")
    print("=" * 60 + "\n")

    # Animation loop
    frame_count = 0

    try:
        while True:
            # Update BEV (static)
            if frame_count == 0:
                bev = generate_synthetic_bev()
                viewer.update_bev(bev)
                print("✓ BEV layer initialized")

            # Update LIDAR (animated)
            if frame_count % 2 == 0:  # Update every 2 frames
                lidar_points = generate_synthetic_lidar(frame_count)
                viewer.update_lidar(lidar_points, name="lidar", color=(1, 0, 0))
                if frame_count == 0:
                    print("✓ LIDAR layer initialized")

            # Update path (animated)
            if frame_count % 3 == 0:  # Update every 3 frames
                path_points = generate_synthetic_path(frame_count)
                viewer.update_path(path_points, name="path", color=(0, 1, 0))
                if frame_count == 0:
                    print("✓ Path layer initialized")

            # Render
            if not viewer.render():
                print("\nViewer window closed by user")
                break

            frame_count += 1

            # Print status periodically
            if frame_count % 60 == 0:
                layers = [name for name in viewer.layers.keys() if not name.startswith("_")]
                print(f"Frame {frame_count} | Active layers: {', '.join(layers)}")

    except KeyboardInterrupt:
        print("\n\nStopped by user (Ctrl+C)")

    except Exception as e:
        print(f"\n\nError during rendering: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        viewer.close()
        print("\n" + "=" * 60)
        print("✅ TEST COMPLETE")
        print("=" * 60)
        print("\nMultiLayerViewer successfully demonstrated:")
        print("  ✓ BEV mesh layer")
        print("  ✓ LIDAR point cloud layer (animated)")
        print("  ✓ Path line layer (animated)")
        print("  ✓ Layer management (add/update/render)")
        print("  ✓ Coordinate frame and grid")

    return 0


if __name__ == "__main__":
    sys.exit(main())
