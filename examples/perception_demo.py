"""
Perception Pipeline Demo
Demonstrates Camera → IPM → Visualization pipeline using new perception modules

Usage:
    # USB/CSI Camera
    python perception_demo.py --camera rgb --camera-id 0

    # RealSense RGB-D
    python perception_demo.py --camera rgbd

    # With custom parameters
    python perception_demo.py --camera rgbd --cam-height 1.5 --cam-pitch 25 --visualizer open3d
"""

import argparse
import time
import sys

# Import camera interface
from sensors.camera_interface import RGBCamera, RGBDCamera

# Import perception modules
from perception.core_types import CameraPose, BEVConfig
from perception.camera_adapter import CameraAdapter
from perception.preprocessing.ipm_processor import IPMProcessor
from perception.visualization.open3d_viewer import Open3DViewer, BEV2DViewer


def main():
    parser = argparse.ArgumentParser(
        description="Perception Pipeline Demo: Camera → IPM → Visualization",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    # Camera selection
    parser.add_argument('--camera', choices=['rgb', 'rgbd'], default='rgb',
                       help='Camera type: rgb (USB/CSI) or rgbd (RealSense)')
    parser.add_argument('--camera-id', type=int, default=0,
                       help='Camera device ID for RGB camera (default: 0)')
    parser.add_argument('--width', type=int, default=1280,
                       help='Camera width (default: 1280)')
    parser.add_argument('--height', type=int, default=720,
                       help='Camera height (default: 720)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Camera FPS (default: 30)')

    # Camera pose
    parser.add_argument('--cam-height', type=float, default=1.2,
                       help='Camera height above ground (m, default: 1.5)')
    parser.add_argument('--cam-pitch', type=float, default=0.0,
                       help='Camera pitch angle (deg, default: 20)')
    parser.add_argument('--cam-forward', type=float, default=0.0,
                       help='Camera forward offset (m, default: 0)')

    # BEV configuration
    parser.add_argument('--bev-x-max', type=float, default=20.0,
                       help='BEV far distance (m, default: 20)')
    parser.add_argument('--bev-y-range', type=float, default=5.0,
                       help='BEV lateral range (m, default: 5)')
    parser.add_argument('--bev-resolution', type=float, default=0.02,
                       help='BEV resolution (m/pixel, default: 0.02 = 20mm/px)')

    # Visualization
    parser.add_argument('--visualizer', choices=['open3d', '2d', 'both'], default='2d',
                       help='Visualization mode (default: 2d)')
    parser.add_argument('--show-grid', action='store_true',
                       help='Show metric grid overlay')

    args = parser.parse_args()

    print("="*70)
    print("PERCEPTION PIPELINE DEMO")
    print("="*70)

    # === STEP 1: Initialize Camera ===
    print(f"\n[1/4] Initializing {args.camera.upper()} camera...")

    if args.camera == 'rgbd':
        camera = RGBDCamera(width=args.width, height=args.height, fps=args.fps)
        camera_id = "rgbd_camera"
    else:
        camera = RGBCamera(
            camera_id=args.camera_id,
            width=args.width,
            height=args.height,
            fps=args.fps
        )
        camera_id = "rgb_camera"

    # Connect and start camera
    if not camera.connect():
        print("ERROR: Failed to connect to camera")
        return 1

    if not camera.start():
        print("ERROR: Failed to start camera")
        camera.disconnect()
        return 1

    print(f"✓ Camera connected: {camera.get_resolution()}")

    # Wait for camera to warm up
    print("  Warming up camera...")
    time.sleep(1.0)

    # === STEP 2: Get Intrinsics ===
    print("\n[2/4] Converting camera intrinsics...")
    intrinsics = CameraAdapter.convert_intrinsics(camera)
    print(f"✓ Intrinsics: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
    print(f"  Resolution: {intrinsics.width}x{intrinsics.height}")

    # === STEP 3: Setup IPM Processor ===
    print("\n[3/4] Initializing IPM processor...")

    camera_pose = CameraPose(
        height=args.cam_height,
        pitch=args.cam_pitch,
        forward=args.cam_forward,
        lateral=0.0,
        roll=0.0,
        yaw=0.0
    )

    bev_config = BEVConfig(
        x_min=0.0,
        x_max=args.bev_x_max,
        y_min=-args.bev_y_range,
        y_max=args.bev_y_range,
        resolution=args.bev_resolution
    )

    processor = IPMProcessor(intrinsics, camera_pose, bev_config)

    bev_size = bev_config.get_size()
    print(f"✓ IPM ready: {bev_size[0]}x{bev_size[1]} pixels")
    print(f"  Coverage: {args.bev_x_max}m forward, ±{args.bev_y_range}m lateral")
    print(f"  Resolution: {args.bev_resolution*1000:.1f} mm/pixel")

    # === STEP 4: Setup Visualizer ===
    print("\n[4/4] Initializing visualizer...")

    viewer_3d = None
    viewer_2d = None

    if args.visualizer in ['open3d', 'both']:
        viewer_3d = Open3DViewer(
            window_name="BEV 3D View",
            width=1280,
            height=720,
            show_grid=True
        )
        print("✓ Open3D viewer created")

    if args.visualizer in ['2d', 'both']:
        viewer_2d = BEV2DViewer(
            window_name="Bird's Eye View",
            scale=2
        )
        print("✓ 2D BEV viewer created")

    # === Main Loop ===
    print("\n" + "="*70)
    print("RUNNING PERCEPTION PIPELINE")
    print("="*70)
    print("\nProcessing: Camera → IPM → Visualization")
    print("Press 'q' in visualization window to quit")
    print("="*70 + "\n")

    frame_count = 0
    fps_time = time.time()
    fps = 0.0

    try:
        while True:
            loop_start = time.time()

            # Get camera frame
            camera_frame = CameraAdapter.get_frame_auto(camera, camera_id)

            if camera_frame is None:
                time.sleep(0.01)
                continue

            # Process to BEV
            bev_repr = processor.process(camera_frame)

            # Add grid if requested
            if args.show_grid:
                bev_repr.bev_image = processor.add_grid(bev_repr.bev_image, spacing=1.0)

            # Update visualizers
            if viewer_3d:
                if not viewer_3d.update(bev_repr):
                    print("\n3D viewer closed by user")
                    break

            if viewer_2d:
                if not viewer_2d.update(bev_repr):
                    print("\n2D viewer closed by user")
                    break

            # Update FPS
            frame_count += 1
            elapsed = time.time() - fps_time
            if elapsed > 1.0:
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f} | Frames: {frame_count} | "
                      f"BEV: {bev_repr.bev_image.shape} | "
                      f"Points: {len(bev_repr.point_cloud) if bev_repr.point_cloud is not None else 0}")
                frame_count = 0
                fps_time = time.time()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        # Cleanup
        print("\nCleaning up...")

        if viewer_3d:
            viewer_3d.close()

        if viewer_2d:
            viewer_2d.close()

        camera.stop()
        camera.disconnect()

        print("\n" + "="*70)
        print("DONE")
        print("="*70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
