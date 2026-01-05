#!/usr/bin/env python3
"""
BEV + LIDAR Simultaneous Visualization
Option 2: MultiLayerViewer with Separate Sensor Threads

Visualizes:
- BEV mesh from camera + IPM (textured ground plane)
- LIDAR point cloud from Velodyne (3D structure)

Both in the same 3D window with unified coordinate frame.
"""

import numpy as np
import time
import argparse
import sys
from threading import Thread, Event

sys.path.insert(0, '..')

# Sensors
from sensors.camera_interface import RGBDCamera
from sensors.lidar_interface import VelodyneLIDAR, LidarState

# Perception
from perception import PerceptionState
from perception.camera_adapter import CameraAdapter
from perception.preprocessing.ipm_processor import IPMProcessor
from perception.visualization.open3d_viewer import MultiLayerViewer
from perception.core_types import CameraPose, BEVConfig


def perception_thread_func(perception_state, camera, ipm, stop_event, target_fps=20.0):
    """
    Perception thread: Camera → IPM → BEV

    Args:
        perception_state: PerceptionState instance
        camera: RGBDCamera instance
        ipm: IPMProcessor instance
        stop_event: Event to signal thread stop
        target_fps: Target frame rate
    """
    print(f"[PERCEPTION] INFO: Thread started ({target_fps} FPS)")

    dt = 1.0 / target_fps
    frame_count = 0
    fps_time = time.time()

    try:
        while not stop_event.is_set():
            loop_start = time.time()

            # Get camera frame
            frame = CameraAdapter.get_frame_auto(camera, "camera")
            if frame is None:
                time.sleep(0.001)
                continue

            # Generate BEV
            bev_repr = ipm.process(frame)

            # Update state
            perception_state.update(bev_repr=bev_repr)

            # FPS tracking
            frame_count += 1
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                print(f"[PERCEPTION] INFO: {fps:.1f} FPS")
                frame_count = 0
                fps_time = time.time()

            # Rate control
            elapsed = time.time() - loop_start
            time.sleep(max(0, dt - elapsed))

    except Exception as e:
        print(f"[PERCEPTION] ERROR: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("[PERCEPTION] INFO: Thread stopped")


def main():
    parser = argparse.ArgumentParser(
        description='BEV + LIDAR Simultaneous Visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Both sensors
  %(prog)s

  # LIDAR only (no camera)
  %(prog)s --lidar-only

  # BEV only (no LIDAR)
  %(prog)s --bev-only

  # With PCAP file instead of live LIDAR
  %(prog)s --pcap mydata.pcap
        """
    )

    # Sensor options
    parser.add_argument('--lidar-only', action='store_true', help='LIDAR visualization only')
    parser.add_argument('--bev-only', action='store_true', help='BEV visualization only')
    parser.add_argument('--lidar-port', type=int, default=2368, help='LIDAR UDP port (default: 2368)')
    parser.add_argument('--pcap', type=str, help='PCAP file to replay instead of live LIDAR')

    # Visualization options
    parser.add_argument('--window-width', type=int, default=1600, help='Window width (default: 1600)')
    parser.add_argument('--window-height', type=int, default=900, help='Window height (default: 900)')
    parser.add_argument('--lidar-color', type=str, default='red',
                        help='LIDAR color: red, green, blue, white, height (default: red)')

    args = parser.parse_args()

    # Validate args
    if args.lidar_only and args.bev_only:
        print("ERROR: Cannot use --lidar-only and --bev-only together")
        return

    enable_camera = not args.lidar_only
    enable_lidar = not args.bev_only

    print("\n" + "="*70)
    print("BEV + LIDAR VISUALIZATION")
    print("="*70)
    print(f"Camera (BEV): {'ENABLED' if enable_camera else 'DISABLED'}")
    print(f"LIDAR:        {'ENABLED' if enable_lidar else 'DISABLED'}")
    print("="*70 + "\n")

    # Initialize sensors
    camera = None
    ipm = None
    lidar = None
    perception_state = None
    lidar_state = None

    # =========================================================================
    # Setup Camera + IPM
    # =========================================================================
    if enable_camera:
        print("[CAMERA] INFO: Connecting camera...")
        camera = RGBDCamera(width=1280, height=720, fps=30)

        if not camera.connect():
            print("[CAMERA] ERROR: Failed to connect")
            return

        if not camera.start():
            print("[CAMERA] ERROR: Failed to start")
            camera.disconnect()
            return

        print("[CAMERA] INFO: Connected (1280x720 @ 30fps)")

        # Setup IPM
        print("[IPM] INFO: Configuring IPM...")
        intrinsics = CameraAdapter.convert_intrinsics(camera)
        pose = CameraPose(height=1.2, pitch=5.0, roll=0.0, yaw=0.0)
        bev_config = BEVConfig(
            x_min=0.0,
            x_max=20.0,
            y_min=-5.0,
            y_max=5.0,
            resolution=0.01
        )
        ipm = IPMProcessor(intrinsics, pose, bev_config)
        print(f"[IPM] INFO: Configured (BEV: {bev_config.x_max}m x {bev_config.y_max*2}m)")

        perception_state = PerceptionState()

    # =========================================================================
    # Setup LIDAR
    # =========================================================================
    if enable_lidar:
        print(f"[LIDAR] INFO: Connecting LIDAR (port {args.lidar_port})...")

        try:
            import velodyne_decoder as vd
        except ImportError:
            print("[LIDAR] ERROR: velodyne_decoder not installed")
            print("Install with: pip install velodyne-decoder")
            if camera:
                camera.stop()
                camera.disconnect()
            return

        lidar = VelodyneLIDAR(port=args.lidar_port)

        if not lidar.connect():
            print("[LIDAR] ERROR: Failed to connect")
            if camera:
                camera.stop()
                camera.disconnect()
            return

        if not lidar.start():
            print("[LIDAR] ERROR: Failed to start")
            lidar.disconnect()
            if camera:
                camera.stop()
                camera.disconnect()
            return

        print("[LIDAR] INFO: Connected")
        print(f"[LIDAR] INFO: {lidar.get_intrinsics()}")

        lidar_state = LidarState(lidar)

    # =========================================================================
    # Start Perception Thread (if camera enabled)
    # =========================================================================
    perception_thread = None
    perception_stop_event = Event()

    if enable_camera:
        perception_thread = Thread(
            target=perception_thread_func,
            args=(perception_state, camera, ipm, perception_stop_event, 20.0),
            daemon=False
        )
        perception_thread.start()
        print("[PERCEPTION] INFO: Thread started")

    # =========================================================================
    # Create Unified Viewer
    # =========================================================================
    print("\n[VIEWER] INFO: Creating 3D viewer...")

    viewer = MultiLayerViewer(
        window_name="BEV + LIDAR Visualization",
        width=args.window_width,
        height=args.window_height,
        show_grid=True,
        show_origin=True
    )

    print("[VIEWER] INFO: Viewer ready")

    # Parse LIDAR color
    lidar_color_map = {
        'red': (1, 0, 0),
        'green': (0, 1, 0),
        'blue': (0, 0, 1),
        'white': (1, 1, 1),
        'height': None  # Use height-based coloring
    }
    lidar_color = lidar_color_map.get(args.lidar_color, (1, 0, 0))

    # =========================================================================
    # Main Visualization Loop
    # =========================================================================
    print("\n" + "="*70)
    print("Controls:")
    print("  Mouse:    Rotate/Pan/Zoom")
    print("  Q/ESC:    Quit")
    print("  H:        Help")
    print("="*70 + "\n")

    print("[MAIN] INFO: Starting visualization loop...")

    viz_interval = 0.05  # 20 Hz
    running = True

    bev_update_count = 0
    lidar_update_count = 0

    try:
        while running:
            loop_start = time.time()

            # =================================================================
            # Update BEV layer
            # =================================================================
            if enable_camera and perception_state:
                bev = perception_state.get()
                if bev is not None:
                    viewer.update_bev(bev)
                    bev_update_count += 1

            # =================================================================
            # Update LIDAR layer
            # =================================================================
            if enable_lidar and lidar_state:
                lidar_points = lidar_state.get_points()
                if lidar_points is not None and len(lidar_points) > 0:
                    viewer.update_lidar(
                        lidar_points,
                        name="lidar",
                        color=lidar_color
                    )
                    lidar_update_count += 1

            # =================================================================
            # Render frame
            # =================================================================
            if not viewer.render():
                print("\n[VIEWER] INFO: Window closed")
                running = False
                break

            # Rate control
            elapsed = time.time() - loop_start
            time.sleep(max(0, viz_interval - elapsed))

    except KeyboardInterrupt:
        print("\n[MAIN] INFO: Interrupted by user")

    finally:
        running = False

        print("\n[MAIN] INFO: Shutting down...")

        # Stop perception thread
        if perception_thread:
            perception_stop_event.set()
            perception_thread.join(timeout=2.0)
            print("[PERCEPTION] INFO: Thread stopped")

        # Stop camera
        if camera:
            camera.stop()
            camera.disconnect()
            print("[CAMERA] INFO: Stopped")

        # Stop LIDAR
        if lidar:
            lidar.stop()
            lidar.disconnect()
            print("[LIDAR] INFO: Stopped")

        # Close viewer
        viewer.close()
        print("[VIEWER] INFO: Closed")

        print("\n" + "="*70)
        print("Statistics:")
        print(f"  BEV updates:   {bev_update_count}")
        print(f"  LIDAR updates: {lidar_update_count}")
        print("="*70)
        print("\n✓ Done")


if __name__ == "__main__":
    main()
