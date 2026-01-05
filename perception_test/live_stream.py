#!/usr/bin/env python3
"""
Live streaming test - Shows real-time lane/road segmentation with FPS counter
Uses sensors/camera_interface.py for camera access
"""

import sys
import os

# Add parent directory for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import time
from sensors.camera_interface import RGBDCamera
from perception_test.lane_detector import LaneDetector
from perception_test.visualizer import draw_masks_simple


def main():
    print("=" * 60)
    print("  LIVE LANE/ROAD SEGMENTATION")
    print("=" * 60)
    print()

    # Initialize detector
    detector = LaneDetector()
    if not detector.is_ready():
        print("ERROR: Model failed to load!")
        return

    # Initialize camera (uses sensors/camera_interface.py)
    print("Connecting to RealSense camera...")
    camera = RGBDCamera(width=1280, height=720, fps=30)

    if not camera.connect():
        print("ERROR: Camera connection failed!")
        print("  Check that camera is plugged in: lsusb | grep Intel")
        return

    # Start capture thread
    camera.start()
    print("Camera started")

    # Wait for first frame
    time.sleep(0.5)

    print("STREAMING LIVE (saving frames to /tmp/segmented_*.jpg)")
    print("-" * 60)

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # Get frame from threaded camera
            frame = camera.get_color_frame()
            if frame is None:
                continue  # Frame not ready yet

            # Detect road and lane masks
            road_mask, lane_mask = detector.detect(frame)

            # Visualize
            vis = draw_masks_simple(frame.copy(), road_mask, lane_mask)

            frame_count += 1

            # Display (if X11 available)
            try:
                cv2.imshow("Lane Detection - Press 'q' to quit", vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
            except cv2.error:
                pass  # Display not available

            # Save and log every 5 frames
            if frame_count % 5 == 0:
                cv2.imwrite(f'/tmp/segmented_{frame_count:04d}.jpg', vis)
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0.0
                road_px = road_mask.sum()
                lane_px = lane_mask.sum()
                print(f"Frame {frame_count:4d} | FPS: {fps:5.1f} | Road: {road_px:6d}px | Lane: {lane_px:6d}px")

    except KeyboardInterrupt:
        print("\n(Interrupted by user)")
    finally:
        elapsed = time.time() - start_time
        print("-" * 60)
        fps = frame_count / elapsed if elapsed > 0 else 0.0
        print(f"Processed {frame_count} frames in {elapsed:.1f}s ({fps:.1f} FPS)")
        print(f"Frames saved to: /tmp/segmented_*.jpg")
        camera.disconnect()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    main()
