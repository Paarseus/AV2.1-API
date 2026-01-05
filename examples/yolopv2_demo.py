"""
YOLOPv2 Demo
Real-time detection, drivable area, and lane line segmentation

Usage:
    # Default (auto-detects weights, uses RealSense D455)
    python yolopv2_demo.py --camera rgbd

    # With USB camera
    python yolopv2_demo.py --camera rgb --camera-id 0

    # Smaller display (fit small screens)
    python yolopv2_demo.py --camera rgbd --display-width 800 --display-height 600

    # Larger display (fit 1080p screens)
    python yolopv2_demo.py --camera rgbd --display-width 1920 --display-height 1080

    # Custom weights file
    python yolopv2_demo.py --camera rgbd --weights /path/to/custom.pt

    # Run on CPU instead of GPU
    python yolopv2_demo.py --camera rgbd --device cpu
"""

import argparse
import cv2
import numpy as np
import sys
import time

from sensors.camera_interface import RGBCamera, RGBDCamera
from perception.preprocessing.yolopv2_adapter import YOLOPv2Adapter


def visualize(image: np.ndarray, output, max_width: int = 1280, max_height: int = 720) -> np.ndarray:
    """
    Create 2x2 grid visualization:
    [Original | Drivable Area]
    [Lane Lines | Detections]

    Args:
        image: RGB input image
        output: YOLOPv2Output
        max_width: Maximum display width in pixels
        max_height: Maximum display height in pixels
    """
    h, w = image.shape[:2]

    # Convert RGB to BGR for OpenCV display
    image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # 1. Original
    img1 = image_bgr.copy()

    # 2. Drivable area (green overlay)
    img2 = image_bgr.copy()
    green_overlay = np.zeros_like(image_bgr)
    green_overlay[:, :, 1] = output.drivable_area
    img2 = cv2.addWeighted(img2, 0.7, green_overlay, 0.3, 0)

    # 3. Lane lines (red overlay)
    img3 = image_bgr.copy()
    red_overlay = np.zeros_like(image_bgr)
    red_overlay[:, :, 2] = output.lane_lines
    img3 = cv2.addWeighted(img3, 0.7, red_overlay, 0.3, 0)

    # 4. Detections (yellow boxes)
    img4 = image_bgr.copy()
    for det in output.detections:
        x1, y1, x2, y2, conf, cls = det
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        cv2.rectangle(img4, (x1, y1), (x2, y2), (0, 255, 255), 3)

        label = f"{conf:.2f}"
        cv2.putText(img4, label, (x1, y1 - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Combine into 2x2 grid
    top = np.hstack([img1, img2])
    bottom = np.hstack([img3, img4])
    grid = np.vstack([top, bottom])

    # Add labels
    cv2.putText(grid, "Original", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(grid, "Drivable Area", (w + 10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(grid, "Lane Lines", (10, h + 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(grid, f"Detections ({len(output.detections)})", (w + 10, h + 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Resize to fit screen (always resize to specified display size)
    grid_h, grid_w = grid.shape[:2]

    scale = min(max_width / grid_w, max_height / grid_h)
    new_w = int(grid_w * scale)
    new_h = int(grid_h * scale)

    if scale != 1.0:
        grid = cv2.resize(grid, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return grid


def main():
    parser = argparse.ArgumentParser(description="YOLOPv2 Demo")

    # Model
    parser.add_argument('--weights', type=str, default=None,
                       help='Path to YOLOPv2 .pt file (default: auto-detect in perception/preprocessing/yolopv2.pt)')
    parser.add_argument('--device', type=str, default='cuda',
                       help='cuda or cpu')
    parser.add_argument('--conf-thres', type=float, default=0.3,
                       help='Detection confidence threshold')

    # Camera
    parser.add_argument('--camera', choices=['rgb', 'rgbd'], default='rgb',
                       help='Camera type')
    parser.add_argument('--camera-id', type=int, default=0,
                       help='Camera ID for RGB camera')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=30)

    # Display size (for the 2x2 grid)
    parser.add_argument('--display-width', type=int, default=1280,
                       help='Maximum display width in pixels (default: 1280)')
    parser.add_argument('--display-height', type=int, default=720,
                       help='Maximum display height in pixels (default: 720)')

    args = parser.parse_args()

    print("=" * 60)
    print("YOLOPv2 DEMO")
    print("=" * 60)

    # Initialize YOLOPv2
    print(f"\n[1/2] Loading YOLOPv2...")
    if args.weights:
        print(f"  Using weights: {args.weights}")
    else:
        print(f"  Using default weights: perception/preprocessing/yolopv2.pt")

    yolo = YOLOPv2Adapter(
        weights=args.weights,
        device=args.device,
        conf_thres=args.conf_thres
    )

    # Initialize camera
    print(f"\n[2/2] Connecting to camera...")
    if args.camera == 'rgbd':
        camera = RGBDCamera(width=args.width, height=args.height, fps=args.fps)
    else:
        camera = RGBCamera(
            camera_id=args.camera_id,
            width=args.width,
            height=args.height,
            fps=args.fps
        )

    if not camera.connect() or not camera.start():
        print("ERROR: Camera failed")
        return 1

    print("✓ Camera ready")
    time.sleep(0.5)

    print("\n" + "=" * 60)
    print(f"Display size: {args.display_width}x{args.display_height}")
    print("Press 'q' to quit")
    print("=" * 60 + "\n")

    # Main loop
    frame_count = 0
    fps_time = time.time()
    fps = 0.0

    try:
        while True:
            # Get frame
            frame = camera.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            # Inference
            t0 = time.time()
            output = yolo.infer(frame)
            inf_time = (time.time() - t0) * 1000

            # Visualize
            vis = visualize(frame, output, args.display_width, args.display_height)
            cv2.putText(vis, f"FPS: {fps:.1f} | Inference: {inf_time:.0f}ms",
                       (10, vis.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("YOLOPv2", vis)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Update FPS
            frame_count += 1
            if time.time() - fps_time > 1.0:
                fps = frame_count / (time.time() - fps_time)
                print(f"FPS: {fps:.1f} | Inference: {inf_time:.0f}ms | Detections: {len(output.detections)}")
                frame_count = 0
                fps_time = time.time()

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    finally:
        cv2.destroyAllWindows()
        camera.stop()
        camera.disconnect()
        print("Done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
