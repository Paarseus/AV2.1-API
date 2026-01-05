"""
IPM Live - Single Script for RealSense Camera Bird's Eye View
All-in-one: Camera + IPM + Visualization + Live Parameter Adjustment

Usage:
    python ipm_live.py

Controls:
    w/s : Increase/Decrease PITCH (camera tilt)
    a/d : Increase/Decrease HEIGHT
    i/k : Increase/Decrease RESOLUTION
    g   : Toggle grid
    q   : Quit
"""

import numpy as np
import cv2
import pyrealsense2 as rs
import time


class IPMLive:
    """All-in-one IPM system with live parameter adjustment"""

    def __init__(self):
        # Camera parameters (adjustable live)
        self.cam_height = 1.5      # Height above ground (m)
        self.cam_pitch = 0.0      # Pitch angle (degrees, positive = down)
        self.cam_forward = 0.0     # Forward offset (m)
        self.cam_lateral = 0.0     # Lateral offset (m)

        # BEV parameters (adjustable live)
        self.bev_x_min = 0.0
        self.bev_x_max = 20.0
        self.bev_y_min = -5.0
        self.bev_y_max = 5.0
        self.bev_resolution = 0.01  # m/pixel

        # Visualization
        self.show_grid = True
        self.show_fps = True

        # Camera
        self.pipeline = None
        self.align = None
        self.intrinsics = None
        self.K = None

        # Precomputed mapping (will be rebuilt when parameters change)
        self.map_x = None
        self.map_y = None
        self.need_rebuild = True

        # FPS tracking
        self.frame_count = 0
        self.fps_time = time.time()
        self.fps = 0.0

    def start_camera(self):
        """Initialize RealSense camera"""
        print("Starting RealSense D455...")

        self.pipeline = rs.pipeline()
        config = rs.config()

        # Configure streams
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # Get intrinsics
        color_stream = profile.get_stream(rs.stream.color)
        color_profile = color_stream.as_video_stream_profile()
        intr = color_profile.get_intrinsics()

        self.intrinsics = {
            'fx': intr.fx,
            'fy': intr.fy,
            'cx': intr.ppx,
            'cy': intr.ppy,
            'width': intr.width,
            'height': intr.height
        }

        self.K = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        print(f"✓ Camera ready: {intr.width}×{intr.height}")
        print(f"  fx={intr.fx:.1f}, fy={intr.fy:.1f}")

    def get_frame(self):
        """Capture frame from camera"""
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        color_frame = aligned.get_color_frame()
        if not color_frame:
            return None

        color = np.asanyarray(color_frame.get_data())
        return color

    def build_transform(self):
        """Build transformation matrix from current parameters"""
        # Convert to radians
        pitch_rad = np.radians(self.cam_pitch)

        # Base rotation: vehicle → camera frame
        R_base = np.array([
            [0, -1,  0],
            [0,  0, -1],
            [1,  0,  0]
        ], dtype=np.float32)

        # Pitch rotation
        R_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
            [0, np.sin(pitch_rad), np.cos(pitch_rad)]
        ], dtype=np.float32)

        # Combined rotation
        R = R_pitch @ R_base

        # Translation
        t_vehicle = np.array([self.cam_forward, self.cam_lateral, self.cam_height])
        t_camera = -R @ t_vehicle

        # Build 4×4 matrix
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3] = t_camera

        return T

    def rebuild_mapping(self):
        """Rebuild pixel mapping for current parameters"""
        print(f"Rebuilding IPM map (res={self.bev_resolution*1000:.1f}mm/px)...")

        # Calculate output size
        width = int((self.bev_y_max - self.bev_y_min) / self.bev_resolution)
        height = int((self.bev_x_max - self.bev_x_min) / self.bev_resolution)

        self.map_x = np.zeros((height, width), dtype=np.float32)
        self.map_y = np.zeros((height, width), dtype=np.float32)

        # Get transformation
        T = self.build_transform()

        # For each BEV pixel
        for i in range(height):
            for j in range(width):
                # BEV pixel to world coordinates
                # Convention: bottom = near, top = far
                X = self.bev_x_max - i * self.bev_resolution
                Y = self.bev_y_min + j * self.bev_resolution

                # Project to camera
                point_vehicle = np.array([X, Y, 0.0, 1.0])
                point_camera = T @ point_vehicle
                Xc, Yc, Zc = point_camera[:3]

                if Zc > 0:  # In front of camera
                    u = self.K[0, 0] * (Xc / Zc) + self.K[0, 2]
                    v = self.K[1, 1] * (Yc / Zc) + self.K[1, 2]
                    self.map_x[i, j] = u
                    self.map_y[i, j] = v
                else:
                    self.map_x[i, j] = -1
                    self.map_y[i, j] = -1

        self.need_rebuild = False
        print(f"✓ Map ready: {width}×{height} BEV")

    def process_frame(self, image):
        """Transform image to BEV"""
        if self.need_rebuild:
            self.rebuild_mapping()

        bev = cv2.remap(
            image,
            self.map_x,
            self.map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )

        return bev

    def add_grid(self, image):
        """Add metric grid to BEV"""
        output = image.copy()
        height, width = image.shape[:2]

        # Grid spacing in pixels
        spacing_px_x = int(1.0 / self.bev_resolution)  # 1 meter
        spacing_px_y = int(1.0 / self.bev_resolution)

        # Draw grid
        for i in range(0, height, spacing_px_x):
            cv2.line(output, (0, i), (width, i), (0, 255, 0), 1)

        for j in range(0, width, spacing_px_y):
            cv2.line(output, (j, 0), (j, height), (0, 255, 0), 1)

        return output

    def create_display(self, camera_img, bev_img):
        """Create side-by-side display"""
        # Add grid to BEV if enabled
        if self.show_grid:
            bev_display = self.add_grid(bev_img)
        else:
            bev_display = bev_img.copy()

        # Upscale BEV for visibility (2x)
        bev_large = cv2.resize(bev_display,
                               (bev_display.shape[1] * 2, bev_display.shape[0] * 2),
                               interpolation=cv2.INTER_CUBIC)

        # Resize camera to match BEV height
        scale = bev_large.shape[0] / camera_img.shape[0]
        cam_resized = cv2.resize(camera_img,
                                 (int(camera_img.shape[1] * scale), bev_large.shape[0]))

        # Add labels
        self._add_labels(cam_resized, bev_large)

        # Combine
        display = np.hstack([cam_resized, bev_large])
        return display

    def _add_labels(self, cam_img, bev_img):
        """Add text overlays"""
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Camera view label
        cv2.putText(cam_img, "Camera", (10, 30), font, 1.0, (255, 255, 255), 2)

        # BEV label and parameters
        cv2.putText(bev_img, "Bird's Eye View", (10, 30), font, 1.0, (255, 255, 255), 2)

        y = 70
        params = [
            f"Height: {self.cam_height:.2f}m [a/d]",
            f"Pitch:  {self.cam_pitch:.1f}deg [w/s]",
            f"Resolution: {self.bev_resolution*1000:.1f}mm/px [i/k]",
            f"Grid: {'ON' if self.show_grid else 'OFF'} [g]",
        ]

        for text in params:
            cv2.putText(bev_img, text, (10, y), font, 0.6, (0, 255, 255), 2)
            y += 30

        # FPS
        if self.show_fps:
            cv2.putText(bev_img, f"FPS: {self.fps:.1f}", (10, bev_img.shape[0] - 20),
                       font, 0.7, (0, 255, 0), 2)

        # Instructions at bottom
        cv2.putText(bev_img, "Press 'q' to quit", (10, bev_img.shape[0] - 50),
                   font, 0.5, (200, 200, 200), 1)

    def update_fps(self):
        """Update FPS counter"""
        self.frame_count += 1
        elapsed = time.time() - self.fps_time

        if elapsed > 0.5:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_time = time.time()

    def run(self):
        """Main loop"""
        print("\n" + "="*70)
        print("IPM LIVE - Real-Time Bird's Eye View")
        print("="*70)

        self.start_camera()
        self.rebuild_mapping()

        print("\n🎮 Controls:")
        print("  w/s : Adjust PITCH (camera tilt angle)")
        print("  a/d : Adjust HEIGHT (camera height above ground)")
        print("  i/k : Adjust RESOLUTION (finer/coarser)")
        print("  g   : Toggle grid overlay")
        print("  q   : Quit")
        print("\n" + "="*70)
        print("Adjust HEIGHT and PITCH until road appears correctly in BEV")
        print("="*70 + "\n")

        cv2.namedWindow("IPM Live", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("IPM Live", 1920, 1080)

        try:
            while True:
                # Capture
                color = self.get_frame()
                if color is None:
                    continue

                # Process
                bev = self.process_frame(color)

                # Display
                display = self.create_display(color, bev)
                cv2.imshow("IPM Live", display)

                # Update FPS
                self.update_fps()

                # Handle keys
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break

                elif key == ord('w'):  # Increase pitch
                    self.cam_pitch += 1.0
                    self.need_rebuild = True
                    print(f"Pitch: {self.cam_pitch:.1f}°")

                elif key == ord('s'):  # Decrease pitch
                    self.cam_pitch -= 1.0
                    self.need_rebuild = True
                    print(f"Pitch: {self.cam_pitch:.1f}°")

                elif key == ord('a'):  # Increase height
                    self.cam_height += 0.05
                    self.need_rebuild = True
                    print(f"Height: {self.cam_height:.2f}m")

                elif key == ord('d'):  # Decrease height
                    self.cam_height = max(0.1, self.cam_height - 0.05)
                    self.need_rebuild = True
                    print(f"Height: {self.cam_height:.2f}m")

                elif key == ord('i'):  # Finer resolution
                    self.bev_resolution = max(0.002, self.bev_resolution - 0.005)
                    self.need_rebuild = True
                    print(f"Resolution: {self.bev_resolution*1000:.1f}mm/px")

                elif key == ord('k'):  # Coarser resolution
                    self.bev_resolution += 0.005
                    self.need_rebuild = True
                    print(f"Resolution: {self.bev_resolution*1000:.1f}mm/px")

                elif key == ord('g'):  # Toggle grid
                    self.show_grid = not self.show_grid
                    print(f"Grid: {'ON' if self.show_grid else 'OFF'}")

        except KeyboardInterrupt:
            print("\nInterrupted")

        finally:
            cv2.destroyAllWindows()
            self.pipeline.stop()

            print("\n" + "="*70)
            print("FINAL PARAMETERS:")
            print("="*70)
            print(f"  Camera Height: {self.cam_height:.3f} m")
            print(f"  Camera Pitch:  {self.cam_pitch:.1f}°")
            print(f"  BEV Resolution: {self.bev_resolution*1000:.1f} mm/pixel")
            print("\nUse these values in your config file!")
            print("="*70)


if __name__ == "__main__":
    app = IPMLive()
    app.run()
