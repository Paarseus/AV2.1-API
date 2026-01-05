"""
Lane Detector - YOLOPv2 road/lane segmentation
Clean, minimal interface for use throughout FINALE codebase
"""

import os
import cv2
import torch
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

from utils.utils import (
    select_device,
    driving_area_mask,
    lane_line_mask,
    letterbox
)


class LaneDetector:
    """
    Detects drivable area and lane lines using YOLOPv2.

    Usage:
        detector = LaneDetector()
        road_mask, lane_mask = detector.detect(frame)
    """

    def __init__(self, weights="data/weights/yolopv2.pt", device='cpu'):
        """
        Initialize the lane detector.

        Args:
            weights: Path to YOLOPv2 weights
            device: 'cpu' or '0' for GPU
        """
        self.device = select_device(device)
        self.img_size = 640
        self.model = None

        # Resolve weights path
        if not os.path.isabs(weights):
            weights = os.path.join(SCRIPT_DIR, weights)

        self._load_model(weights)

    def _load_model(self, weights):
        """Load YOLOPv2 model"""
        if not os.path.exists(weights):
            print(f"ERROR: Weights not found: {weights}")
            return

        try:
            map_location = 'cpu' if self.device.type == 'cpu' else None
            self.model = torch.jit.load(weights, map_location=map_location)
            self.model = self.model.to(self.device)
            self.model.eval()

            # Warmup
            dummy = torch.zeros(1, 3, self.img_size, self.img_size).to(self.device).float()
            with torch.no_grad():
                _ = self.model(dummy)

            print(f"LaneDetector ready on {self.device}")
        except Exception as e:
            print(f"ERROR: Failed to load model: {e}")
            self.model = None

    def is_ready(self) -> bool:
        """Check if model is loaded and ready"""
        return self.model is not None

    def detect(self, frame: np.ndarray) -> tuple:
        """
        Detect road and lane masks.

        Args:
            frame: BGR image (any resolution, 1280x720 optimal)

        Returns:
            tuple: (road_mask, lane_mask) - both np.uint8 arrays same size as input
        """
        if self.model is None:
            h, w = frame.shape[:2]
            return np.zeros((h, w), np.uint8), np.zeros((h, w), np.uint8)

        h_orig, w_orig = frame.shape[:2]

        # Resize to 1280x720 (official YOLOPv2 resolution)
        if (w_orig, h_orig) == (1280, 720):
            frame_resized = frame
        else:
            frame_resized = cv2.resize(frame, (1280, 720))

        # Letterbox for model
        img, _, _ = letterbox(frame_resized, self.img_size)
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
        img = np.ascontiguousarray(img)

        # To tensor
        img_tensor = torch.from_numpy(img).to(self.device).float() / 255.0
        if img_tensor.ndimension() == 3:
            img_tensor = img_tensor.unsqueeze(0)

        # Inference
        with torch.no_grad():
            [pred, anchor_grid], seg, ll = self.model(img_tensor)

        # Extract masks (720x1280)
        road_mask = driving_area_mask(seg)
        lane_mask = lane_line_mask(ll)

        # Resize to original if needed
        if (w_orig, h_orig) != (1280, 720):
            road_mask = cv2.resize(road_mask.astype(np.uint8), (w_orig, h_orig),
                                   interpolation=cv2.INTER_NEAREST)
            lane_mask = cv2.resize(lane_mask.astype(np.uint8), (w_orig, h_orig),
                                   interpolation=cv2.INTER_NEAREST)

        return road_mask.astype(np.uint8), lane_mask.astype(np.uint8)


if __name__ == "__main__":
    # Quick test
    detector = LaneDetector()
    print(f"Ready: {detector.is_ready()}")
