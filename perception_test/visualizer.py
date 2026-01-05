"""
Visualization utilities for lane detection
Matches official YOLOPv2 show_seg_result output
"""

import cv2
import numpy as np


def draw_masks(frame: np.ndarray, road_mask: np.ndarray, lane_mask: np.ndarray,
               road_color=(0, 255, 0), lane_color=(0, 0, 255), alpha=0.5) -> np.ndarray:
    """
    Overlay road and lane masks on frame.

    Args:
        frame: BGR image
        road_mask: Drivable area mask (binary)
        lane_mask: Lane lines mask (binary)
        road_color: BGR color for road (default green)
        lane_color: BGR color for lanes (default red)
        alpha: Blend factor (0-1)

    Returns:
        np.ndarray: Frame with overlays
    """
    vis = frame.copy()

    # Create color overlay
    overlay = np.zeros_like(vis)
    overlay[road_mask > 0] = road_color
    overlay[lane_mask > 0] = lane_color

    # Blend where mask is non-zero
    mask = (road_mask > 0) | (lane_mask > 0)
    vis[mask] = cv2.addWeighted(vis, 1 - alpha, overlay, alpha, 0)[mask]

    return vis


def draw_masks_simple(frame: np.ndarray, road_mask: np.ndarray, lane_mask: np.ndarray) -> np.ndarray:
    """
    Simple visualization matching official YOLOPv2 demo exactly.

    Args:
        frame: BGR image (modified in-place)
        road_mask: Drivable area mask
        lane_mask: Lane lines mask

    Returns:
        np.ndarray: Modified frame
    """
    # Match official show_seg_result colors and blending
    color_area = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
    color_area[road_mask == 1] = [0, 255, 0]  # Green for road
    color_area[lane_mask == 1] = [0, 0, 255]  # Red for lanes (BGR)

    # Blend
    color_mask = np.mean(color_area, axis=2)
    frame[color_mask != 0] = frame[color_mask != 0] * 0.5 + color_area[color_mask != 0] * 0.5

    return frame
