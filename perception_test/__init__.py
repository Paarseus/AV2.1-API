"""
Perception Test - YOLOPv2 Lane/Road Detection

Usage:
    from sensors.camera_interface import RGBDCamera
    from perception_test import LaneDetector
    from perception_test.visualizer import draw_masks

    camera = RGBDCamera()
    camera.connect()
    camera.start()

    frame = camera.get_color_frame()
    road_mask, lane_mask = LaneDetector().detect(frame)
    vis = draw_masks(frame, road_mask, lane_mask)
"""

from .lane_detector import LaneDetector

__all__ = ['LaneDetector']
