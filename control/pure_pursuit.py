import numpy as np
from typing import Optional
import math


class PurePursuitController:
    def __init__(
        self, 
        K_dd: float = 0.4,
        wheel_base: float = 1.23,
        waypoint_shift: float = 0,
        min_lookahead: float = 3.0,
        max_lookahead: float = 20.0
    ):

        self.K_dd = K_dd
        self.wheel_base = wheel_base
        self.waypoint_shift = waypoint_shift
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        
    def get_steering_angle(
        self,
        waypoints: np.ndarray,
        speed: float,
        return_debug: bool = False
    ) -> float:

        if len(waypoints) < 2:
            if return_debug:
                return 0.0, {'lookahead': 0.0, 'target_x': 0.0, 'target_y': 0.0, 'alpha': 0.0}
            return 0.0

        if speed < 0.1:  # Avoid division by zero
            speed = 0.1

        # Transform waypoints to rear axle frame
        waypoints = waypoints.copy()  # Don't modify original
        waypoints[:, 0] += self.waypoint_shift

        # Calculate adaptive lookahead distance
        lookahead_distance = np.clip(
            self.K_dd * speed,
            self.min_lookahead,
            self.max_lookahead
        )

        # Find the target point on the path
        target_point = self._get_target_point(lookahead_distance, waypoints)

        if target_point is None:
            # No valid target point found, aim for last waypoint
            target_point = waypoints[-1]

        # Calculate angle to target point (alpha)
        alpha = np.arctan2(target_point[1], target_point[0])

        # Pure Pursuit steering law
        # Formula: steering = atan(2 * L * sin(alpha) / lookahead_distance)
        steering_angle = np.arctan2(
            2.0 * self.wheel_base * np.sin(alpha),
            lookahead_distance
        )

        if return_debug:
            debug_info = {
                'lookahead': float(lookahead_distance),
                'target_x': float(target_point[0]),
                'target_y': float(target_point[1]),
                'alpha': float(alpha)
            }
            return steering_angle, debug_info

        return steering_angle


    def _get_target_point(self, lookahead_distance: float, waypoints: np.ndarray):
        # 1) Keep only points that are in front of the axle (x >= 0)
        ahead_mask = waypoints[:, 0] >= 0.0
        wp_ahead = waypoints[ahead_mask]
        if wp_ahead.size == 0:
            # Nothing ahead; aim at the furthest overall as a safe fallback
            d_all = np.linalg.norm(waypoints, axis=1)
            return waypoints[np.argmax(d_all)]

        # 2) Find first point ahead whose distance >= lookahead
        d = np.linalg.norm(wp_ahead, axis=1)
        hits = np.where(d >= lookahead_distance)[0]
        if len(hits) == 0:
            # No hit; aim at the furthest ahead
            return wp_ahead[np.argmax(d)]

        idx = hits[0]

        # 3) Optional: linear interpolation to hit lookahead exactly
        if idx > 0:
            p1, p2 = wp_ahead[idx - 1], wp_ahead[idx]
            d1, d2 = d[idx - 1], d[idx]
            if d2 - d1 > 1e-3:
                t = np.clip((lookahead_distance - d1) / (d2 - d1), 0.0, 1.0)
                return p1 + t * (p2 - p1)

        return wp_ahead[idx]



def transform_path_to_vehicle_frame(
    path_xy: np.ndarray,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_heading: float
) -> np.ndarray:
    """
    Transform path waypoints from world ENU frame to vehicle body frame.

    Args:
        path_xy: Path waypoints in world ENU coordinates (Nx2 array)
        vehicle_x: Vehicle x position in world ENU (East)
        vehicle_y: Vehicle y position in world ENU (North)
        vehicle_heading: Vehicle heading in ENU radians
                        (0=East, π/2=North, counter-clockwise positive)
                        Use XsensReceiver.get_heading_enu()

    Returns:
        Path waypoints in vehicle frame (x=forward, y=left)
    """
    translated = path_xy - np.array([vehicle_x, vehicle_y])
    rotation_angle = vehicle_heading
    cos_heading = np.cos(rotation_angle)
    sin_heading = np.sin(rotation_angle)

    rotation_matrix = np.array([
        [cos_heading, -sin_heading],
        [sin_heading, cos_heading]
    ])

    vehicle_frame_path = translated @ rotation_matrix
    
    return vehicle_frame_path

