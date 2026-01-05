#!/usr/bin/env python3
"""
Control Logger for Autonomous Vehicle Debugging

Logs detailed control state for debugging controller issues:
- Oscillation: lookahead, steering, speed
- Full circles: waypoints ahead, all behind flag
- Path offset: crosstrack error, heading error

Usage:
    logger = ControlLogger()

    # In control loop
    logger.log_iteration(
        timestamp=time.time(),
        iteration=i,
        vehicle_x=x,
        vehicle_y=y,
        heading_rad=heading,
        speed=speed,
        steering_deg=steering,
        rtk_status=status,
        waypoints_vehicle=waypoints_vehicle,
        waypoints_world=waypoints_world,
        distance_to_goal=dist
    )

    # At shutdown
    logger.save_csv("debug_log.csv")
    logger.print_summary()
"""

import time
import csv
import numpy as np
from typing import List, Dict, Any


class ControlLogger:
    """
    Simple synchronous logger for control debugging.

    Designed for easy upgrade to async if needed (just modify _store method).
    """

    def __init__(self):
        """Initialize logger with empty data list"""
        self.data: List[Dict[str, Any]] = []

    def log_iteration(
        self,
        timestamp: float,
        iteration: int,
        vehicle_x: float,
        vehicle_y: float,
        heading_rad: float,
        speed: float,
        steering_deg: float,
        rtk_status: str,
        waypoints_vehicle: np.ndarray,
        waypoints_world: np.ndarray,
        distance_to_goal: float,
        lat: float = 0.0,
        lon: float = 0.0,
        loop_time_ms: float = 0.0,
        lookahead_distance: float = 0.0,
        control_enabled: bool = True,
        target_point_x: float = 0.0,
        target_point_y: float = 0.0,
        alpha: float = 0.0,
        control_mode: str = "N"
    ):
        """
        Log a single control iteration with debug data.

        Args:
            timestamp: Current time (seconds)
            iteration: Loop iteration number
            vehicle_x, vehicle_y: Vehicle position in world frame (UTM)
            heading_rad: Vehicle heading in radians (ENU frame)
            speed: Vehicle speed (m/s)
            steering_deg: Commanded steering angle (degrees)
            rtk_status: GPS RTK status string
            waypoints_vehicle: Waypoints in vehicle frame (Nx2 array)
            waypoints_world: Waypoints in world frame (Nx2 array)
            distance_to_goal: Distance to final goal (meters)
            lat, lon: GPS coordinates (optional)
            loop_time_ms: Loop execution time (optional)
            lookahead_distance: Pure Pursuit lookahead distance (meters)
            control_enabled: Whether steering control is active
            target_point_x, target_point_y: Target point in vehicle frame (meters)
            alpha: Angle to target point (radians)
            control_mode: Drive mode (N/D/S/R)
        """

        # Compute all debug metrics
        log_entry = self._compute_log_entry(
            timestamp=timestamp,
            iteration=iteration,
            vehicle_x=vehicle_x,
            vehicle_y=vehicle_y,
            heading_rad=heading_rad,
            speed=speed,
            steering_deg=steering_deg,
            rtk_status=rtk_status,
            waypoints_vehicle=waypoints_vehicle,
            waypoints_world=waypoints_world,
            distance_to_goal=distance_to_goal,
            lat=lat,
            lon=lon,
            loop_time_ms=loop_time_ms,
            lookahead_distance=lookahead_distance,
            control_enabled=control_enabled,
            target_point_x=target_point_x,
            target_point_y=target_point_y,
            alpha=alpha,
            control_mode=control_mode
        )

        # Store it (separated for easy async upgrade)
        self._store(log_entry)

    def _compute_log_entry(
        self,
        timestamp: float,
        iteration: int,
        vehicle_x: float,
        vehicle_y: float,
        heading_rad: float,
        speed: float,
        steering_deg: float,
        rtk_status: str,
        waypoints_vehicle: np.ndarray,
        waypoints_world: np.ndarray,
        distance_to_goal: float,
        lat: float,
        lon: float,
        loop_time_ms: float,
        lookahead_distance: float,
        control_enabled: bool,
        target_point_x: float,
        target_point_y: float,
        alpha: float,
        control_mode: str
    ) -> Dict[str, Any]:
        """
        Compute all debug metrics from raw data.

        This method does all the computation work. When upgrading to async,
        this stays unchanged - only _store() changes.
        """

        # ===== Waypoint Analysis =====
        # Count waypoints ahead of vehicle (in vehicle frame, x >= 0 is ahead)
        waypoints_ahead = int(np.sum(waypoints_vehicle[:, 0] >= 0))
        waypoints_total = len(waypoints_vehicle)
        all_waypoints_behind = (waypoints_ahead == 0)

        # Find closest waypoint
        if len(waypoints_vehicle) > 0:
            wp_distances = np.linalg.norm(waypoints_vehicle, axis=1)
            closest_wp_dist = float(np.min(wp_distances))
            closest_wp_idx = int(np.argmin(wp_distances))

            # Get next waypoint position (in vehicle frame)
            next_wp_x = float(waypoints_vehicle[closest_wp_idx][0])
            next_wp_y = float(waypoints_vehicle[closest_wp_idx][1])
        else:
            closest_wp_dist = 0.0
            closest_wp_idx = 0
            next_wp_x = 0.0
            next_wp_y = 0.0

        # ===== Crosstrack Error (distance from vehicle to path) =====
        if len(waypoints_world) > 0:
            vehicle_pos = np.array([vehicle_x, vehicle_y])
            path_distances = np.linalg.norm(waypoints_world - vehicle_pos, axis=1)
            crosstrack_error = float(np.min(path_distances))
        else:
            crosstrack_error = 0.0

        # ===== Lateral Error (signed) and Heading Error =====
        lateral_error_signed = 0.0
        heading_error = 0.0

        if len(waypoints_vehicle) >= 2:
            # Find closest two waypoints to compute path direction
            distances = np.linalg.norm(waypoints_vehicle, axis=1)
            closest_idx = np.argmin(distances)

            # Get closest point
            closest_wp = waypoints_vehicle[closest_idx]

            # Lateral error: positive if vehicle is to the left of path
            # In vehicle frame: positive y is left
            lateral_error_signed = float(closest_wp[1])

            # Heading error: angle between vehicle heading and path direction
            # Vehicle is at origin facing +x in vehicle frame
            # Path direction is the angle to the next waypoint
            if len(waypoints_vehicle) > closest_idx + 1:
                next_wp = waypoints_vehicle[closest_idx + 1]
                path_direction = np.arctan2(next_wp[1] - closest_wp[1],
                                             next_wp[0] - closest_wp[0])
                # Vehicle heading in vehicle frame is 0 (facing +x)
                # Heading error is path direction relative to vehicle
                heading_error = float(path_direction)

        # ===== Assemble log entry =====
        return {
            # Timing
            'timestamp': timestamp,
            'iteration': iteration,
            'loop_time_ms': loop_time_ms,

            # Vehicle state
            'vehicle_x': vehicle_x,
            'vehicle_y': vehicle_y,
            'lat': lat,
            'lon': lon,
            'heading_rad': heading_rad,
            'heading_deg': np.rad2deg(heading_rad),
            'speed_ms': speed,
            'steering_deg': steering_deg,
            'rtk_status': rtk_status,

            # Control state
            'control_enabled': control_enabled,
            'control_mode': control_mode,

            # Pure Pursuit controller state
            'lookahead_distance': lookahead_distance,
            'target_point_x_vehicle': target_point_x,
            'target_point_y_vehicle': target_point_y,
            'alpha_rad': alpha,
            'alpha_deg': np.rad2deg(alpha),

            # Waypoint info
            'waypoints_ahead': waypoints_ahead,
            'waypoints_total': waypoints_total,
            'all_waypoints_behind': all_waypoints_behind,
            'closest_waypoint_dist': closest_wp_dist,
            'closest_waypoint_idx': closest_wp_idx,
            'next_waypoint_x_vehicle': next_wp_x,
            'next_waypoint_y_vehicle': next_wp_y,

            # Performance metrics
            'crosstrack_error_m': crosstrack_error,
            'lateral_error_signed_m': lateral_error_signed,
            'heading_error_rad': heading_error,
            'heading_error_deg': np.rad2deg(heading_error),
            'distance_to_goal': distance_to_goal
        }

    def _store(self, log_entry: Dict[str, Any]):
        """
        Store log entry (in-memory, synchronous).

        When upgrading to async, modify this method to put in queue.
        All computation logic in _compute_log_entry stays unchanged.
        """
        self.data.append(log_entry)

    def save_csv(self, filename: str = "controller_debug.csv"):
        """Export logged data to CSV file"""
        if not self.data:
            print("[LOGGER] No data to save")
            return

        try:
            with open(filename, 'w', newline='') as f:
                fieldnames = list(self.data[0].keys())
                writer = csv.DictWriter(f, fieldnames=fieldnames)

                writer.writeheader()
                writer.writerows(self.data)

            print(f"[LOGGER] Saved {len(self.data)} records to {filename}")

        except Exception as e:
            print(f"[LOGGER] ERROR: Failed to save: {e}")

    def print_summary(self):
        """Print basic summary of the run"""
        if not self.data:
            print("[LOGGER] No data collected")
            return

        print("\n" + "="*50)
        print("[LOGGER] SUMMARY")
        print("="*50)

        total = len(self.data)
        duration = self.data[-1]['timestamp'] - self.data[0]['timestamp']

        print(f"Iterations: {total}")
        print(f"Duration: {duration:.1f}s")
        print(f"Avg crosstrack: {np.mean([abs(d['crosstrack_error_m']) for d in self.data]):.2f}m")

        # Critical issues only
        behind_count = sum(1 for d in self.data if d['all_waypoints_behind'])
        if behind_count > 0:
            print(f"\n⚠️  Vehicle lost path {behind_count} times (full circles)")

        print("="*50 + "\n")
