#!/usr/bin/env python3
"""
DWA Controller for Ackermann Vehicle with LIDAR

Real-time obstacle avoidance using Dynamic Window Approach with live LIDAR data.
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Transformer

from sensors.xsens_receiver import XsensReceiver
from sensors.lidar_interface import VelodyneLIDAR, LidarState
from actuators import VehicleActuatorUDP
from planning import AckermannDWACostmap
from perception import OccupancyGrid2D, Costmap
from control import PID
from utils import load_config


class DWALidarRunner:
    """DWA-based vehicle controller with live LIDAR obstacle detection."""

    def __init__(self):
        # Load config
        cfg = load_config()
        self.cfg = cfg

        # Vehicle
        veh_cfg = cfg['vehicle']
        self.wheelbase = veh_cfg['wheelbase']
        self.max_steering_deg = veh_cfg['max_steering_deg']
        self.robot_radius = veh_cfg['robot_radius']
        self.safety_margin = veh_cfg['safety_margin']

        # Speed
        self.max_speed = cfg['planning']['ackermann_dwa']['max_speed']

        # Grid
        grid_cfg = cfg['perception']['occupancy_grid']
        self.grid_size = grid_cfg['width']
        self.grid_resolution = grid_cfg['resolution']
        self.grid_decay = grid_cfg['decay_factor']

        # LIDAR
        lidar_cfg = cfg['sensors']['lidar']
        self.lidar_port = lidar_cfg['port']
        self.lidar_z_min = lidar_cfg['z_min']
        self.lidar_z_max = lidar_cfg['z_max']
        self.lidar_range_min = lidar_cfg.get('range_min', 0.5)
        self.lidar_range_max = lidar_cfg.get('range_max', 50.0)

        # Navigation
        self.goal_tolerance = cfg['planning']['goal_tolerance']
        self.goal_distance = 20.0  # Goal distance ahead
        self.control_rate_hz = cfg['system']['control_rate_hz']

        # Actuator
        udp_cfg = cfg['actuators']['udp']
        self.actuator_ip = udp_cfg['teensy_ip']
        self.actuator_port = udp_cfg['teensy_port']
        self.min_throttle = cfg['actuators'].get('min_throttle', 0.0)
        self.max_throttle = cfg['actuators']['max_throttle']

        # PID speed controller
        pid_cfg = cfg['control']['pid']
        self.speed_pid = PID(
            kp=pid_cfg['kp'],
            ki=pid_cfg['ki'],
            kd=pid_cfg['kd'],
            output_min=pid_cfg['output_min'],
            output_max=pid_cfg['output_max']
        )

        # Runtime state
        self.gps = None
        self.lidar = None
        self.lidar_state = None
        self.actuator = None
        self.dwa = None
        self.grid = None
        self.costmap = None
        self.start_utm = None
        self.goal_utm = None
        self.running = False

        # UTM transformer (WGS84 -> UTM Zone 11N)
        self.utm_transformer = Transformer.from_crs("EPSG:4326", "EPSG:32611", always_xy=True)

    def gps_to_utm(self, lat, lon):
        """Convert GPS to UTM coordinates."""
        return self.utm_transformer.transform(lon, lat)

    def setup_gps(self, require_rtk=False, timeout=30.0):
        """Initialize GPS and wait for fix."""
        print("[GPS] Starting...")

        try:
            self.gps = XsensReceiver(log_file=None, update_rate=100)
            if not self.gps.start(wait_for_rtk_fixed=require_rtk, rtk_timeout=60.0, fail_on_timeout=False):
                print("[GPS] Failed to start")
                return False

            deadline = time.time() + timeout
            while time.time() < deadline:
                pos = self.gps.get_current_position()
                if pos and self.gps.is_fresh():
                    lat, lon, _ = pos
                    self.start_utm = self.gps_to_utm(lat, lon)
                    print(f"[GPS] Position: {lat:.7f}, {lon:.7f}")
                    print(f"[GPS] UTM: {self.start_utm[0]:.1f}, {self.start_utm[1]:.1f}")
                    return True
                time.sleep(0.1)

            print("[GPS] Timeout waiting for fix")
            return False

        except Exception as e:
            print(f"[GPS] Error: {e}")
            return False

    def setup_lidar(self):
        """Initialize LIDAR and wait for data."""
        print("[LIDAR] Starting...")

        try:
            self.lidar = VelodyneLIDAR(port=self.lidar_port)
            if not self.lidar.connect() or not self.lidar.start():
                print("[LIDAR] Connection failed")
                return False

            self.lidar_state = LidarState(self.lidar)

            # Wait for data
            for i in range(25):
                if self.lidar_state.get_points() is not None:
                    print(f"[LIDAR] Data received (z_min={self.lidar_z_min}, z_max={self.lidar_z_max})")
                    return True
                time.sleep(0.2)

            print("[LIDAR] No data received")
            return False

        except Exception as e:
            print(f"[LIDAR] Error: {e}")
            return False

    def setup_dwa(self):
        """Initialize DWA planner and costmap."""
        print("[DWA] Setting up...")

        self.grid = OccupancyGrid2D(
            width=self.grid_size,
            height=self.grid_size,
            resolution=self.grid_resolution,
            decay_factor=self.grid_decay,
            use_raycasting=False
        )

        self.costmap = Costmap(
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin
        )

        # Use config for all DWA parameters
        dwa_cfg = self.cfg['planning']['ackermann_dwa']
        self.dwa = AckermannDWACostmap(config={
            'max_speed': dwa_cfg['max_speed'],
            'min_speed': dwa_cfg['min_speed'],
            'wheelbase': self.wheelbase,
            'max_steering': dwa_cfg['max_steering'],
            'max_steering_rate': dwa_cfg['max_steering_rate'],
            'max_accel': dwa_cfg['max_accel'],
            'predict_time': dwa_cfg['predict_time'],
            'dt': dwa_cfg['dt'],
            'v_resolution': dwa_cfg['v_resolution'],
            'steering_resolution': dwa_cfg['steering_resolution'],
            # Cost gains (note: config keys differ from DWA internal names)
            'heading_cost_gain': dwa_cfg['to_goal_cost_gain'],
            'speed_cost_gain': dwa_cfg['speed_cost_gain'],
            'obstacle_cost_gain': dwa_cfg['obstacle_cost_gain'],
            'dist_cost_gain': dwa_cfg['goal_dist_cost_gain'],
        })

        print(f"[DWA] Grid: {self.grid_size}m, speed: {dwa_cfg['max_speed']} m/s")
        return True

    def setup_goal(self):
        """Set goal ahead of current heading."""
        print("[GOAL] Setting up...")

        heading = self._wait_for_heading()
        if heading is None:
            return False

        print(f"[GOAL] Heading: {math.degrees(heading):.1f} deg ENU")

        cos_h, sin_h = math.cos(heading), math.sin(heading)
        self.goal_utm = (
            self.start_utm[0] + self.goal_distance * cos_h,
            self.start_utm[1] + self.goal_distance * sin_h
        )

        print(f"[GOAL] Set {self.goal_distance}m ahead")
        return True

    def _wait_for_heading(self, attempts=10, delay=0.5):
        """Wait for valid heading from GPS."""
        for i in range(attempts):
            heading = self.gps.get_heading_enu()
            if heading is not None and not np.isnan(heading):
                return heading
            print(f"[GOAL] Waiting for heading... ({i + 1}/{attempts})")
            time.sleep(delay)

        print("[GOAL] ERROR: No valid heading from GPS")
        return None

    def setup_actuators(self):
        """Initialize UDP actuator connection."""
        print("[ACT] Setting up...")

        try:
            self.actuator = VehicleActuatorUDP(ip=self.actuator_ip, port=self.actuator_port)
            self.actuator.estop(False)
            self.actuator.set_mode("D")
            self.actuator.set_throttle(0.0)
            self.actuator.set_brake(0.0)
            self.actuator.set_steer_norm(0.0)

            if self.actuator.ping():
                print(f"[ACT] Connected @ {self.actuator_ip}:{self.actuator_port}")
            else:
                print(f"[ACT] Warning: No ping response (continuing)")

            return True
        except Exception as e:
            print(f"[ACT] Error: {e}")
            self.actuator = None
            return False

    def run(self):
        """Main control loop."""
        if not all([self.gps, self.lidar_state, self.dwa, self.costmap, self.goal_utm]):
            print("[RUN] Not initialized")
            return

        dt = 1.0 / self.control_rate_hz
        self.running = True
        steering_rad = 0.0
        iteration = 0

        # Precompute goal in grid frame (relative to start)
        goal_grid = (
            self.goal_utm[0] - self.start_utm[0],
            self.goal_utm[1] - self.start_utm[1]
        )

        # Visualization
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 8))
        extent = [
            -self.grid.origin_x, self.grid.width - self.grid.origin_x,
            -self.grid.origin_y, self.grid.height - self.grid.origin_y
        ]
        trajectory = []

        print(f"[RUN] Starting at {self.control_rate_hz}Hz")

        try:
            while self.running:
                t0 = time.time()

                # Get sensor data
                pos = self.gps.get_current_position()
                if not pos or not self.gps.is_fresh():
                    time.sleep(dt)
                    continue

                lat, lon, _ = pos
                heading = self.gps.get_heading_enu()
                speed = self.gps.get_speed() or 0.0

                if heading is None or np.isnan(heading):
                    time.sleep(dt)
                    continue

                # Update grid from LIDAR
                points = self.lidar_state.get_points()
                if points is not None and len(points) > 0:
                    # Height + range filter
                    ranges = np.hypot(points[:, 0], points[:, 1])
                    mask = ((points[:, 2] > self.lidar_z_min) &
                            (points[:, 2] < self.lidar_z_max) &
                            (ranges > self.lidar_range_min) &
                            (ranges < self.lidar_range_max))
                    filtered = points[mask]
                    if len(filtered) > 0:
                        self.grid.update(filtered[:, :2])

                # Update costmap
                self.costmap.update(self.grid, threshold=0.55)

                # Convert to grid frame
                veh_utm = self.gps_to_utm(lat, lon)
                veh_grid = (veh_utm[0] - self.start_utm[0], veh_utm[1] - self.start_utm[1])

                # Check goal
                dist_to_goal = np.hypot(veh_grid[0] - goal_grid[0], veh_grid[1] - goal_grid[1])
                if dist_to_goal < self.goal_tolerance:
                    print(f"[RUN] Goal reached! {dist_to_goal:.1f}m")
                    break

                # DWA planning
                state = (veh_grid[0], veh_grid[1], heading, speed, steering_rad)
                v_cmd, steering_cmd = self.dwa.compute_velocity(state, goal_grid, self.costmap)

                # Clip steering to physical limit
                steering_deg = np.clip(
                    math.degrees(steering_cmd),
                    -self.max_steering_deg,
                    self.max_steering_deg
                )
                steering_rad = math.radians(steering_deg)

                # PID speed control
                speed_error = v_cmd - speed
                throttle = self.speed_pid.compute(speed_error, dt)

                # Actuate
                if self.actuator:
                    self.actuator.set_steer_deg(-steering_deg)

                    if v_cmd > 0.01:
                        throttle = np.clip(throttle, self.min_throttle, self.max_throttle)
                        self.actuator.set_throttle(throttle)
                        self.actuator.set_brake(0.0)
                    else:
                        self.actuator.set_throttle(0.0)
                        self.actuator.set_brake(0.0)
                        self.speed_pid.reset()

                # Log every 10 iterations
                if iteration % 10 == 0:
                    n_obs = len(self.costmap.get_obstacle_points())
                    print(f"[{iteration:4d}] spd:{speed:.2f} v:{v_cmd:.2f} thr:{throttle:.2f} str:{steering_deg:+.1f} goal:{dist_to_goal:.1f}m obs:{n_obs}")

                # Visualize every 10 iterations (for performance)
                trajectory.append(veh_grid)
                if iteration % 10 == 0:
                    self._update_plot(ax, extent, trajectory, state, v_cmd, steering_cmd, steering_deg, goal_grid, dist_to_goal, throttle)

                # Maintain loop rate
                elapsed = time.time() - t0
                time.sleep(max(0, dt - elapsed))
                iteration += 1

        except KeyboardInterrupt:
            print("\n[RUN] Stopped by user")
        finally:
            self.running = False
            plt.ioff()
            plt.close()

    def _update_plot(self, ax, extent, trajectory, state, v_cmd, steering_cmd, steering_deg, goal_grid, dist_to_goal, throttle):
        """Update visualization."""
        ax.clear()

        rgb = self.costmap.to_rgb()
        if rgb is not None:
            ax.imshow(rgb, origin='upper', extent=extent)

        # Trajectory
        if len(trajectory) > 1:
            traj = np.array(trajectory)
            ax.plot(traj[:, 0], traj[:, 1], 'b-', lw=2)

        # Predicted path
        pred = self.dwa.predict_trajectory(state, v_cmd, steering_cmd)
        ax.plot(pred[:, 0], pred[:, 1], 'c-', lw=2)

        # Vehicle
        veh_x, veh_y, heading = state[0], state[1], state[2]
        ax.add_patch(plt.Circle((veh_x, veh_y), self.costmap.inflation_radius,
                                color='b', fill=False, lw=2, ls='--'))
        ax.arrow(veh_x, veh_y, math.cos(heading) * 1.5, math.sin(heading) * 1.5,
                 head_width=0.3, color='blue')

        # Goal
        ax.plot(goal_grid[0], goal_grid[1], 'm*', ms=15)

        ax.set_xlim(-5, goal_grid[0] + 5)
        ax.set_ylim(-15, 15)
        ax.set_aspect('equal')

        n_obs = len(self.costmap.get_obstacle_points())
        ax.set_title(f'v:{v_cmd:.2f} thr:{throttle:.2f} str:{steering_deg:.1f}° goal:{dist_to_goal:.1f}m obs:{n_obs}')
        plt.pause(0.001)

    def stop(self):
        """Shutdown all systems."""
        self.running = False

        if self.actuator:
            try:
                self.actuator.set_throttle(0.0)
                self.actuator.set_brake(1.0)
                self.actuator.set_steer_norm(0.0)
                self.actuator.close()
            except Exception:
                pass

        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except Exception:
                pass

        if self.gps:
            self.gps.stop()

        print("[SYS] Stopped")


def main():
    print("DWA Controller - LIDAR Obstacle Avoidance\n")

    runner = DWALidarRunner()

    try:
        if not runner.setup_gps(require_rtk=True):
            return 1

        if not runner.setup_lidar():
            return 2

        if not runner.setup_dwa():
            return 3

        if not runner.setup_goal():
            return 4

        runner.setup_actuators()

        input("\nPress ENTER to start...")
        runner.run()

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        runner.stop()

    return 0


if __name__ == "__main__":
    exit(main())
