#!/usr/bin/env python3
"""
Vibration Logger for Road Surface Classification

High-rate IMU logger with real-time keyboard labeling.

Usage:
    python utils/vibration_logger.py --output data/run1.csv

Keys:  A=asphalt  S=sidewalk  G=grass  B=bump  Q=quit
"""

import os
import sys
import csv
import time
import select
import tty
import termios
import argparse
from datetime import datetime
from threading import Thread, Event

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sensors.xsens_receiver import XsensReceiver

LABEL_KEYS = {'a': 'asphalt', 's': 'sidewalk', 'g': 'grass', 'b': 'bump'}
COLUMNS = ['timestamp', 'elapsed', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
           'lat', 'lon', 'speed', 'rtk_status', 'label']


class VibrationLogger:
    """High-rate IMU logger with keyboard labeling."""

    def __init__(self, output_file, rate=100.0):
        self.output_file = output_file
        self.sample_interval = 1.0 / rate
        self.xsens = None
        self.label = 'asphalt'
        self.sample_count = 0
        self.start_time = None
        self.label_counts = {}
        self._stop = Event()
        self._old_term = None

    def start(self, wait_for_gps=True):
        """Start Xsens and begin logging."""
        print(f"[LOGGER] Output: {self.output_file}")

        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)

        self.xsens = XsensReceiver(update_rate=100)
        if not self.xsens.start(wait_for_rtk_fixed=False):
            print("[LOGGER] ERROR: Failed to start Xsens")
            return False

        if wait_for_gps:
            print("[LOGGER] Waiting for GPS...")
            for _ in range(60):
                pos = self.xsens.get_current_position()
                if pos and pos[0]:
                    print(f"[LOGGER] GPS: {pos[0]:.6f}, {pos[1]:.6f}")
                    break
                time.sleep(0.5)

        self._csv = open(self.output_file, 'w', newline='')
        self._writer = csv.DictWriter(self._csv, fieldnames=COLUMNS)
        self._writer.writeheader()

        # Setup terminal for raw input
        self._old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.start_time = time.time()
        self._stop.clear()
        Thread(target=self._log_loop, daemon=True).start()
        return True

    def _log_loop(self):
        """Background logging at target rate."""
        next_time = time.time()

        while not self._stop.is_set():
            now = time.time()
            if now < next_time:
                time.sleep(0.001)
                continue

            data = self.xsens.get_all_data()
            acc = data.get('acceleration') or {}
            gyro = data.get('angular_velocity_body') or {}
            vel = data.get('velocity') or {}

            east = vel.get('east', 0.0) or 0.0
            north = vel.get('north', 0.0) or 0.0

            row = {
                'timestamp': now,
                'elapsed': f"{now - self.start_time:.3f}",
                'ax': acc.get('east', 0.0) or 0.0,
                'ay': acc.get('north', 0.0) or 0.0,
                'az': acc.get('up', 0.0) or 0.0,
                'gx': gyro.get('p', 0.0) or 0.0,
                'gy': gyro.get('q', 0.0) or 0.0,
                'gz': gyro.get('r', 0.0) or 0.0,
                'lat': data.get('latitude') or 0.0,
                'lon': data.get('longitude') or 0.0,
                'speed': (east**2 + north**2) ** 0.5,
                'rtk_status': data.get('rtk_status', 'UNKNOWN'),
                'label': self.label,
            }

            self._writer.writerow(row)
            self.sample_count += 1
            self.label_counts[self.label] = self.label_counts.get(self.label, 0) + 1

            next_time += self.sample_interval
            if next_time < now:
                next_time = now + self.sample_interval

    def check_key(self):
        """Check for keypress, return key or None."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def stop(self):
        """Stop logging."""
        self._stop.set()
        time.sleep(0.1)

        if self._old_term:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)

        if hasattr(self, '_csv'):
            self._csv.close()
        if self.xsens:
            self.xsens.stop()

        elapsed = time.time() - self.start_time if self.start_time else 0
        print(f"\n\n[LOGGER] Done: {self.sample_count:,} samples, {elapsed:.1f}s")
        print(f"[LOGGER] File: {self.output_file}")

        if self.label_counts:
            print("[LOGGER] Labels:")
            for lbl, cnt in sorted(self.label_counts.items()):
                print(f"    {lbl}: {cnt:,} ({100*cnt/self.sample_count:.1f}%)")


def main():
    parser = argparse.ArgumentParser(description='Log IMU data with labeling')
    parser.add_argument('--output', '-o',
                        default=f'data/vibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
    parser.add_argument('--duration', '-d', type=float, default=0)
    parser.add_argument('--no-gps-wait', action='store_true')
    args = parser.parse_args()

    logger = VibrationLogger(args.output)
    if not logger.start(wait_for_gps=not args.no_gps_wait):
        sys.exit(1)

    print("\n" + "="*50)
    print("  [A] Asphalt  [S] Sidewalk  [G] Grass  [B] Bump")
    print("  [Q] Quit")
    print("="*50 + "\n")

    end_time = time.time() + args.duration if args.duration > 0 else float('inf')

    try:
        while time.time() < end_time:
            key = logger.check_key()
            if key == 'q':
                break
            elif key in LABEL_KEYS:
                logger.label = LABEL_KEYS[key]

            elapsed = time.time() - logger.start_time
            pos = logger.xsens.get_current_position() if logger.xsens else (None, None)
            lat = f"{pos[0]:.6f}" if pos and pos[0] else "---"
            lon = f"{pos[1]:.6f}" if pos and pos[1] else "---"

            print(f"\r  {elapsed:6.1f}s | {logger.label.upper():8s} | "
                  f"{logger.sample_count:,} samples | GPS: {lat}, {lon}",
                  end='', flush=True)
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        logger.stop()


if __name__ == '__main__':
    main()
