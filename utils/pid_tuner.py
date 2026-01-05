#!/usr/bin/env python3
"""
PID Tuning Tool for Vehicle Speed Control (UDP)

Usage:
    python pid_tuner.py --kp 0.3 --ki 0.05 --kd 0.0 --target 1.0
    python pid_tuner.py --ip 192.168.13.177 --udp-port 5005 --target 2.0

Press Ctrl+C to stop and save data.
"""

import sys
import os
import time
import argparse
import csv
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control import PID
from actuators import VehicleActuatorUDP
from sensors.xsens_receiver import XsensReceiver


def main():
    parser = argparse.ArgumentParser(description="PID Tuning Tool")
    parser.add_argument("--kp", type=float, default=0.3)
    parser.add_argument("--ki", type=float, default=0.05)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--target", type=float, default=1.0, help="Target speed m/s")
    parser.add_argument("--max-throttle", type=float, default=0.4)
    parser.add_argument("--duration", type=float, default=15.0, help="Test duration")
    parser.add_argument("--ip", type=str, default="192.168.13.177", help="Teensy IP")
    parser.add_argument("--udp-port", type=int, default=5005, help="Teensy UDP port")
    args = parser.parse_args()

    # Setup
    print(f"PID Tuner: Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    print(f"Target: {args.target} m/s, Max throttle: {args.max_throttle}")

    pid = PID(kp=args.kp, ki=args.ki, kd=args.kd,
              output_min=0.0, output_max=args.max_throttle)

    actuator = VehicleActuatorUDP(ip=args.ip, port=args.udp_port)
    gps = XsensReceiver(log_file=None, update_rate=100)

    # Start GPS
    if not gps.start():
        print("ERROR: GPS failed to start")
        return 1

    print("Waiting for GPS data...")
    time.sleep(2)

    # Prepare logging
    log_data = []
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"pid_tune_{timestamp}.csv"

    # Start vehicle
    actuator.set_mode("D")
    actuator.set_throttle(0.0)
    actuator.set_brake(0.0)
    actuator.estop(False)

    print(f"\nStarting {args.duration}s test in 3 seconds...")
    time.sleep(3)
    print("GO!\n")

    dt = 0.05
    start_time = time.time()

    try:
        while time.time() - start_time < args.duration:
            loop_start = time.time()

            # Get speed and compute control
            speed = gps.get_speed()
            error = args.target - speed
            throttle = pid.compute(error, dt)

            # Apply throttle
            actuator.set_throttle(throttle)

            # Log
            elapsed = time.time() - start_time
            log_data.append({
                'time': elapsed,
                'target': args.target,
                'actual': speed,
                'error': error,
                'throttle': throttle
            })

            # Print
            print(f"t={elapsed:5.1f}  target={args.target:.2f}  "
                  f"speed={speed:.2f}  error={error:+.2f}  throttle={throttle:.3f}")

            # Maintain loop rate
            elapsed_loop = time.time() - loop_start
            if elapsed_loop < dt:
                time.sleep(dt - elapsed_loop)

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        # Stop vehicle
        print("\nBraking...")
        actuator.set_throttle(0.0)
        actuator.set_brake(1.0)

        # Save data
        if log_data:
            with open(filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=log_data[0].keys())
                writer.writeheader()
                writer.writerows(log_data)
            print(f"\nSaved to {filename}")

            # Print summary
            speeds = [d['actual'] for d in log_data]
            print(f"\nResults: max={max(speeds):.2f}, final={speeds[-1]:.2f}, "
                  f"overshoot={(max(speeds)-args.target)/args.target*100:.1f}%")

        # Cleanup
        gps.stop()
        actuator.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
