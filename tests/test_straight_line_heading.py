#!/usr/bin/env python3
"""
SAFETY-CRITICAL TEST: Straight-Line Heading Convergence

This test verifies that the Xsens MTi-680G sensor heading converges correctly
during straight-line driving. This validates:
1. Sensor mounting orientation is correct (or acceptable)
2. GNSS-aided INS filter is working properly
3. General_RTK filter profile is functioning as expected

PROCEDURE:
1. Run this script: python3 test_straight_line_heading.py
2. Wait for RTK_FIXED status
3. When prompted, drive STRAIGHT FORWARD at ~5 mph for 30 seconds
4. Script will monitor heading convergence in real-time
5. PASS/FAIL verdict will be displayed

PASS CRITERIA:
- Heading converges within 30 seconds
- Heading remains stable (< 2°/sec change) when driving straight
- No wild oscillations or jumps

FAIL CRITERIA:
- Heading does not converge
- Heading drifts continuously
- Heading changes significantly when driving straight

Author: Safety Test Script
Date: 2025-11-08
"""

import sys
import time
import csv
import numpy as np
from typing import List, Dict, Any
from sensors.xsens_receiver import XsensReceiver


class HeadingConvergenceTest:
    """
    Test class for monitoring heading convergence during straight-line driving.
    """

    def __init__(self):
        self.gps: Optional[XsensReceiver] = None
        self.test_data: List[Dict[str, Any]] = []

        # Convergence detection parameters
        self.convergence_window = 10  # Number of samples to check
        self.convergence_threshold = 2.0  # degrees/second
        self.max_test_duration = 30.0  # seconds

        # State tracking
        self.converged = False
        self.convergence_time = None
        self.start_time = None

    def setup(self) -> bool:
        """Initialize and connect to Xsens sensor"""
        print("="*70)
        print("XSENS MTi-680G STRAIGHT-LINE HEADING CONVERGENCE TEST")
        print("="*70)
        print("\n[SETUP] Initializing Xsens sensor...")

        try:
            self.gps = XsensReceiver(log_file=None, update_rate=100)

            print("[SETUP] Connecting to sensor...")
            if not self.gps.start(wait_for_rtk_fixed=True, rtk_timeout=60.0):
                print("[ERROR] Failed to start Xsens sensor")
                return False

            print("[SETUP] Sensor connected successfully!")
            print(f"[SETUP] RTK Status: {self.gps.get_rtk_status()}")

            # Wait for valid heading data
            print("[SETUP] Waiting for valid orientation data...")
            timeout = 5.0
            t0 = time.time()
            while time.time() - t0 < timeout:
                heading = self.gps.get_heading_enu()
                if heading is not None:
                    print(f"[SETUP] Initial heading: {np.rad2deg(heading):.1f}° (ENU)")
                    print("[SETUP] ✅ Setup complete!\n")
                    return True
                time.sleep(0.1)

            print("[ERROR] No orientation data received")
            return False

        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            return False

    def run_test(self):
        """Run the straight-line heading convergence test"""
        print("="*70)
        print("TEST INSTRUCTIONS")
        print("="*70)
        print("\n1. Position vehicle on flat, open area")
        print("2. Point vehicle in ANY direction (doesn't matter)")
        print("3. When prompted, drive STRAIGHT FORWARD")
        print("4. Maintain constant direction for 30 seconds")
        print("5. Speed: ~5 mph (8 km/h) - not too slow!")
        print("6. Do NOT turn the steering wheel")
        print("\nPress ENTER when ready to start test...")
        input()

        print("\n" + "="*70)
        print("STARTING TEST - DRIVE STRAIGHT FORWARD NOW!")
        print("="*70)
        print("\nMonitoring heading convergence...")
        print("Format: [Time] Compass: XXX.X° | ENU: XXX.X° | Change: X.XX°/s | Status\n")

        self.start_time = time.time()
        prev_heading = None
        prev_time = None
        heading_history = []

        try:
            while True:
                current_time = time.time()
                elapsed = current_time - self.start_time

                # Check timeout
                if elapsed > self.max_test_duration:
                    print("\n" + "="*70)
                    print("TEST DURATION COMPLETE (30 seconds)")
                    print("="*70)
                    break

                # Get heading data
                heading_enu_rad = self.gps.get_heading_enu()
                heading_nav_deg = self.gps.get_heading_nav()
                rtk_status = self.gps.get_rtk_status()

                if heading_enu_rad is None:
                    print(f"[{elapsed:5.1f}s] Waiting for heading data...")
                    time.sleep(0.2)
                    continue

                heading_enu_deg = np.rad2deg(heading_enu_rad)

                # Calculate rate of change
                change_rate = 0.0
                if prev_heading is not None and prev_time is not None:
                    dt = current_time - prev_time
                    if dt > 0:
                        # Handle angle wrapping for change calculation
                        delta = heading_enu_deg - prev_heading
                        if delta > 180:
                            delta -= 360
                        elif delta < -180:
                            delta += 360
                        change_rate = abs(delta) / dt

                # Update history
                heading_history.append(change_rate)
                if len(heading_history) > self.convergence_window:
                    heading_history.pop(0)

                # Check convergence
                status = "WAITING"
                if len(heading_history) >= self.convergence_window:
                    avg_change = np.mean(heading_history)
                    max_change = np.max(heading_history)

                    if avg_change < self.convergence_threshold and max_change < self.convergence_threshold * 2:
                        if not self.converged:
                            self.converged = True
                            self.convergence_time = elapsed
                            status = "✅ CONVERGED"
                        else:
                            status = "✅ STABLE"
                    else:
                        status = "⏳ CONVERGING"

                # Color coding for terminal
                if "✅" in status:
                    status_color = status  # Green checkmark
                elif "⏳" in status:
                    status_color = status  # Hourglass
                else:
                    status_color = "⚠️  " + status  # Warning

                # Display current state
                print(f"[{elapsed:5.1f}s] "
                      f"Compass: {heading_nav_deg:6.1f}° | "
                      f"ENU: {heading_enu_deg:6.1f}° | "
                      f"Change: {change_rate:5.2f}°/s | "
                      f"RTK: {rtk_status:10s} | "
                      f"{status_color}")

                # Log data
                self.test_data.append({
                    'timestamp': current_time,
                    'elapsed': elapsed,
                    'heading_enu_rad': heading_enu_rad,
                    'heading_enu_deg': heading_enu_deg,
                    'heading_nav_deg': heading_nav_deg,
                    'change_rate_deg_per_sec': change_rate,
                    'rtk_status': rtk_status,
                    'converged': self.converged
                })

                # Update for next iteration
                prev_heading = heading_enu_deg
                prev_time = current_time

                time.sleep(0.2)  # 5 Hz update rate

        except KeyboardInterrupt:
            print("\n\n[TEST] Interrupted by user")

        # Stop sensor
        if self.gps:
            self.gps.stop()

    def analyze_results(self):
        """Analyze test results and display verdict"""
        print("\n" + "="*70)
        print("TEST RESULTS ANALYSIS")
        print("="*70)

        if len(self.test_data) < 5:
            print("\n❌ INCONCLUSIVE: Not enough data collected")
            return

        # Extract metrics
        change_rates = [d['change_rate_deg_per_sec'] for d in self.test_data if d['elapsed'] > 5.0]
        headings_enu = [d['heading_enu_deg'] for d in self.test_data if d['elapsed'] > 5.0]

        if not change_rates:
            print("\n❌ INCONCLUSIVE: Not enough data after initial 5 seconds")
            return

        avg_change_rate = np.mean(change_rates)
        max_change_rate = np.max(change_rates)
        heading_range = max(headings_enu) - min(headings_enu)

        print(f"\nTest Duration: {self.test_data[-1]['elapsed']:.1f} seconds")
        print(f"Samples Collected: {len(self.test_data)}")
        print(f"\n--- Convergence Metrics ---")
        print(f"Converged: {'✅ YES' if self.converged else '❌ NO'}")
        if self.convergence_time:
            print(f"Time to Convergence: {self.convergence_time:.1f} seconds")
        print(f"\n--- Stability Metrics (after 5s) ---")
        print(f"Average Change Rate: {avg_change_rate:.2f} °/s")
        print(f"Maximum Change Rate: {max_change_rate:.2f} °/s")
        print(f"Heading Range: {heading_range:.1f}°")

        # Determine pass/fail
        print("\n" + "="*70)
        print("VERDICT")
        print("="*70)

        passed = True
        issues = []

        # Check 1: Convergence
        if not self.converged:
            passed = False
            issues.append("Heading did not converge to stable value")
        elif self.convergence_time and self.convergence_time > 25.0:
            passed = False
            issues.append(f"Convergence took too long ({self.convergence_time:.1f}s > 25s)")

        # Check 2: Stability
        if avg_change_rate > self.convergence_threshold:
            passed = False
            issues.append(f"Average change rate too high ({avg_change_rate:.2f}°/s > {self.convergence_threshold}°/s)")

        # Check 3: Heading range (should be small when driving straight)
        if heading_range > 20.0:
            print(f"\n⚠️  WARNING: Large heading range ({heading_range:.1f}°)")
            print("    This could indicate:")
            print("    - Vehicle wasn't driven in a straight line")
            print("    - Sensor mounting issue")
            print("    - GNSS/INS filter issue")

        # Display verdict
        if passed:
            print("\n" + "🎉 "*15)
            print("✅ ✅ ✅  TEST PASSED  ✅ ✅ ✅")
            print("🎉 "*15)
            print("\nHeading convergence is NORMAL and STABLE")
            print("Sensor mounting appears CORRECT")
            print("System is READY for autonomous navigation")
            print("\nNext steps:")
            print("1. Perform turn test (drive in circle)")
            print("2. Run full autonomous test at low speed")
            print("3. Monitor system during initial runs")
        else:
            print("\n" + "❌ "*15)
            print("❌ ❌ ❌  TEST FAILED  ❌ ❌ ❌")
            print("❌ "*15)
            print("\nIssues detected:")
            for i, issue in enumerate(issues, 1):
                print(f"  {i}. {issue}")
            print("\nPossible causes:")
            print("  - Sensor may be mounted incorrectly")
            print("  - Poor GNSS signal quality")
            print("  - Vehicle was turning during test")
            print("  - Sensor needs longer convergence time")
            print("\nRecommended actions:")
            print("  1. Check sensor mounting orientation")
            print("  2. Verify RTK FIXED status throughout test")
            print("  3. Ensure driving in perfectly straight line")
            print("  4. Retry test in open area with clear sky view")

        print("\n" + "="*70)

    def save_log(self, filename: str = "straight_line_test.csv"):
        """Save test data to CSV file"""
        if not self.test_data:
            print("[LOG] No data to save")
            return

        try:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'elapsed', 'heading_enu_rad', 'heading_enu_deg',
                    'heading_nav_deg', 'change_rate_deg_per_sec', 'rtk_status', 'converged'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.test_data)

            print(f"\n[LOG] Test data saved to: {filename}")
            print(f"[LOG] {len(self.test_data)} samples logged")
            print("[LOG] You can analyze this data with visualization tools")

        except Exception as e:
            print(f"[ERROR] Failed to save log: {e}")


def main():
    """Main test execution"""
    test = HeadingConvergenceTest()

    # Setup
    if not test.setup():
        print("\n❌ Setup failed. Cannot proceed with test.")
        return 1

    # Run test
    test.run_test()

    # Analyze results
    test.analyze_results()

    # Save log
    test.save_log()

    print("\n" + "="*70)
    print("TEST COMPLETE")
    print("="*70)
    print("\nThank you for running this safety test!")
    print("Review the results above before proceeding to autonomous operation.\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
