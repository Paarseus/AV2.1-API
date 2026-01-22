#!/usr/bin/env python3
"""
Lab Demo Runner - Ulsan Student Visit
Interactive menu to launch demos during the lab visit.

Usage:
    python lab_demo/demo_runner.py
"""

import os
import sys
import subprocess
from pathlib import Path

# Get project root directory
PROJECT_ROOT = Path(__file__).parent.parent.resolve()
os.chdir(PROJECT_ROOT)

# Set PYTHONPATH so imports work correctly
os.environ['PYTHONPATH'] = str(PROJECT_ROOT)

# Demo definitions: (name, command, description, requires_hardware)
DEMOS = {
    # Algorithm Demos (Synthetic - No Hardware Required)
    'A': ('DWA Path Planning',
          'python examples/ackermann_dwa_synthetic.py',
          'Animated vehicle navigating around obstacles using DWA',
          False),

    'B': ('Occupancy Grid & Costmap',
          'python examples/costmap_synthetic.py',
          'Side-by-side probability grid vs inflated costmap',
          False),

    'C': ('3D Sensor Fusion',
          'python tests/test_bev_lidar_synthetic.py',
          'Interactive 3D view - BEV camera + LIDAR points',
          False),

    # Live Sensor Demos (Hardware Required)
    'D': ('LIDAR Point Cloud (LIVE)',
          'python tests/lidar/01_raw_pointcloud.py',
          'Real-time 3D point cloud from Velodyne LIDAR',
          True),

    'E': ('GPS/IMU Tracking (LIVE)',
          'python tests/example_mti_receive_data.py',
          'Real-time position, heading, velocity from Xsens',
          True),

    'F': ('Camera BEV (LIVE)',
          'python examples/perception_demo.py --camera rgb --camera-id 0 --visualizer 2d',
          'Live camera with Bird\'s Eye View transformation',
          True),

    # Vehicle Control
    'H': ('WebUI Phone Control',
          'cd webui && python3 server.py',
          'Start WebUI server for phone control',
          True),

    # Verification
    'V': ('Verify All Demos',
          'python lab_demo/verify_demos.py',
          'Check that all synthetic demos can start',
          False),

    '0': ('LIDAR Connection Test',
          'python tests/lidar/00_basic_test.py',
          'Verify Velodyne LIDAR is connected',
          True),
}

def print_banner():
    """Print welcome banner."""
    print("\n" + "=" * 60)
    print("  LAB DEMO RUNNER - Ulsan Student Visit")
    print("  Autonomous Vehicle Laboratory, Cal Poly Pomona")
    print("=" * 60)

def print_menu():
    """Print demo menu."""
    print("\n--- ALGORITHM DEMOS (No Hardware) ---")
    for key in ['A', 'B', 'C']:
        name, _, desc, _ = DEMOS[key]
        print(f"  [{key}] {name}")
        print(f"      {desc}")

    print("\n--- LIVE SENSOR DEMOS (Hardware Required) ---")
    for key in ['D', 'E', 'F']:
        name, _, desc, hw = DEMOS[key]
        marker = "*" if hw else " "
        print(f"  [{key}]{marker} {name}")
        print(f"      {desc}")

    print("\n--- VEHICLE CONTROL ---")
    for key in ['H']:
        name, _, desc, _ = DEMOS[key]
        print(f"  [{key}] {name}")
        print(f"      {desc}")

    print("\n--- UTILITIES ---")
    for key in ['V', '0']:
        name, _, desc, hw = DEMOS[key]
        marker = "*" if hw else " "
        print(f"  [{key}]{marker} {name}")
        print(f"      {desc}")

    print("\n  [Q] Quit")
    print("\n  * = Requires hardware\n")

def run_demo(key):
    """Run selected demo."""
    if key not in DEMOS:
        print(f"Unknown demo: {key}")
        return

    name, cmd, desc, hw = DEMOS[key]

    print("\n" + "-" * 50)
    print(f"Starting: {name}")
    if hw:
        print("NOTE: This demo requires hardware!")
    print(f"Command: {cmd}")
    print("-" * 50 + "\n")

    try:
        # Set environment with PYTHONPATH
        env = os.environ.copy()
        env['PYTHONPATH'] = str(PROJECT_ROOT)
        # Use shell=True for cd commands
        subprocess.run(cmd, shell=True, cwd=PROJECT_ROOT, env=env)
    except KeyboardInterrupt:
        print("\n\nDemo interrupted.")
    except Exception as e:
        print(f"\nError running demo: {e}")

    print("\n" + "-" * 50)
    print(f"Demo '{name}' finished")
    print("-" * 50)
    input("\nPress Enter to continue...")

def main():
    """Main interactive loop."""
    print_banner()

    while True:
        print_menu()
        choice = input("Select demo [A-H, V, 0, Q]: ").strip().upper()

        if choice == 'Q':
            print("\nGoodbye!")
            break
        elif choice in DEMOS:
            run_demo(choice)
        else:
            print(f"\nInvalid selection: {choice}")
            print("Please choose from the menu options.")

if __name__ == "__main__":
    main()
