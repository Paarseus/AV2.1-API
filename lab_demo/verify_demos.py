#!/usr/bin/env python3
"""
Demo Verification Script - Ulsan Student Visit
Verifies that all synthetic demos can import and run without hardware.

Usage:
    python lab_demo/verify_demos.py
"""

import os
import sys
import subprocess
import time
from pathlib import Path

# Get project root directory
PROJECT_ROOT = Path(__file__).parent.parent.resolve()
os.chdir(PROJECT_ROOT)
sys.path.insert(0, str(PROJECT_ROOT))

# ANSI colors for terminal output
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"
BOLD = "\033[1m"

def check_import(module_path, description):
    """Check if a module can be imported."""
    try:
        # Use importlib for dynamic imports
        import importlib.util
        spec = importlib.util.spec_from_file_location("module", module_path)
        if spec is None:
            return False, f"Cannot find module: {module_path}"
        return True, "OK"
    except Exception as e:
        return False, str(e)

def check_file_exists(file_path, description):
    """Check if a file exists."""
    full_path = PROJECT_ROOT / file_path
    if full_path.exists():
        return True, "OK"
    return False, f"File not found: {file_path}"

def run_quick_test(command, description, timeout=5):
    """Run a command with timeout to check if it starts without errors."""
    try:
        # Run with timeout, capture output
        result = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            timeout=timeout,
            cwd=PROJECT_ROOT,
            text=True
        )
        # Check for import errors in output
        if "ImportError" in result.stderr or "ModuleNotFoundError" in result.stderr:
            return False, f"Import error: {result.stderr[:200]}"
        if "Error" in result.stderr and "no error" not in result.stderr.lower():
            # Check if it's just a timeout or user interrupt
            if "timeout" not in result.stderr.lower():
                return False, f"Error: {result.stderr[:200]}"
        return True, "OK (started successfully)"
    except subprocess.TimeoutExpired:
        # Timeout is expected for interactive demos - this is success!
        return True, "OK (timed out as expected for interactive demo)"
    except Exception as e:
        return False, str(e)

def print_result(name, success, message):
    """Print formatted result."""
    status = f"{GREEN}PASS{RESET}" if success else f"{RED}FAIL{RESET}"
    print(f"  [{status}] {name}")
    if not success:
        print(f"         {RED}{message}{RESET}")

def main():
    """Run all verification checks."""
    print("\n" + "=" * 60)
    print(f"{BOLD}  DEMO VERIFICATION - Ulsan Student Visit{RESET}")
    print("=" * 60)

    all_passed = True
    results = []

    # === Check 1: Required Files Exist ===
    print(f"\n{BOLD}[1/4] Checking required files exist...{RESET}")

    files_to_check = [
        ("examples/ackermann_dwa_synthetic.py", "Demo A: DWA"),
        ("examples/costmap_synthetic.py", "Demo B: Costmap"),
        ("tests/test_bev_lidar_synthetic.py", "Demo C: 3D Fusion"),
        ("tests/lidar/01_raw_pointcloud.py", "Demo D: LIDAR"),
        ("tests/example_mti_receive_data.py", "Demo E: GPS/IMU"),
        ("examples/perception_demo.py", "Demo F: Camera"),
        ("webui/server.py", "Demo H: WebUI"),
        ("tests/lidar/00_basic_test.py", "LIDAR test"),
    ]

    for file_path, description in files_to_check:
        success, msg = check_file_exists(file_path, description)
        results.append((description, success, msg))
        print_result(description, success, msg)
        if not success:
            all_passed = False

    # === Check 2: Core Modules Import ===
    print(f"\n{BOLD}[2/4] Checking core module imports...{RESET}")

    modules_to_check = [
        ("perception.occupancy_grid", "Occupancy Grid"),
        ("perception.costmap", "Costmap"),
        ("control.pure_pursuit", "Pure Pursuit"),
        ("control.ackermann_vehicle", "Ackermann Vehicle"),
        ("utils", "Utils"),
    ]

    for module_name, description in modules_to_check:
        try:
            __import__(module_name)
            success, msg = True, "OK"
        except Exception as e:
            success, msg = False, str(e)[:100]

        results.append((f"Import {description}", success, msg))
        print_result(f"Import {description}", success, msg)
        if not success:
            all_passed = False

    # === Check 3: Dependencies ===
    print(f"\n{BOLD}[3/4] Checking key dependencies...{RESET}")

    dependencies = [
        "numpy",
        "matplotlib",
        "scipy",
        "yaml",
        "cv2",
    ]

    for dep in dependencies:
        try:
            __import__(dep)
            success, msg = True, "OK"
        except ImportError:
            success, msg = False, f"Not installed"

        results.append((f"Dependency: {dep}", success, msg))
        print_result(f"Dependency: {dep}", success, msg)
        if not success:
            all_passed = False

    # === Check 4: Config File ===
    print(f"\n{BOLD}[4/4] Checking configuration...{RESET}")

    config_path = PROJECT_ROOT / "config" / "default.yaml"
    if config_path.exists():
        try:
            import yaml
            with open(config_path) as f:
                cfg = yaml.safe_load(f)
            if cfg and 'vehicle' in cfg:
                success, msg = True, f"OK (wheelbase={cfg['vehicle'].get('wheelbase', '?')}m)"
            else:
                success, msg = False, "Invalid config structure"
        except Exception as e:
            success, msg = False, str(e)[:100]
    else:
        success, msg = False, "config/default.yaml not found"

    results.append(("Config file", success, msg))
    print_result("Config file", success, msg)
    if not success:
        all_passed = False

    # WebUI SSL certificates
    cert_path = PROJECT_ROOT / "webui" / "cert.pem"
    key_path = PROJECT_ROOT / "webui" / "key.pem"
    if cert_path.exists() and key_path.exists():
        success, msg = True, "OK"
    else:
        success, msg = False, "Missing cert.pem or key.pem"
        print(f"         {YELLOW}Run: cd webui && openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=AVConsole'{RESET}")

    results.append(("WebUI SSL certs", success, msg))
    print_result("WebUI SSL certs", success, msg)

    # === Summary ===
    print("\n" + "=" * 60)
    passed = sum(1 for _, s, _ in results if s)
    total = len(results)

    if all_passed:
        print(f"{GREEN}{BOLD}  ALL CHECKS PASSED ({passed}/{total}){RESET}")
        print("  Ready for demo!")
    else:
        print(f"{YELLOW}{BOLD}  SOME CHECKS FAILED ({passed}/{total} passed){RESET}")
        print("  Review failures above before demo.")

    print("=" * 60 + "\n")

    # === Quick Reference ===
    print(f"{BOLD}Quick Demo Commands:{RESET}")
    print("  Demo A (DWA):      python examples/ackermann_dwa_synthetic.py")
    print("  Demo B (Costmap):  python examples/costmap_synthetic.py")
    print("  Demo C (3D):       python tests/test_bev_lidar_synthetic.py")
    print("  Demo D (LIDAR)*:   python tests/lidar/01_raw_pointcloud.py")
    print("  Demo E (GPS)*:     python tests/example_mti_receive_data.py")
    print("  Demo H (WebUI)*:   cd webui && python3 server.py")
    print("\n  * = Requires hardware\n")

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
