# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UTM Navigator is an autonomous vehicle research platform developed at the Autonomous Vehicle Laboratory, Cal Poly Pomona. It's a full-scale AV testbed implementing the **sense-plan-control-act** pipeline for GPS-guided autonomous navigation with obstacle avoidance.

## Architecture

```
Sensors → Perception → Planning → Control → Actuators
   ↓          ↓           ↓          ↓          ↓
 xsens    occupancy    navigator  pure_pursuit  vehicle_actuator
 lidar    costmap      dwa        pid           (CAN via Teensy)
 camera   IPM/BEV      ackermann_dwa
```

### Key Data Flow

1. `XsensReceiver` (`sensors/xsens_receiver.py`) provides GPS position (UTM), heading (ENU), and speed via `xsensdeviceapi` SDK
2. `Navigator` (`planning/navigator.py`) generates route waypoints from OSMnx road network, handles WGS84↔UTM transforms via `pyproj`
3. `PurePursuitController` (`control/pure_pursuit.py`) computes steering angle; use `transform_path_to_vehicle_frame()` before calling
4. `VehicleActuator` (Serial) or `VehicleActuatorUDP` (Ethernet) sends commands to Teensy

### Entry Points

- **`runner.py`** - Main autonomous controller: GPS waypoint following with Pure Pursuit, control in background thread, visualization in main thread
- **`runner_dwa.py`** - DWA-based obstacle avoidance mode (synthetic obstacles)
- **`runner_dwa_lidar.py`** - DWA with live Velodyne LIDAR obstacle detection, ego-centric frame
- **`examples/`** - Standalone demos (DWA, LIDAR BEV, costmap, IPM calibration, manual control)

## Commands

### Running the Vehicle

```bash
python runner.py              # Main autonomous mode (Pure Pursuit + GPS)
python runner_dwa.py          # DWA obstacle avoidance (synthetic)
python runner_dwa_lidar.py    # DWA with live LIDAR
python examples/manual_control.py  # Manual keyboard control
```

### Running Tests

```bash
# Individual test files (no pytest framework)
python tests/test_occupancy_grid_fixes.py
python tests/test_bev_lidar_synthetic.py
python tests/test_multilayer_synthetic.py
python tests/test_straight_line_heading.py
python tests/example_mti_receive_data.py  # Xsens sensor validation

# LIDAR integration tests (in order)
python tests/lidar/00_basic_test.py       # Velodyne connection check
python tests/lidar/01_raw_pointcloud.py   # Raw point cloud visualization
python tests/lidar/02_occupancy_grid.py   # Grid mapping from LIDAR
python tests/lidar/03_costmap.py          # Costmap with inflation
```

### Running Examples

```bash
# DWA planners
python examples/dwa_synthetic.py
python examples/ackermann_dwa_synthetic.py
python examples/ackermann_dwa_costmap_lidar.py

# Perception
python examples/visualize_bev_lidar.py
python examples/costmap_synthetic.py
```

### WebUI (Phone Remote Control)

```bash
cd webui && pip install -r requirements.txt
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=AVConsole'
python3 server.py  # Open https://<your-ip>:8000 on phone
```

## Configuration

All parameters in `config/default.yaml`. Copy to `config/local.yaml` to customize.

```python
from utils import load_config
cfg = load_config()  # Loads default.yaml (or local.yaml if present)
wheelbase = cfg['vehicle']['wheelbase']
```

Key config sections: `system`, `navigation`, `vehicle`, `control.pure_pursuit`, `control.pid`, `planning.dwa`, `planning.ackermann_dwa`, `perception.occupancy_grid`, `actuators`, `sensors`

## Key Technical Details

### Coordinate Systems
- **WGS84** (lat/lon) - GPS input
- **UTM** - Projected metric coordinates for planning/control (auto-detected by OSMnx, e.g., EPSG:32611)
- **Vehicle Frame** - X forward, Y left (origin at vehicle center)
- **ENU** - East-North-Up for heading (0=East, π/2=North, CCW positive)

### Important Constants (from config)
- Max steering angle: 28° (mechanical limit)
- Wheelbase: 1.23m
- Goal tolerance: 2.0m
- Control rate: 20 Hz
- Watchdog timeout: 500ms (Teensy triggers E-stop)

### Teensy UDP Protocol

Commands to `192.168.13.177:5005`:
```
E 1|0           - E-stop on/off
T 0..1          - Throttle
M N|D|S|R       - Mode (Neutral/Drive/Sport/Reverse)
B 0..1          - Brake
S -1..1         - Steer (left -, right +)
A E=0 T=0.5 M=D B=0 S=0.1  - All-in-one command
P               - Request state (returns JSON)
```

### Thread Safety
`VehicleState` in `runner.py` uses `Lock()` for thread-safe sharing. Always use `.update()` and `.get()` methods.

### Perception Notes
- **OccupancyGrid2D**: Log-odds Bayesian update (-5.0 to 5.0 internal, 0.0 to 1.0 probability external), Bresenham raycasting
- **Costmap**: Builds on occupancy grid, adds inflation for robot radius

## Module Quick Reference

| Module | Key Classes | Purpose |
|--------|-------------|---------|
| `sensors/` | `XsensReceiver`, `VelodyneLIDAR` | GPS/IMU via xsensdeviceapi SDK, LIDAR via UDP |
| `perception/` | `OccupancyGrid2D`, `Costmap` | Probabilistic grid mapping |
| `planning/` | `Navigator`, `DWA`, `AckermannDWA`, `AckermannDWACostmap` | Route planning, local planners |
| `control/` | `PurePursuitController`, `AckermannVehicle`, `PID` | Path tracking, vehicle model, speed control |
| `actuators/` | `VehicleActuator`, `VehicleActuatorUDP` | Serial/Ethernet to Teensy |
| `utils/` | `load_config`, `ControlLogger`, `VibrationLogger` | Config loader, data logging |

## Dependencies

Core: `numpy`, `scipy`, `matplotlib`, `opencv-python`, `pyyaml`
Planning: `osmnx`, `networkx`, `pyproj`, `shapely`
Sensors: `xsensdeviceapi`, `velodyne-decoder`, `pyrealsense2` (optional)
Visualization: `open3d` (optional)
WebUI: `fastapi`, `uvicorn`, `websockets`

## Environment Setup

```bash
export PYTHONPATH=/path/to/FINALE:$PYTHONPATH
```
