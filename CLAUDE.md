# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

UTM Navigator is an autonomous vehicle research platform developed at the Autonomous Vehicle Laboratory, Cal Poly Pomona. It's a full-scale AV testbed implementing the complete **sense-plan-control-act** pipeline for GPS-guided autonomous navigation with obstacle avoidance.

## Architecture

```
Sensors → Perception → Planning → Control → Actuators
   ↓          ↓           ↓          ↓          ↓
 xsens    occupancy    navigator  pure_pursuit  vehicle_actuator
 lidar    costmap      dwa        pid           (CAN via Teensy)
 camera   IPM/BEV      ackermann_dwa
```

### Core Modules

- **`sensors/`** - Hardware interfaces (Xsens GPS/IMU, Velodyne LIDAR, cameras)
- **`perception/`** - Occupancy grids (log-odds Bayesian), costmaps, IPM/BEV transforms
- **`planning/`** - Route planning via OSMnx (`Navigator`), DWA local planners (`DWA`, `AckermannDWA`, `AckermannDWACostmap`)
- **`control/`** - Pure Pursuit steering, PID, Ackermann vehicle model
- **`actuators/`** - Teensy CAN interface (`VehicleActuator` for Serial, `VehicleActuatorUDP` for Ethernet)
- **`webui/`** - Phone-based remote control with joystick, voice commands, E-STOP
- **`firmware/`** - Teensy 4.1 CAN firmware (master.cpp, steer.cpp, throttle.cpp, brake.cpp)

### Entry Points

- **`runner.py`** - Main autonomous controller: GPS waypoint following with Pure Pursuit
- **`runner_dwa.py`** - DWA-based obstacle avoidance mode
- **`examples/`** - Standalone demos (DWA, LIDAR BEV, costmap, IPM calibration, manual control)

### Key Data Flow

1. `XsensReceiver` provides GPS position (UTM), heading (ENU), and speed
2. `Navigator` generates route waypoints from OSMnx road network
3. `PurePursuitController` computes steering angle to follow path
4. `VehicleActuator`/`VehicleActuatorUDP` sends commands to Teensy via Serial/Ethernet

## Commands

### Running the Vehicle

```bash
# Main autonomous mode (GPS waypoint following)
python runner.py

# DWA obstacle avoidance mode
python runner_dwa.py

# Manual keyboard control
python examples/manual_control.py
```

### Running Examples

```bash
# DWA with synthetic obstacles
python examples/dwa_synthetic.py
python examples/ackermann_dwa_synthetic.py

# DWA with LIDAR
python examples/dwa_lidar.py
python examples/ackermann_dwa_costmap_lidar.py

# Perception demos
python examples/visualize_bev_lidar.py
python examples/costmap_synthetic.py
```

### Running Tests

```bash
python tests/test_occupancy_grid_fixes.py
python tests/test_bev_lidar_synthetic.py
python tests/test_multilayer_synthetic.py
python tests/test_straight_line_heading.py
python tests/example_mti_receive_data.py  # Xsens sensor validation
```

### WebUI (Phone Remote Control)

```bash
cd webui
pip install -r requirements.txt
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=AVConsole'
python3 server.py
# Open https://<your-ip>:8000 on phone
```

### Building Documentation

```bash
cd docs && pip install -r requirements.txt && make html
```

## Key Technical Details

### Coordinate Systems
- **WGS84** (lat/lon) - GPS input
- **UTM** - Projected metric coordinates for planning/control (auto-detected by OSMnx, e.g., EPSG:32611 for Cal Poly Pomona)
- **Vehicle Frame** - X forward, Y left

### Important Constants
- Max steering angle: 28 degrees (mechanical limit)
- Goal tolerance: 2.0 meters
- Min control speed: 0.1 m/s

### Teensy UDP Protocol

Commands to `192.168.13.177:5005`:
- `E 1|0` - E-stop on/off
- `T 0..1` - Throttle
- `M N|D|S|R` - Mode (Neutral/Drive/Sport/Reverse)
- `B 0..1` - Brake
- `S -1..1` - Steer (left -, right +)
- `A E=0 T=0.5 M=D B=0 S=0.1` - All-in-one command
- `P` - Request state (returns JSON)

Watchdog: 500ms timeout triggers E-stop if no command received.

### Thread Safety
`VehicleState` class in `runner.py` uses `Lock()` for thread-safe sharing between control loop and visualization thread.

### Sensor Interfaces
- **Xsens MTi-680G**: Uses `xsensdeviceapi` SDK, RTK-capable GPS/IMU
- **Velodyne LIDAR**: UDP packet decoding in `examples/velodyne_decoder_final.py`
- **Cameras**: USB/CSI via OpenCV, RealSense via `pyrealsense2`

### Planning Algorithms
- **Pure Pursuit**: Speed-adaptive lookahead distance for path tracking
- **DWA**: Dynamic Window Approach with differential (`DWA`) and Ackermann (`AckermannDWA`, `AckermannDWACostmap`) variants
- **Occupancy Grid**: Log-odds Bayesian update with Bresenham raycasting

## Dependencies

Core: `numpy`, `scipy`, `matplotlib`, `opencv-python`
Planning: `osmnx`, `networkx`, `pyproj`
Sensors: `xsensdeviceapi`, `pyrealsense2` (optional)
Visualization: `open3d` (optional)
WebUI: `fastapi`, `uvicorn`, `websockets`

## Environment Setup

```bash
export PYTHONPATH=/path/to/FINALE:$PYTHONPATH

# For perception module
conda create -n perception python=3.9
conda activate perception
pip install numpy opencv-python open3d pyrealsense2
```
