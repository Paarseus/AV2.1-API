# Lab Demo Materials - Ulsan Student Visit

**Date:** January 22, 2026
**Groups:** 61 students total (30 + 31)
**Duration:** 80 minutes per group

## Contents

| File | Description |
|------|-------------|
| `demo_plan.tex` | Full demo plan with schedule, talking points, and backup plans |
| `checklist.tex` | One-page printable checklist for the day of the visit |
| `demo_runner.py` | Interactive menu to launch demos |
| `verify_demos.py` | Pre-visit verification script |

## Quick Start

### 1. Verify everything works (morning of visit)
```bash
python lab_demo/verify_demos.py
```

### 2. Run demos interactively
```bash
python lab_demo/demo_runner.py
```

### 3. Compile LaTeX documents (optional)
```bash
cd lab_demo
pdflatex demo_plan.tex
pdflatex checklist.tex
```

## Demo Commands Reference

**Recommended:** Use the demo runner which handles PYTHONPATH automatically:
```bash
python lab_demo/demo_runner.py
```

**Manual commands** (run from project root with PYTHONPATH set):
```bash
export PYTHONPATH=.

# Algorithm Demos (No Hardware Required)
python examples/ackermann_dwa_synthetic.py    # Demo A: DWA
python examples/costmap_synthetic.py          # Demo B: Costmap
python tests/test_bev_lidar_synthetic.py      # Demo C: 3D Fusion

# Live Sensor Demos (Hardware Required)
python tests/lidar/01_raw_pointcloud.py       # Demo D: LIDAR
python tests/example_mti_receive_data.py      # Demo E: GPS/IMU
python examples/perception_demo.py --camera rgb --camera-id 0  # Demo F

# Vehicle Control
cd webui && python3 server.py                 # Demo H: WebUI
```

## Schedule

| Time | Activity | Duration |
|------|----------|----------|
| 0:00 | Welcome & Research Overview | 12 min |
| 0:12 | Algorithm Visualization Demos | 20 min |
| 0:32 | Live Sensor Demos | 15 min |
| 0:47 | Hands-On Vehicle Interaction | 25 min |
| 1:12 | Q&A and Discussion | 8 min |
