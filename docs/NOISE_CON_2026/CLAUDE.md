# CLAUDE.md - NOISE-CON 2026 Project

This file provides context for Claude Code when working on the NOISE-CON 2026 paper project.

## Project Overview

This is a research paper for **NOISE-CON 2026** ("Good Vibrations" conference) on vibration-based road surface classification using the DRIVE autonomous vehicle platform's existing IMU.

**Paper Title**: "Vibration-Based Road Surface Classification for Low-Speed Autonomous Campus Vehicles: An Educational Platform Approach"

**Conference Deadlines**:
- Abstract: February 20, 2026
- Full Paper: May 26, 2026
- Conference: July 9-11, 2026, Long Beach, CA

## Key Files

### This Project
```
docs/NOISE_CON_2026/
├── NOISE_CON_2026_paper.tex    # Main IEEE paper (draft with TODOs)
├── references.bib              # Bibliography
├── figures/                    # Paper figures (to be added)
└── CLAUDE.md                   # This file
```

### Related Codebase Files (to be created)
```
utils/vibration_logger.py           # High-rate IMU logging (TODO)
perception/vibration_features.py    # Feature extraction (TODO)
perception/road_classifier.py       # ML classification (TODO)
examples/road_surface_demo.py       # Real-time demo (TODO)
```

### Existing Files to Leverage
```
sensors/xsens_receiver.py           # IMU data at 100 Hz (acceleration, gyro)
utils/control_logger.py             # Existing logging infrastructure
config/default.yaml                 # Configuration parameters
runner.py                           # Main autonomous loop
```

## Technical Context

### IMU Data Available (from XsensReceiver)
- `acceleration`: {"east", "north", "up"} - Free acceleration in ENU frame at 100 Hz
- `angular_velocity_body`: {"p", "q", "r"} - Calibrated gyroscope at 100 Hz
- `get_current_position()`: GPS lat/lon for geo-tagging

### Classification Approach
1. **Window**: 1.0 second (100 samples), 50% overlap
2. **Features**: Time-domain (RMS, peak-to-peak, kurtosis) + Frequency-domain (FFT, spectral centroid)
3. **Models**: Random Forest (baseline), CNN-LSTM (target)
4. **Classes**: Smooth asphalt, Rough asphalt, Speed bumps, Potholes, Concrete

### Target Metrics
- Classification accuracy: >90%
- Inference latency: <50 ms
- Real-time rate: 2 classifications/second

## Development Workflow

### Phase 1: Data Collection (Next Step)
1. Create `utils/vibration_logger.py` - High-rate logging with GPS tags
2. Define campus routes for each surface type
3. Collect data at 5, 10, 15 mph

### Phase 2: Feature Extraction
1. Create `perception/vibration_features.py`
2. Implement time-domain features (RMS, peak-to-peak, crest factor, kurtosis, ZCR)
3. Implement frequency-domain features (FFT, spectral centroid, band powers)

### Phase 3: Classification
1. Create `perception/road_classifier.py`
2. Train Random Forest baseline
3. Train CNN-LSTM
4. Evaluate and compare

### Phase 4: Integration
1. Add road condition to `VehicleState` in `runner.py`
2. Create real-time demo
3. Optional: Speed adaptation based on road condition

### Phase 5: Paper Completion
1. Fill in [TODO] sections in LaTeX
2. Generate figures
3. Write results section

## Commands

### Build LaTeX Paper
```bash
cd /home/mspacman/Desktop/Navigator/UTM_Version/FINALE/docs/NOISE_CON_2026
pdflatex NOISE_CON_2026_paper.tex
bibtex NOISE_CON_2026_paper
pdflatex NOISE_CON_2026_paper.tex
pdflatex NOISE_CON_2026_paper.tex
```

### Test IMU Data Access
```bash
cd /home/mspacman/Desktop/Navigator/UTM_Version/FINALE
python -c "
from sensors.xsens_receiver import XsensReceiver
xs = XsensReceiver()
# This will test sensor connection
"
```

## Key Research References

1. **Souza 2025** (Scientific Reports) - CNN-LSTM multi-IMU, 5 surface types
2. **Rahman 2024** (Scientific Reports) - Data representation comparison, 93.4% accuracy
3. **Jain 2024** (MDPI) - Visual + IMU fusion
4. **Lee 2023** (IEEE ITSC) - Road roughness via AI

## Notes

- IMU mounted near vehicle center of mass on DRIVE platform
- Using General_RTK filter profile (no magnetometer due to metal chassis)
- Vertical acceleration (Up component) is primary signal for road roughness
- Vehicle is 350 lb electric UTV with 1.23m wheelbase

## Links

- NOISE-CON 2026: https://www.xcdsystem.com/inceusa/program/RKIPxmk/index.cfm
- Main DRIVE paper: `docs/ASEE_Zone4_2026_DRIVE_v3.tex`
- Platform config: `config/default.yaml`

## Previous Plan

The full implementation plan is at:
`/home/mspacman/.claude/plans/bubbly-hopping-cray.md`

This includes detailed timeline, risk mitigation, and backup topics (mobile noise mapping).
