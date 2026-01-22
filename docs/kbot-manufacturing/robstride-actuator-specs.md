# RobStride Actuator Technical Specifications

Reference document for RobStride actuators used in K-Scale KBot humanoid robot.

---

## Actuator Overview

KBot uses **20 RobStride actuators** total:
- 5 per arm (10 total for upper body)
- 5 per leg (10 total for lower body)

---

## RS-04 (Shoulder/Hip Joints)

### Specifications

| Parameter | Value |
|-----------|-------|
| Peak Torque | 120 Nm |
| Rated Torque | 40 Nm |
| Diameter | Ø110 mm |
| Height | 50 mm |
| Weight | 1.42 kg |
| Gear Ratio | 9:1 |
| Voltage Range | 15-60V (48V nominal) |
| Protection Rating | IP52 |

### Mechanical Dimensions

```
        ┌─────────────────────┐
        │                     │
        │    Ø106±0.1        │  ← Outer mounting flange
        │                     │
        │   ┌───────────┐     │
        │   │           │     │
        │   │ Ø35.86    │     │  ← Output shaft bore (±0.025)
        │   │  ±0.025   │     │
        │   └───────────┘     │
        │                     │
        │    Ø36±0.1         │  ← Inner mounting surface
        │                     │
        └─────────────────────┘
               50mm height
```

### Mounting Pattern

| Feature | Specification |
|---------|---------------|
| Mounting holes (EQS) | 10× M4 |
| Secondary mount | 9× M5 |
| Bolt circle diameter | 94mm (M4), 88mm (M5) |
| Hole depth | 8mm minimum |

### CNC Part Requirements

**Mating bracket requirements:**
- Material: 6061-T6 or 7075-T6
- Bore tolerance: ±0.025mm for shaft
- Face flatness: 0.05mm
- Surface finish: Ra 1.6μm or better
- Anodizing: Type II or III

---

## RS-03 (Elbow/Knee Joints)

### Specifications

| Parameter | Value |
|-----------|-------|
| Peak Torque | 60 Nm |
| Rated Torque | 20 Nm |
| Weight | ~0.9 kg |
| Gear Ratio | 9:1 |
| Voltage Range | 15-60V |

### Mounting Pattern

| Feature | Specification |
|---------|---------------|
| Mounting holes | 8× M4 |
| Bolt circle diameter | ~75mm |

---

## RS-02 (Wrist Roll / Ankle Roll)

### Specifications

| Parameter | Value |
|-----------|-------|
| Peak Torque | 17 Nm |
| Rated Torque | 6 Nm |
| Dimensions | 78.5 × 78.5 × 45.5 mm |
| Weight | 405g |
| Gear Ratio | 6:1 |
| Voltage Range | 15-60V |

### Mechanical Dimensions

```
      ┌───────────────────────┐
      │        78.5mm         │
      │   ┌───────────────┐   │
      │   │   Ø73±0.2    │   │  ← Mounting flange
      │   │               │   │
      │   │  ┌───────┐    │   │
      │   │  │Ø24±0.2│    │   │  ← Output bore
      │   │  └───────┘    │   │
      │   │               │   │
      │   └───────────────┘   │
      └───────────────────────┘
              65mm height
```

### Mounting Pattern

| Feature | Specification |
|---------|---------------|
| Face mount | 9× M3 |
| Side mount | 6× M4 |
| M3 bolt circle | 65mm |
| M4 bolt circle | 70mm |

---

## RS-01 (Wrist Pitch / Ankle Pitch)

### Specifications

| Parameter | Value |
|-----------|-------|
| Peak Torque | 17 Nm |
| Rated Torque | ~5 Nm |
| Weight | ~350g |
| Gear Ratio | 6:1 |

### Mounting Pattern

| Feature | Specification |
|---------|---------------|
| Mounting holes | 8× M3 |
| Secondary mount | 4× M4 |

---

## RS-00 (Hand/Gripper)

### Specifications

| Parameter | Value |
|-----------|-------|
| Peak Torque | 17 Nm |
| Rated Torque | ~4 Nm |
| Weight | ~280g |
| Gear Ratio | 6:1 |

### Mounting Pattern

| Feature | Specification |
|---------|---------------|
| Mounting holes | 6× M3 |

---

## Communication Interface

All RobStride actuators use **CAN bus** communication.

### CAN Specifications

| Parameter | Value |
|-----------|-------|
| Protocol | CAN 2.0B |
| Baud rate | 1 Mbps |
| Connector | JST GH 1.25mm 4-pin |
| Daisy chain | Supported |

### Pinout

| Pin | Function | Color |
|-----|----------|-------|
| 1 | CAN_H | Yellow |
| 2 | CAN_L | Green |
| 3 | GND | Black |
| 4 | +5V (optional) | Red |

---

## Power Distribution

### Per-Actuator Power Requirements

| Actuator | Peak Current | Continuous Current |
|----------|--------------|-------------------|
| RS-04 | 40A | 15A |
| RS-03 | 25A | 10A |
| RS-02 | 12A | 5A |
| RS-01 | 10A | 4A |
| RS-00 | 8A | 3A |

### System Power Budget (One Arm)

| Joint | Actuator | Peak Power | Continuous |
|-------|----------|------------|------------|
| Shoulder pitch | RS-04 | 1920W | 720W |
| Shoulder roll | RS-04 | 1920W | 720W |
| Elbow | RS-03 | 1200W | 480W |
| Wrist roll | RS-02 | 576W | 240W |
| Wrist pitch | RS-01 | 480W | 192W |
| **Total** | - | **6096W** | **2352W** |

---

## Bracket Design Recommendations

### General Guidelines

1. **Material thickness**: Minimum 4mm aluminum for RS-04, 3mm for others
2. **Stiffness**: Add ribs or gussets to prevent flexing
3. **Heat dissipation**: Consider adding heat sink features for high-duty applications
4. **Cable routing**: Include channels for CAN and power cables
5. **Strain relief**: Add mounting points for cable strain relief

### Critical Tolerances for CNC

| Feature | RS-04 | RS-03 | RS-02 | RS-01/00 |
|---------|-------|-------|-------|----------|
| Shaft bore | ±0.025mm | ±0.025mm | ±0.05mm | ±0.05mm |
| Mounting face | ±0.05mm flat | ±0.05mm flat | ±0.1mm flat | ±0.1mm flat |
| Hole positions | ±0.1mm | ±0.1mm | ±0.1mm | ±0.1mm |
| Perpendicularity | 0.1mm/100mm | 0.1mm/100mm | 0.2mm/100mm | 0.2mm/100mm |

---

## Maintenance Notes

- Actuators are sealed but not waterproof (IP52)
- Check mounting bolt torque periodically
- Monitor for unusual sounds or vibration
- Keep CAN connectors clean and secure

---

## Resources

- Official datasheet: Contact RobStride
- CAD models: Available in KBot Onshape
- K-Scale Discord for community support
