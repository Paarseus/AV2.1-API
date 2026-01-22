# KBot Arm Bill of Materials (BOM)

Template for tracking parts needed to build one KBot arm assembly.

---

## Actuators (5 per arm)

| Item | Part Number | Qty | Unit Price | Subtotal | Source | Notes |
|------|-------------|-----|------------|----------|--------|-------|
| Shoulder Pitch Motor | RS-04 | 1 | $TBD | $TBD | RobStride | 120Nm peak |
| Shoulder Roll Motor | RS-04 | 1 | $TBD | $TBD | RobStride | 120Nm peak |
| Elbow Motor | RS-03 | 1 | $TBD | $TBD | RobStride | 60Nm peak |
| Wrist Roll Motor | RS-02 | 1 | $TBD | $TBD | RobStride | 17Nm peak |
| Wrist Pitch Motor | RS-01 | 1 | $TBD | $TBD | RobStride | 17Nm peak |
| **Actuator Subtotal** | - | **5** | - | **$TBD** | - | - |

---

## CNC Machined Parts

### Shoulder Assembly

| Item | Material | Qty | Tolerance | Finish | Est. Price | Notes |
|------|----------|-----|-----------|--------|------------|-------|
| Shoulder Base Bracket | 7075-T6 | 1 | ±0.05mm | Anodize | ~$25-40 | Mounts to torso |
| Shoulder Pitch Mount | 6061-T6 | 1 | ±0.05mm | Anodize | ~$20-35 | RS-04 mount |
| Shoulder Roll Mount | 6061-T6 | 1 | ±0.05mm | Anodize | ~$20-35 | RS-04 mount |
| Shoulder Cover (L) | 6061-T6 | 1 | ±0.1mm | Anodize | ~$15-25 | Aesthetic cover |
| Shoulder Cover (R) | 6061-T6 | 1 | ±0.1mm | Anodize | ~$15-25 | Aesthetic cover |

### Upper Arm Assembly

| Item | Material | Qty | Tolerance | Finish | Est. Price | Notes |
|------|----------|-----|-----------|--------|------------|-------|
| Upper Arm Link | 6061-T6 | 1 | ±0.1mm | Anodize | ~$20-35 | Structural link |
| Upper Arm Cover | 6061-T6 | 1 | ±0.2mm | Anodize | ~$10-20 | Optional |
| Elbow Motor Mount | 6061-T6 | 1 | ±0.05mm | Anodize | ~$15-25 | RS-03 mount |

### Forearm Assembly

| Item | Material | Qty | Tolerance | Finish | Est. Price | Notes |
|------|----------|-----|-----------|--------|------------|-------|
| Forearm Link | 6061-T6 | 1 | ±0.1mm | Anodize | ~$15-30 | Structural link |
| Forearm Cover | 6061-T6 | 1 | ±0.2mm | Anodize | ~$10-20 | Optional |
| Wrist Roll Mount | 6061-T6 | 1 | ±0.05mm | Anodize | ~$12-20 | RS-02 mount |
| Wrist Pitch Mount | 6061-T6 | 1 | ±0.05mm | Anodize | ~$10-18 | RS-01 mount |

### CNC Parts Summary

| Category | Part Count | Est. Total |
|----------|------------|------------|
| Shoulder Assembly | 5 | ~$95-160 |
| Upper Arm Assembly | 3 | ~$45-80 |
| Forearm Assembly | 4 | ~$47-88 |
| **CNC Subtotal** | **12** | **~$185-330** |

---

## Fasteners

### Metric Socket Head Cap Screws (SHCS)

| Size | Length | Qty | Use | Source |
|------|--------|-----|-----|--------|
| M4 × 10mm | SHCS | 40 | RS-04 mounting | McMaster/Amazon |
| M4 × 12mm | SHCS | 20 | Bracket assembly | McMaster/Amazon |
| M5 × 12mm | SHCS | 36 | RS-04 secondary | McMaster/Amazon |
| M5 × 16mm | SHCS | 10 | Structural joints | McMaster/Amazon |
| M3 × 8mm | SHCS | 36 | RS-02/01 mounting | McMaster/Amazon |
| M3 × 10mm | SHCS | 18 | Cover attachment | McMaster/Amazon |

### Other Fasteners

| Item | Size | Qty | Use |
|------|------|-----|-----|
| M4 Lock Nut | Nyloc | 20 | Secure joints |
| M5 Lock Nut | Nyloc | 10 | Secure joints |
| M4 Washer | Standard | 40 | Under bolt heads |
| M5 Washer | Standard | 20 | Under bolt heads |
| M4 × 6mm | Set screw | 8 | Shaft clamping |

### Fastener Subtotal: ~$20-40

---

## Bearings (if applicable)

| Type | ID × OD × H | Qty | Est. Price | Notes |
|------|-------------|-----|------------|-------|
| Deep groove | 35 × 55 × 10mm | 2 | ~$10 ea | Shoulder support |
| Deep groove | 24 × 37 × 7mm | 2 | ~$6 ea | Wrist support |
| Needle roller | As spec | 2 | ~$8 ea | Optional |

### Bearing Subtotal: ~$48-80

---

## Electrical

| Item | Qty | Est. Price | Notes |
|------|-----|------------|-------|
| CAN cable assembly | 5 | ~$5 ea | JST GH 4-pin |
| Power cable (14 AWG) | 2m | ~$10 | 48V distribution |
| Power connectors | 5 | ~$3 ea | XT60 or similar |
| Cable sleeve | 2m | ~$8 | Protection/aesthetics |

### Electrical Subtotal: ~$58-75

---

## Cost Summary (One Arm)

| Category | Estimated Cost |
|----------|----------------|
| Actuators (5×) | $TBD (check RobStride) |
| CNC Parts | $185-330 |
| Fasteners | $20-40 |
| Bearings | $48-80 |
| Electrical | $58-75 |
| **Hardware Subtotal** | **$310-525** |
| **+ Actuators** | **+ $TBD** |

---

## Ordering Notes

### Quantity Discounts

- Order 2 arms worth of parts for per-unit savings
- JLCCNC offers discounts at 5+ quantity
- Fasteners cheaper in bulk (100 packs)

### Lead Times

| Item | Lead Time |
|------|-----------|
| CNC Parts (JLCCNC) | 3-5 days + 5-7 shipping |
| Actuators | TBD (check K-Scale) |
| Fasteners (domestic) | 2-5 days |
| Bearings (McMaster) | 1-3 days |

### Spares Recommendation

| Item | Recommended Spares |
|------|-------------------|
| M3 screws | +20% |
| M4 screws | +20% |
| M5 screws | +10% |
| Washers | +50% |
| CAN cables | +1 |
| Critical CNC parts | +0 (order as needed) |

---

## Part Number Reference

Fill in actual part numbers from KBot CAD:

| Description | KBot Part # | STEP Filename | Notes |
|-------------|-------------|---------------|-------|
| Shoulder Base | | | |
| Shoulder Pitch Mount | | | |
| Shoulder Roll Mount | | | |
| Upper Arm Link | | | |
| Elbow Mount | | | |
| Forearm Link | | | |
| Wrist Roll Mount | | | |
| Wrist Pitch Mount | | | |

---

*Template - Update with actual part numbers from KBot Onshape CAD*
