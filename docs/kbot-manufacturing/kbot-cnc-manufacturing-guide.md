# K-Scale KBot CNC Manufacturing Guide

This document provides comprehensive guidance for manufacturing K-Scale KBot humanoid robot parts using CNC machining services.

---

## Table of Contents

1. [KBot Specifications](#kbot-specifications)
2. [RobStride Actuators](#robstride-actuators)
3. [CNC Service Providers](#cnc-service-providers)
4. [Material Selection](#material-selection)
5. [Tolerance Guidelines](#tolerance-guidelines)
6. [Surface Finish Options](#surface-finish-options)
7. [Threading Specifications](#threading-specifications)
8. [JLCCNC Ordering Guide](#jlccnc-ordering-guide)
9. [Design Guidelines](#design-guidelines)
10. [Quick Reference Tables](#quick-reference-tables)

---

## KBot Specifications

### Physical Dimensions
| Parameter | Value |
|-----------|-------|
| Height | 4'7" (1.4m) |
| Weight | 77 lbs (34 kg) |
| Arm Payload | ~10 kg (22 lbs) |
| Battery Life | Up to 4 hours |
| Power System | 48V distributed |
| Total Actuators | 20 (5 per limb) |

### Pricing & Availability
- **Price**: $8,999 (first 100 units)
- **Shipping**: Late 2025

### Communication Interfaces
- **Actuators**: CAN bus
- **Head**: USB

### CAD Resources
| Resource | URL |
|----------|-----|
| Onshape CAD | https://cad.onshape.com/publications/e15cf8edefacbba3009917c0/ |
| GitHub | https://github.com/kscalelabs/kbot (mechanical/ folder) |
| Teardown Video | https://youtube.com/watch?v=qhZi9rtdEKg |

### Licensing
| Component | License |
|-----------|---------|
| Hardware | CERN-OHL-S (strongly reciprocal) |
| Software | GPL v3 |

---

## RobStride Actuators

KBot uses RobStride actuators with 5 actuators per arm. Below is the likely configuration:

### Actuator Specifications

| Model | Peak Torque | Rated Torque | Dimensions (mm) | Weight | Likely Joint |
|-------|-------------|--------------|-----------------|--------|--------------|
| **RS-04** | 120 Nm | 40 Nm | Ø110 × 50 | 1.42 kg | Shoulder pitch/roll |
| **RS-03** | 60 Nm | - | - | - | Elbow |
| **RS-02** | 17 Nm | 6 Nm | 78.5×78.5×45.5 | 405g | Wrist roll |
| **RS-01** | 17 Nm | - | - | - | Wrist pitch |
| **RS-00** | 17 Nm | - | - | - | Hand/gripper |

### RS-04 Mounting Details (Shoulder)

| Parameter | Specification |
|-----------|---------------|
| Outer Diameter | Ø106±0.1 mm |
| Inner Diameter 1 | Ø35.86±0.025 mm |
| Inner Diameter 2 | Ø36±0.1 mm |
| Mounting Screws | 10× M4 EQS, 9× M5 |
| Power Range | 48V (15-60V) |
| Gear Ratio | 9:1 |
| Protection | IP52 |

### RS-02 Mounting Details (Wrist)

| Parameter | Specification |
|-----------|---------------|
| Outer Dimension | Ø78.5 mm |
| Height | 65 mm |
| Outer Tolerance | Ø73±0.2 mm |
| Inner Diameter | Ø24±0.2 mm |
| Mounting Screws | 9× M3, 6× M4 |

---

## CNC Service Providers

### Comparison Table

| Service | Base Price | Lead Time | Best For | Notes |
|---------|-----------|-----------|----------|-------|
| **JLCCNC** | $5+ per part | 3 days + 2-5 day shipping | Budget, most parts | Recommended for prototyping |
| **PCBWay** | $8+ per part | 5-7 days | Combined PCB+CNC orders | Slightly higher pricing |
| **Xometry** | $50+ per part | 3-4 days | US-based, high quality | Premium pricing |
| **Fictiv** | $100+ per part | Variable | Premium, consulting | Robotics/medtech specialty |
| **eMachineShop** | Varies | Varies | Free US shipping | Good for domestic orders |

### JLCCNC Specifications (Recommended)

| Parameter | Specification |
|-----------|---------------|
| Starting Price | $5 per part |
| Lead Time | 3 business days |
| Standard Tolerance | ±0.05mm |
| Surface Finish | Ra 1.6μm |
| Max Part Size | 1100 × 600 × 500 mm |
| Min Size (metals) | 30 × 20 × 5 mm |
| Aluminum Options | 6061, 7075 |
| Anodizing | Available (black, red, gold common) |

---

## Material Selection

### Aluminum Alloy Comparison

| Alloy | Tensile Strength | Best For | Cost | Anodizing | Notes |
|-------|------------------|----------|------|-----------|-------|
| **6061-T6** | 310 MPa | Most robot parts | Lower | Excellent | Easy to machine |
| **7075-T6** | 572 MPa | High-stress parts | 30-50% higher | Harder to achieve even finish | 2× stronger than 6061 |

### Recommendations by Part Type

| Part Category | Recommended Material | Justification |
|---------------|---------------------|---------------|
| Motor mount brackets | 6061-T6 | Good machinability, excellent anodizing |
| Structural links | 6061-T6 | Balanced cost/strength |
| Shoulder/elbow joints | 7075-T6 | Higher stress, needs extra strength |
| Wrist components | 6061-T6 | Lower loads, good finish |
| Decorative covers | 6061-T6 | Best anodizing results |

---

## Tolerance Guidelines

### Tolerance Selection by Application

| Application | Tolerance | Cost Impact | Notes |
|-------------|-----------|-------------|-------|
| Standard parts | ±0.1mm | Baseline | Default, lower cost |
| Bearing fits | ±0.05mm | +10-20% | Critical for smooth operation |
| Motor mounts | ±0.05mm | +10-20% | Ensures proper alignment |
| Decorative parts | ±0.2mm | -10% | Acceptable for non-functional surfaces |
| Shaft holes | ±0.025mm | +30-50% | Precision fits, use sparingly |

### RobStride Actuator Mounting Tolerances

| Feature | Required Tolerance |
|---------|-------------------|
| RS-04 outer bore | ±0.1mm |
| RS-04 shaft bore | ±0.025mm |
| RS-02 mounting surface | ±0.2mm |
| Screw hole positions | ±0.1mm |

---

## Surface Finish Options

### Available Finishes

| Finish | Ra Value | Cost | Use Case |
|--------|----------|------|----------|
| As-machined | 3.2-6.3μm | Baseline | Internal parts, prototypes |
| Bead blasted | ~3.2μm | +5-10% | Matte appearance, hides tool marks |
| Brushed | ~1.6μm | +10-15% | Linear aesthetic |
| Polished | <0.8μm | +20-30% | Decorative, smooth contact surfaces |

### Anodizing Options

| Type | Thickness | Hardness | Best For | Notes |
|------|-----------|----------|----------|-------|
| Type II (Standard) | 8-25μm | Decorative | General protection, aesthetics | Available in colors |
| Type III (Hard coat) | 25-75μm | HV 400-600 | High-wear surfaces | Limited colors (black, natural) |

### Common Anodize Colors
- Black (most common)
- Red
- Blue
- Gold
- Clear/Natural

---

## Threading Specifications

### JLCCNC Standard Thread Capabilities

Thread tapping is **included at no extra charge** for standard metric threads.

| Thread | Tap Drill | Max Depth | Min Wall Thickness |
|--------|-----------|-----------|---------------------|
| M2 | Ø1.6mm | 6mm | 5mm |
| M2.5 | Ø2.05mm | 7.5mm | 6.25mm |
| M3 | Ø2.5mm | 9mm | 7.5mm |
| M4 | Ø3.3mm | 12mm | 10mm |
| M5 | Ø4.2mm | 15mm | 12.5mm |
| M6 | Ø5.0mm | 18mm | 15mm |
| M8 | Ø6.8mm | 24mm | 20mm |

### Thread Design Rules

1. **Effective depth**: ≤ 3× thread diameter
2. **Blind holes**: Add 0.5× diameter for tap runout
3. **Wall thickness**: Minimum 2.5× thread diameter for aluminum
4. **Edge distance**: Minimum 2× thread diameter from edges

### RobStride Mounting Thread Summary

| Actuator | Mount Pattern | Thread Sizes | JLCCNC Compatible |
|----------|---------------|--------------|-------------------|
| RS-04 | 10-hole EQS | M4, M5 | Yes |
| RS-03 | 8-hole | M4 | Yes |
| RS-02/01 | 9-hole + 6-hole | M3, M4 | Yes |
| RS-00 | 6-hole | M3 | Yes |

### When to Tap Yourself

- Thread depth > 3× diameter
- Non-standard thread pitches
- Adding helicoils/thread inserts later
- Custom thread forms

---

## JLCCNC Ordering Guide

### Step 1: Export CAD Files

1. Open KBot CAD in Onshape
2. Select the part to manufacture
3. Right-click → Export → STEP (.stp)
4. Save each part as a separate file

**Supported formats**: STEP (.stp, .step), IGES (.igs), Solidworks (.sldprt), and others

### Step 2: Upload to JLCCNC

1. Navigate to https://jlccnc.com/
2. Click "Instant Quote"
3. Drag and drop or select your STEP file
4. Wait for automatic geometry analysis

### Step 3: Configure Options

| Setting | Options | Recommendation |
|---------|---------|----------------|
| Material | Aluminum 6061, 7075, etc. | 6061 for most parts |
| Quantity | 1+ (no minimum) | Order 2+ for lower per-unit cost |
| Tolerance | ±0.1mm (standard), ±0.05mm (precision) | Use ±0.1mm unless critical |
| Surface Treatment | As machined, bead blast, anodize | Bead blast + anodize for exterior |
| Color | Black, red, gold, natural | Black is most common |

### Step 4: Review and Submit

1. Review instant quote (price and lead time)
2. Check DFM (Design for Manufacturing) feedback
3. Add to cart
4. Complete payment
5. Track order through dashboard

---

## Design Guidelines

### CNC-Friendly Design Rules

| Feature | Minimum | Recommended | Notes |
|---------|---------|-------------|-------|
| Internal corner radius | 0.5mm | R = (depth/10) + 0.5mm | Allows standard end mills |
| Wall thickness (metal) | 0.8mm | 1.5mm+ | Prevents deflection |
| Cavity depth | 3× tool diameter | 2× tool diameter | Better finish quality |
| Thread depth | - | 3× hole diameter max | Standard taps |
| Pocket floor radius | 0.5mm | Match internal corner radius | Consistent tooling |

### Anodizing Preparation

1. **Add hanging holes**: Ø3mm in non-critical areas
2. **Avoid sharp corners**: Internal corners trap anodize solution
3. **Consider masking**: Specify areas to mask from anodizing
4. **Surface prep**: Bead blast before anodizing for best results

### Part Orientation Considerations

- Orient for minimum setup changes
- Consider which surfaces need best finish
- Align critical tolerances with machine axes
- Group features by machining operation

---

## Quick Reference Tables

### Recommended Settings by Part Type

#### Motor Mount Brackets (Critical Fit)

| Setting | Value |
|---------|-------|
| Material | Aluminum 6061-T6 |
| Tolerance | ±0.05mm |
| Surface Finish | Ra 1.6μm |
| Post-processing | Bead blast + Anodize (black) |
| Quantity | 2+ |

#### Structural Links/Housings

| Setting | Value |
|---------|-------|
| Material | Aluminum 6061-T6 |
| Tolerance | ±0.1mm (standard) |
| Surface Finish | As-machined or bead blast |
| Post-processing | Optional anodize |

#### High-Stress Parts (Shoulder/Elbow)

| Setting | Value |
|---------|-------|
| Material | Aluminum 7075-T6 |
| Tolerance | ±0.05mm |
| Surface Finish | Ra 1.6μm |
| Post-processing | Hard coat anodize |
| Note | 30-50% more expensive |

### Cost Optimization Tips

1. **Use standard tolerances** (±0.1mm) where possible
2. **Batch similar parts** together for quantity discounts
3. **Choose 6061 over 7075** unless strength is critical
4. **As-machined finish** for internal/hidden parts
5. **Minimize setup changes** in design
6. **Standard thread sizes** (M3, M4, M5) are included free

---

## Checklist: Before Ordering

- [ ] Clone KBot repo: `git clone https://github.com/kscalelabs/kbot.git`
- [ ] Review mechanical/ folder for STEP files
- [ ] Identify parts that need CNC (vs 3D printing)
- [ ] Check part sizes against manufacturing limits
- [ ] Export STEP files from Onshape for each part
- [ ] Verify tolerance requirements for each part
- [ ] Add anodizing holes if needed
- [ ] Check thread specifications match actuator requirements
- [ ] Get quotes from JLCCNC
- [ ] Review DFM feedback before finalizing

---

## Resources

### K-Scale Community
- **Discord**: Join K-Scale Discord for community support
- **GitHub Issues**: https://github.com/kscalelabs/kbot/issues

### Reference Images
- Motor mapping: `.playwright-mcp/kbot_motor_mapping.png`
- UID reference: `.playwright-mcp/KBot-UID-to-Pic-9073ae4d95d1314072edb5df2b7ac46f.xlsx`

### External Documentation
- [RobStride Actuator Datasheets](https://robstride.com)
- [JLCCNC Design Guidelines](https://jlccnc.com/design-guidelines)
- [Aluminum Alloy Properties](https://www.matweb.com)

---

*Document created: January 2026*
*Last updated: January 2026*
