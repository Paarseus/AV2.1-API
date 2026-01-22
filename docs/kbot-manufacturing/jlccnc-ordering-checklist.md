# JLCCNC Ordering Quick-Start Checklist

Step-by-step guide for ordering KBot CNC parts from JLCCNC.

---

## Pre-Order Preparation

### CAD File Preparation

- [ ] Clone KBot repository
  ```bash
  git clone https://github.com/kscalelabs/kbot.git
  cd kbot/mechanical
  ```

- [ ] Access Onshape CAD
  - URL: https://cad.onshape.com/publications/e15cf8edefacbba3009917c0/

- [ ] Identify parts to manufacture
  - [ ] List all CNC-required parts
  - [ ] Note quantities needed
  - [ ] Check for left/right mirror variants

- [ ] Export STEP files
  - Right-click part in Onshape
  - Select Export → STEP (.stp)
  - Use descriptive filenames (e.g., `shoulder_bracket_left.stp`)

### Design Review

- [ ] Check internal corner radii (minimum 0.5mm)
- [ ] Verify wall thickness (minimum 1.5mm for aluminum)
- [ ] Add anodizing holes if not present (Ø3mm in hidden areas)
- [ ] Confirm thread specifications match actuator requirements
- [ ] Check for overhangs that need support

---

## JLCCNC Order Process

### Step 1: Create Account

- [ ] Go to https://jlccnc.com/
- [ ] Create account (can use JLCPCB credentials if you have one)
- [ ] Verify email

### Step 2: Upload Files

- [ ] Click "Instant Quote"
- [ ] Upload STEP file
- [ ] Wait for geometry analysis (usually < 30 seconds)
- [ ] Review auto-detected features

### Step 3: Configure Part Settings

**For Motor Mount Brackets:**

| Setting | Recommended Value |
|---------|-------------------|
| Material | Aluminum 6061-T6 |
| Tolerance | ±0.05mm (Precision) |
| Surface Treatment | Bead blast |
| Post-processing | Anodize Type II |
| Color | Black |
| Quantity | 2+ (for cost savings) |

**For Structural Links:**

| Setting | Recommended Value |
|---------|-------------------|
| Material | Aluminum 6061-T6 |
| Tolerance | ±0.1mm (Standard) |
| Surface Treatment | As machined or Bead blast |
| Post-processing | Optional anodize |
| Quantity | As needed |

**For High-Stress Parts:**

| Setting | Recommended Value |
|---------|-------------------|
| Material | Aluminum 7075-T6 |
| Tolerance | ±0.05mm (Precision) |
| Surface Treatment | Bead blast |
| Post-processing | Hard anodize Type III |
| Quantity | As needed |

### Step 4: Review DFM Feedback

- [ ] Check for manufacturability warnings
- [ ] Address any flagged issues:
  - [ ] Too thin walls
  - [ ] Too small radii
  - [ ] Impossible to machine features
- [ ] Re-upload corrected files if needed

### Step 5: Specify Threading

- [ ] Select threaded holes from feature list
- [ ] Specify thread size (M2, M3, M4, M5, etc.)
- [ ] Verify thread depth is within limits (max 3× diameter)
- [ ] Standard threading is included free

### Step 6: Add Remarks (Optional)

Include any special instructions:
- Critical dimensions to prioritize
- Surface finish priorities
- Assembly notes
- Inspection requirements

### Step 7: Review Quote

- [ ] Verify per-part price
- [ ] Check lead time (typically 3-5 business days)
- [ ] Review shipping options and costs
- [ ] Calculate total including shipping

### Step 8: Submit Order

- [ ] Add to cart
- [ ] Review final order summary
- [ ] Select shipping method
  - Standard: 5-7 days
  - Express: 2-3 days
- [ ] Complete payment
- [ ] Save order confirmation

---

## Post-Order

### Tracking

- [ ] Monitor order status in dashboard
- [ ] Save tracking number when shipped
- [ ] Estimate delivery date

### Receiving Inspection

- [ ] Verify all parts received
- [ ] Check for shipping damage
- [ ] Inspect critical dimensions
  - [ ] Bore diameters
  - [ ] Hole positions
  - [ ] Surface finish
- [ ] Test fit with actuators (if available)
- [ ] Document any issues

### Quality Issues

If parts don't meet specifications:
- [ ] Document with photos and measurements
- [ ] Contact JLCCNC support within 7 days
- [ ] Request remake or refund

---

## Quick Reference: Common Settings

### Material Selection Guide

| Use Case | Material | Notes |
|----------|----------|-------|
| Most parts | 6061-T6 | Default choice |
| High strength | 7075-T6 | 30-50% more expensive |
| Prototype only | 6061-T6 | Skip anodizing to save cost |

### Tolerance Guide

| Feature Type | Tolerance |
|--------------|-----------|
| General dimensions | ±0.1mm |
| Bearing bores | ±0.05mm |
| Motor shaft bores | ±0.025mm |
| Non-critical | ±0.2mm |

### Lead Time Expectations

| Order Complexity | Manufacturing | Shipping | Total |
|------------------|---------------|----------|-------|
| Simple parts | 3 days | 5-7 days | ~10 days |
| Complex parts | 5-7 days | 5-7 days | ~12 days |
| With anodizing | +1-2 days | 5-7 days | ~14 days |
| Express | 2 days | 2-3 days | ~5 days |

---

## Cost Saving Tips

1. **Batch orders**: Order multiple parts together for volume discount
2. **Standard tolerance**: Use ±0.1mm unless precision is needed
3. **Skip anodizing**: For internal/hidden parts
4. **Combine with PCB order**: If using JLCPCB, shipping can be combined
5. **Avoid 7075**: Use only where 6061 strength is insufficient
6. **Standard threads**: M3, M4, M5 are included free

---

## Troubleshooting Common Issues

### "File not supported"
- Ensure STEP format (.stp or .step)
- Check file isn't corrupted
- Try re-exporting from CAD

### "Feature too small"
- Increase internal corner radii to 0.5mm+
- Increase wall thickness to 1.5mm+
- Combine very thin features

### "Quote too high"
- Reduce precision tolerance where possible
- Skip optional finishes
- Increase quantity for volume discount
- Consider alternative material

### "Long lead time"
- Complex parts take longer
- Hard anodizing adds time
- Consider splitting into simpler parts
