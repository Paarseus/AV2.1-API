# CNC Cost Estimation Worksheet

Calculator for estimating JLCCNC part costs for KBot manufacturing.

---

## Cost Factors

### Base Part Cost Formula

```
Total Part Cost = Base Price + Material Markup + Tolerance Markup + Finish Cost + Threading
```

### JLCCNC Pricing Structure

| Factor | Impact |
|--------|--------|
| Minimum per part | $5 |
| Base machining | Volume-based (~$0.01-0.05/mm³) |
| Material | 6061 baseline, 7075 +30-50% |
| Tolerance | Standard ±0.1mm baseline, ±0.05mm +10-20% |
| Surface finish | As-machined baseline |
| Anodizing | +$2-8 per part (size dependent) |
| Threading | Included for standard metric |

---

## Material Cost Multipliers

| Material | Multiplier | Notes |
|----------|------------|-------|
| Aluminum 6061-T6 | 1.0× | Baseline |
| Aluminum 7075-T6 | 1.3-1.5× | High strength |
| Aluminum 5052 | 1.0× | Good forming |
| Stainless 304 | 2.0-2.5× | Harder to machine |
| Stainless 316 | 2.5-3.0× | Corrosion resistant |
| Brass | 1.5× | Good machinability |
| Copper | 2.0× | Soft, gums tools |

---

## Tolerance Cost Impact

| Tolerance | Cost Multiplier | Use Case |
|-----------|-----------------|----------|
| ±0.2mm | 0.9× | Non-critical dimensions |
| ±0.1mm | 1.0× | Standard (default) |
| ±0.05mm | 1.15× | Precision fits |
| ±0.025mm | 1.3× | Critical bores |
| ±0.01mm | 1.5× | High precision |

---

## Surface Finish Costs

| Finish | Additional Cost | Notes |
|--------|-----------------|-------|
| As machined | +$0 | Default |
| Bead blasted | +$1-3 | Matte appearance |
| Brushed | +$2-5 | Directional finish |
| Polished | +$5-15 | Mirror finish |
| Anodize Type II | +$2-8 | Color options |
| Anodize Type III | +$5-15 | Hard coat |
| Powder coat | +$5-10 | Thick durable finish |
| Nickel plate | +$8-20 | Corrosion protection |

---

## Part Size Estimations

### Small Parts (< 50mm)

| Example | Est. Base Price |
|---------|-----------------|
| Small bracket | $5-10 |
| Motor mount adapter | $8-15 |
| Spacer | $5-8 |
| Small cover | $6-12 |

### Medium Parts (50-150mm)

| Example | Est. Base Price |
|---------|-----------------|
| Arm link section | $15-30 |
| Motor mount bracket | $12-25 |
| Housing half | $20-40 |
| Structural plate | $10-20 |

### Large Parts (150-300mm)

| Example | Est. Base Price |
|---------|-----------------|
| Full arm link | $25-50 |
| Torso mounting plate | $30-60 |
| Large bracket | $20-45 |
| Enclosure | $40-80 |

---

## Estimation Worksheet

### Per-Part Calculator

Fill in for each part:

| Field | Part 1 | Part 2 | Part 3 | Part 4 |
|-------|--------|--------|--------|--------|
| Part Name | | | | |
| Size Category | S/M/L | S/M/L | S/M/L | S/M/L |
| Base Est. | $ | $ | $ | $ |
| Material | 6061/7075 | | | |
| Mat. Mult. | 1.0×/1.3× | | | |
| Tolerance | ±0.1/±0.05 | | | |
| Tol. Mult. | 1.0×/1.15× | | | |
| Finish | | | | |
| Finish Add | +$ | +$ | +$ | +$ |
| Quantity | | | | |
| **Unit Cost** | $ | $ | $ | $ |
| **Line Total** | $ | $ | $ | $ |

---

## Example Calculations

### Motor Mount Bracket (RS-04)

| Component | Value | Calculation |
|-----------|-------|-------------|
| Size | Medium | ~100mm envelope |
| Base estimate | $18 | Medium complexity |
| Material | 6061-T6 | × 1.0 |
| Tolerance | ±0.05mm | × 1.15 |
| Adjusted base | $20.70 | 18 × 1.0 × 1.15 |
| Bead blast | +$2 | |
| Anodize | +$4 | Black Type II |
| **Unit price** | **~$27** | Rounded |
| Quantity | 2 | |
| **Line total** | **~$54** | |

### Upper Arm Link

| Component | Value | Calculation |
|-----------|-------|-------------|
| Size | Large | ~200mm length |
| Base estimate | $32 | Medium complexity |
| Material | 6061-T6 | × 1.0 |
| Tolerance | ±0.1mm | × 1.0 |
| Adjusted base | $32 | 32 × 1.0 × 1.0 |
| Bead blast | +$3 | Larger part |
| Anodize | +$5 | |
| **Unit price** | **~$40** | Rounded |
| Quantity | 1 | |
| **Line total** | **~$40** | |

---

## Quantity Discounts

JLCCNC typical quantity breaks:

| Quantity | Discount |
|----------|----------|
| 1 | 0% |
| 2-4 | 5-10% |
| 5-9 | 10-15% |
| 10-24 | 15-20% |
| 25+ | 20-30% |

### Optimal Order Strategy

1. **Combine left/right parts**: Order both at once for quantity discount
2. **Batch similar parts**: Group by material and finish
3. **Order spares**: If ordering 1, consider ordering 2
4. **Combine with other projects**: Pool orders when possible

---

## Shipping Cost Estimates

### JLCCNC to USA

| Service | Est. Cost | Time |
|---------|-----------|------|
| Standard | $15-30 | 7-12 days |
| Express | $30-60 | 3-5 days |
| Priority | $50-100 | 2-3 days |

*Shipping cost varies by weight and destination*

### Weight Estimate by Part Size

| Part Size | Typical Weight (Al) |
|-----------|---------------------|
| Small | 50-150g |
| Medium | 150-400g |
| Large | 400-1000g |

---

## Full Arm Estimate Template

| Part | Qty | Unit | Finish | Est. Cost |
|------|-----|------|--------|-----------|
| Shoulder Base | 1 | $35 | Anod. | $35 |
| Shoulder Pitch Mount | 1 | $27 | Anod. | $27 |
| Shoulder Roll Mount | 1 | $27 | Anod. | $27 |
| Upper Arm Link | 1 | $40 | Anod. | $40 |
| Elbow Mount | 1 | $22 | Anod. | $22 |
| Forearm Link | 1 | $28 | Anod. | $28 |
| Wrist Roll Mount | 1 | $18 | Anod. | $18 |
| Wrist Pitch Mount | 1 | $15 | Anod. | $15 |
| **Parts Subtotal** | 8 | - | - | **$212** |
| Shipping (standard) | - | - | - | ~$25 |
| **Grand Total** | - | - | - | **~$237** |

*Estimates only - actual prices vary based on exact geometry*

---

## Cost Reduction Strategies

### High Impact

1. **Use standard tolerance** (±0.1mm) - saves 10-15%
2. **Skip anodizing** for prototypes - saves $3-8/part
3. **6061 instead of 7075** where possible - saves 30-50%

### Medium Impact

4. **As-machined finish** - saves $1-3/part
5. **Batch orders** for quantity discount - saves 5-20%
6. **Combine shipping** with other orders

### Low Impact

7. **Reduce complexity** - fewer features = lower cost
8. **Standard thread sizes** - already free
9. **Optimize part orientation** - reduces setup time

---

## Quick Reference

### Typical Per-Part Costs (with anodizing)

| Part Type | 6061 | 7075 |
|-----------|------|------|
| Small bracket | $8-15 | $12-20 |
| Motor mount | $20-35 | $28-50 |
| Arm link | $25-45 | $35-65 |
| Large housing | $50-100 | $70-140 |

### Cost Per Arm (CNC only)

| Scenario | Estimate |
|----------|----------|
| Budget (6061, std tol, minimal anod.) | $150-200 |
| Typical (6061, mixed tol, full anod.) | $200-300 |
| Premium (7075 critical, tight tol) | $300-450 |

---

*All prices are estimates based on typical JLCCNC pricing. Get actual quotes for accurate costs.*
