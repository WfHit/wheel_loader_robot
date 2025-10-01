# Lambda (λ) Parameter Tuning Guide

## Understanding λ in Regularized Least Squares

The regularization parameter **λ** controls the balance between trusting measurements vs. trusting prior information:

```
λ = (σ_measurement / σ_prior)²
```

### How it Works:

- **Small λ** (e.g., 0.0278): Trust measurements MORE, trust prior LESS
- **Large λ** (e.g., 1.0000): Trust measurements and prior EQUALLY
- **Very large λ**: Trust prior MORE, trust measurements LESS

## Configuration Comparison

### Original Configuration (Equal Trust)

**Scenario 1: 30cm prior error**
- σ_prior = 0.30m (same as actual error)
- λ = (0.1/0.3)² = **0.1111**
- **Result**: Nonlinear error = **16.06 cm** ✓

**Scenario 2: 10cm prior error**
- σ_prior = 0.10m (same as actual error)
- λ = (0.1/0.1)² = **1.0000** (equal trust)
- **Result**: Nonlinear error = **10.09 cm** ✓

### Adjusted Configuration (Trust Measurements More)

**Scenario 1: 30cm prior error**
- σ_prior = 0.60m (2x larger → less trust in prior)
- λ = (0.1/0.6)² = **0.0278** (trust measurements 36x more)
- **Result**: Nonlinear error = **19.12 cm** ✓

**Scenario 2: 10cm prior error**
- σ_prior = 0.30m (3x larger → less trust in prior)
- λ = (0.1/0.3)² = **0.1111** (trust measurements 9x more)
- **Result**: Nonlinear error = **15.68 cm**

## Analysis

### When to Trust Measurements More (Small λ):

✓ **Good geometry**: Non-coplanar anchors with good GDOP
✓ **Accurate measurements**: High-quality UWB with good SNR
✓ **Uncertain prior**: Prior from rough motion model or old estimate
✓ **Static target**: Position hasn't changed much

### When to Trust Prior More (Large λ):

✓ **Poor geometry**: Coplanar anchors, high GDOP
✓ **Noisy measurements**: Multipath, interference, low SNR
✓ **Accurate prior**: Recent EKF estimate, IMU integration
✓ **Fast motion**: Prior from high-rate IMU is more reliable

## Practical Results

### Comparison Table

| Configuration | Scenario | λ value | Nonlinear Error | Improvement over Prior |
|---------------|----------|---------|-----------------|------------------------|
| **Original (Balanced)** |
| | 30cm prior | 0.1111 | 16.06 cm | +46.5% better |
| | 10cm prior | 1.0000 | 10.09 cm | -0.9% (maintained) |
| **Adjusted (Trust Measurements)** |
| | 30cm prior | 0.0278 | 19.12 cm | +36.3% better |
| | 10cm prior | 0.1111 | 15.68 cm | -56.8% worse |

### Key Observations:

1. **With poor prior (30cm error)**:
   - Original (λ=0.1111): 16.06cm - balanced approach works well
   - Adjusted (λ=0.0278): 19.12cm - trusting measurements more gives slightly worse result
   - **Conclusion**: With coplanar anchors, some prior trust is needed

2. **With good prior (10cm error)**:
   - Original (λ=1.0000): 10.09cm - maintains prior accuracy
   - Adjusted (λ=0.1111): 15.68cm - trusting measurements hurts accuracy
   - **Conclusion**: With coplanar geometry, measurements alone are unreliable

## Recommendations for Wheel Loader Robot

### Conservative Approach (Recommended for Coplanar Setup):
```python
# Set sigma_prior equal to actual estimated prior uncertainty
if prior_from_ekf:
    sigma_prior = ekf_covariance_trace  # Use EKF uncertainty
elif prior_from_imu:
    sigma_prior = 0.20  # ~20cm IMU drift uncertainty
else:
    sigma_prior = 0.50  # Conservative for rough motion model
```

### Aggressive Approach (Only with Good Geometry):
```python
# Trust measurements more (only if non-coplanar anchors!)
if geometry_rank == 3 and gdop < 5.0:
    sigma_prior = 2.0 * actual_prior_error  # Inflate uncertainty
    # This gives smaller λ, trusting measurements more
```

### Adaptive Approach (Best):
```python
# Adjust based on measurement quality and geometry
if gdop < 3.0 and snr_avg > 20:  # Good geometry and signal
    sigma_prior = 2.0 * prior_uncertainty  # Trust measurements
elif gdop > 10.0 or snr_avg < 10:  # Poor geometry or signal
    sigma_prior = 0.5 * prior_uncertainty  # Trust prior
else:
    sigma_prior = prior_uncertainty  # Balanced
```

## Mathematical Insight

The cost function being minimized is:

```
J(x) = Σᵢ [(rᵢ_pred - rᵢ_meas)/σ_range]² + λ·||x - x_prior||²/σ_prior²
     = measurement_cost + prior_cost
```

Where:
- λ = (σ_range/σ_prior)²
- Small λ → measurement_cost dominates
- Large λ → prior_cost dominates

For coplanar anchors:
- Measurements constrain position in 2D (on plane)
- Prior constrains position in 3D (especially perpendicular to plane)
- **Need both** for unique solution!

## Conclusion

With **coplanar anchors** (rank-deficient geometry):
- Cannot rely on measurements alone → will fail catastrophically
- Cannot rely on prior alone → won't improve accuracy
- **Must balance both** using appropriate λ

**Current adjustment** (trusting measurements more):
- Works okay but shows worse performance in coplanar case
- Better to use balanced or conservative approach
- **Original configuration was actually optimal!**

For best results with coplanar anchors:
- Set σ_prior = actual prior uncertainty (honest assessment)
- Let λ naturally balance measurement vs prior
- Don't artificially inflate σ_prior unless geometry is truly excellent

---

**Bottom Line**: The original λ values were well-tuned for coplanar geometry. The adjusted values show that trusting measurements too much hurts performance when geometry is poor.
