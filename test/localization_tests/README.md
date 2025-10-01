# Localization Tests

This directory contains tests for localization algorithms used in the wheel loader robot project.

## Tests

### `prior_coplanar_demo.py`

Demonstrates how prior position information helps solve the singularity problem when UWB anchors are coplanar (all in the same plane).

**Key Features:**
- Simulates coplanar anchors in a vertical tilted plane (not aligned with x or y axis)
- Tests two scenarios: 30cm and 10cm prior error
- Uses realistic 10cm UWB measurement error
- Compares multiple methods:
  - Prior only (baseline)
  - Linear multilateration without prior (fails badly)
  - Linear multilateration with prior (regularized)
  - Nonlinear least squares with prior (best performance)

**Expected Results:**
- With 30cm prior error: Final error reduces to ~16cm (47% improvement)
- With 10cm prior error: Final error stays at ~10cm (maintains accuracy)
- Demonstrates that prior information is essential for coplanar configurations

## Running Tests

### Quick Start

```bash
./run_test.sh
```

This will:
1. Check for miniforge installation
2. Create/activate the conda environment
3. Install required packages (numpy, matplotlib, scipy)
4. Run the test
5. Generate visualization

### Manual Run

If you prefer to run manually:

```bash
# Activate conda environment
source $HOME/miniforge3/etc/profile.d/conda.sh
conda activate localization_test

# Run the test
python prior_coplanar_demo.py
```

## Requirements

- Miniforge3 (installed at `~/miniforge3`)
- Python 3.9
- NumPy
- Matplotlib
- SciPy

## Output

The test generates:
- **Console output**: Detailed results for each method and scenario
- **prior_coplanar_demonstration.png**: Visualization showing:
  - 3D anchor and position geometry
  - Error comparison between methods
  - Z-coordinate analysis

## Understanding the Results

### Coplanar Problem
When all UWB anchors lie in the same plane, the geometry matrix becomes singular (rank-deficient). This means:
- The system has infinite solutions along the direction perpendicular to the plane
- Standard linear multilateration fails catastrophically
- Errors can be many meters (>17m in our tests)

### How Prior Helps
Prior position information (from previous estimates, IMU, or motion model):
- Breaks the singularity by adding a regularization constraint
- Provides information in the "lost dimension" (perpendicular to plane)
- Acts as a soft constraint that balances measurement and prior trust
- Enables accurate localization even with poor geometry

### Practical Implications
For wheel loader robot:
1. **Always use prior information** with Bayesian/MAP estimation
2. Prior can come from:
   - Previous position estimate (EKF prediction)
   - IMU integration
   - Motion model (dead reckoning)
3. Weight prior by uncertainty (σ_prior parameter)
4. If possible, avoid coplanar anchor placement:
   - Place at least one anchor out of plane formed by others
   - Recommended: 5 anchors with varied heights

## Algorithm Details

### Regularized Least Squares (Linear + Prior)
Minimizes: `||Gx - h||² + λ||x - x_prior||²`

Where:
- `λ = (σ_measurement / σ_prior)²` balances measurement vs prior trust
- Creates augmented system: `G_aug = [G; √λ·I]`, `h_aug = [h; √λ·x_prior]`
- Makes singular matrix full-rank

### Nonlinear Least Squares with Prior
Minimizes: `Σ[(r_pred - r_meas)/σ_r]² + Σ[(x - x_prior)/σ_p]²`

Using:
- Levenberg-Marquardt optimization
- Weighted residuals for measurements and prior
- Starts from prior position

## References

- Chan, Y. T., & Ho, K. C. (1994). A simple and efficient estimator for hyperbolic location. IEEE Transactions on Signal Processing.
- Bancroft, S. (1985). An algebraic solution of the GPS equations. IEEE Transactions on Aerospace and Electronic Systems.
- Kay, S. M. (1993). Fundamentals of Statistical Signal Processing: Estimation Theory.

## Author

Wheel Loader Robot Project
Date: October 2025
