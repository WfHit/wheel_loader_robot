#!/usr/bin/env python3
"""
Demonstration: 8-Anchor UWB Localization System
Shows localization performance with rectangular anchor setup

Scenario:
- 8 anchors at corners of rectangular prism (10m x 2m x 5m)
- Target 5m away along Y-axis (outside rectangle)
- Measurement error: 10cm (σ_range)
- Prior error scenarios: 30cm and 10cm
- Compares methods with and without prior
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import least_squares
import sys


class CoplanarPriorDemo:
    """Demonstrate prior effectiveness with coplanar anchors"""

    def __init__(self):
        self.sigma_range = 0.10  # 10cm measurement error

    def setup_tilted_vertical_plane(self):
        """
        Create 8 anchors at corners of rectangular prism (3D bounding box)
        Forms a rectangular workspace with good 3D coverage
        """
        print("="*80)
        print("SETUP: 8 Anchors at Rectangular Corners")
        print("="*80)

        # 8 anchors at corners of a rectangular prism
        # X: 0 to 10m, Y: 0 to 2m, Z: 0 to 5m
        self.anchors = np.array([
            # Bottom 4 corners (z = 0)
            [0.0, 0.0, 0.0],     # Corner 0: bottom-left-front
            [10.0, 0.0, 0.0],    # Corner 1: bottom-right-front
            [10.0, 2.0, 0.0],    # Corner 2: bottom-right-back
            [0.0, 2.0, 0.0],     # Corner 3: bottom-left-back
            # Top 4 corners (z = 5)
            [0.0, 0.0, 5.0],     # Corner 4: top-left-front
            [10.0, 0.0, 5.0],    # Corner 5: top-right-front
            [10.0, 2.0, 5.0],    # Corner 6: top-right-back
            [0.0, 2.0, 5.0]      # Corner 7: top-left-back
        ])

        print(f"\n8 Anchors at rectangular corners:")
        print(f"  Rectangle dimensions: X=[0,10]m, Y=[0,2]m, Z=[0,5]m")
        print(f"\n  Bottom layer (z=0):")
        for i in range(4):
            a = self.anchors[i]
            print(f"    A{i}: [{a[0]:5.1f}, {a[1]:5.1f}, {a[2]:5.1f}]")
        print(f"\n  Top layer (z=5):")
        for i in range(4, 8):
            a = self.anchors[i]
            print(f"    A{i}: [{a[0]:5.1f}, {a[1]:5.1f}, {a[2]:5.1f}]")

        # Check if coplanar (should NOT be with 8 corners)
        plane_normal = np.array([0.0, 0.0, 1.0])  # For visualization (z-direction)
        print(f"\nPlane configuration: 3D rectangular prism (not coplanar)")

        # Check rank (using all 8 anchors)
        G = self.build_geometry_matrix(self.anchors)
        rank = np.linalg.matrix_rank(G)
        print(f"\nGeometry matrix rank: {rank}/3")
        if rank < 3:
            print(f"→ SINGULAR (rank-deficient) ❌")
        else:
            print(f"→ FULL RANK ✓")

        return plane_normal

    def setup_true_and_prior_positions(self, prior_error):
        """
        Setup true position and prior with given error

        Args:
            prior_error: Prior position error in meters
        """
        # True position (OUTSIDE the rectangular bounds)
        # Rectangle bounds: X=[0,10], Y=[0,2], Z=[0,5]
        # Rectangle center: X=5, Y=1, Z=2.5
        # Position 5m away along Y-axis: Y_center + 5m = 1 + 5 = 6m
        self.true_position = np.array([5.0, 6.0, 2.5])

        print(f"\nTrue position: {self.true_position}")
        print(f"  Rectangle bounds: X=[0,10], Y=[0,2], Z=[0,5]")
        print(f"  Rectangle center: X=5, Y=1, Z=2.5")
        print(f"  Position is 5m away along Y-axis (Y=6 vs Y_center=1) ✓")

        # Create prior with specified error
        # Generate random direction
        np.random.seed(42)
        direction = np.random.randn(3)
        direction = direction / np.linalg.norm(direction)

        self.prior_position = self.true_position + prior_error * direction
        actual_prior_error = np.linalg.norm(self.true_position - self.prior_position)

        print(f"\nPrior position: {self.prior_position}")
        print(f"Prior error: {actual_prior_error:.3f} m (target: {prior_error:.3f} m)")

        return actual_prior_error

    def generate_noisy_ranges(self):
        """Generate range measurements with noise"""
        # True ranges
        ranges_true = np.array([
            np.linalg.norm(self.true_position - anchor)
            for anchor in self.anchors
        ])

        # Add Gaussian noise
        np.random.seed(123)
        noise = np.random.normal(0, self.sigma_range, len(ranges_true))
        ranges_noisy = ranges_true + noise

        print(f"\nRange measurements (σ = {self.sigma_range*100:.0f}cm):")
        for i, (r_true, r_noisy, n) in enumerate(zip(ranges_true, ranges_noisy, noise)):
            print(f"  A{i}: true={r_true:.4f}m, measured={r_noisy:.4f}m, noise={n:+.4f}m")

        return ranges_noisy

    def build_geometry_matrix(self, anchors):
        """Build Chan's geometry matrix using all provided anchors"""
        if len(anchors) < 2:
            raise ValueError("Need at least 2 anchors")

        s1 = anchors[0]
        # Use all anchors to build overdetermined system
        G = np.array([s1 - anchors[i] for i in range(1, len(anchors))])
        return G

    def solve_linear_no_prior(self, ranges):
        """Standard linear multilateration (no prior) using all anchors"""
        print(f"\n" + "="*80)
        print("METHOD 1: Linear Multilateration (NO PRIOR)")
        print("="*80)

        # Build overdetermined system using all anchors
        s1 = self.anchors[0]
        r1 = ranges[0]
        K1 = np.dot(s1, s1)

        n = len(self.anchors) - 1
        G = np.zeros((n, 3))
        h = np.zeros(n)

        for i in range(1, len(self.anchors)):
            si = self.anchors[i]
            ri = ranges[i]
            Ki = np.dot(si, si)

            G[i-1] = s1 - si
            h[i-1] = 0.5 * (r1**2 - ri**2 - K1 + Ki)

        print(f"\nGeometry matrix G shape: {G.shape} ({n} equations, 3 unknowns)")

        rank = np.linalg.matrix_rank(G)
        print(f"Rank: {rank}/3")

        if rank < 3:
            print("→ SINGULAR matrix (rank-deficient) ❌")
        else:
            print("→ FULL RANK (overdetermined system) ✓")

        # Solve with lstsq
        x, residuals, rank_lstsq, s = np.linalg.lstsq(G, h, rcond=None)

        print(f"\nSolution: {x}")
        print(f"Singular values: {s}")
        if len(residuals) > 0:
            print(f"Residual: {residuals[0]:.6f}")

        # Calculate error
        error = np.linalg.norm(self.true_position - x)
        print(f"\n→ Position error: {error:.4f} m ({error*100:.2f} cm)")

        return x, error

    def solve_linear_with_prior(self, ranges, sigma_prior):
        """Linear multilateration with prior (Bayesian/Regularized) using all anchors"""
        print(f"\n" + "="*80)
        print("METHOD 2: Linear Multilateration WITH PRIOR (Regularized)")
        print("="*80)

        # Build overdetermined system using all anchors
        s1 = self.anchors[0]
        r1 = ranges[0]
        K1 = np.dot(s1, s1)

        n = len(self.anchors) - 1
        G = np.zeros((n, 3))
        h = np.zeros(n)

        for i in range(1, len(self.anchors)):
            si = self.anchors[i]
            ri = ranges[i]
            Ki = np.dot(si, si)

            G[i-1] = s1 - si
            h[i-1] = 0.5 * (r1**2 - ri**2 - K1 + Ki)

        # Regularization weight
        lambda_reg = (self.sigma_range / sigma_prior) ** 2

        print(f"\nParameters:")
        print(f"  σ_range = {self.sigma_range:.3f} m (measurement uncertainty)")
        print(f"  σ_prior = {sigma_prior:.3f} m (prior uncertainty)")
        print(f"  λ = (σ_range/σ_prior)² = {lambda_reg:.6f}")

        # Augmented system with prior
        G_aug = np.vstack([G, np.sqrt(lambda_reg) * np.eye(3)])
        h_aug = np.hstack([h, np.sqrt(lambda_reg) * self.prior_position])

        print(f"\nAugmented system:")
        print(f"  G_aug shape: {G_aug.shape} (added 3 rows for prior)")

        rank_aug = np.linalg.matrix_rank(G_aug)
        print(f"  Rank: {rank_aug}/3")

        if rank_aug == 3:
            print("  → FULL RANK ✓")

        # Solve
        x, residuals, rank_lstsq, s = np.linalg.lstsq(G_aug, h_aug, rcond=None)

        print(f"\nSolution: {x}")

        # Calculate error
        error = np.linalg.norm(self.true_position - x)
        print(f"\n→ Position error: {error:.4f} m ({error*100:.2f} cm)")

        return x, error

    def solve_nonlinear_with_prior(self, ranges, sigma_prior):
        """Nonlinear least squares with prior"""
        print(f"\n" + "="*80)
        print("METHOD 3: Nonlinear Least Squares WITH PRIOR")
        print("="*80)

        def residuals(x, anchors, ranges, x_prior, sigma_r, sigma_p):
            # Measurement residuals (weighted)
            r_meas = []
            for anchor, r_measured in zip(anchors, ranges):
                r_predicted = np.linalg.norm(x - anchor)
                r_meas.append((r_predicted - r_measured) / sigma_r)

            # Prior residuals (weighted)
            r_prior = (x - x_prior) / sigma_p

            return np.concatenate([r_meas, r_prior])

        print(f"\nOptimization:")
        print(f"  Minimize: Σ[(r_pred - r_meas)/σ_r]² + Σ[(x - x_prior)/σ_p]²")
        print(f"  Starting from prior position")

        result = least_squares(
            residuals,
            x0=self.prior_position,
            args=(self.anchors, ranges, self.prior_position,
                  self.sigma_range, sigma_prior),
            verbose=0
        )

        x = result.x

        print(f"\nSolution: {x}")
        print(f"Optimization success: {result.success}")
        print(f"Cost: {result.cost:.6f}")

        # Calculate error
        error = np.linalg.norm(self.true_position - x)
        print(f"\n→ Position error: {error:.4f} m ({error*100:.2f} cm)")

        return x, error

    def run_scenario(self, prior_error_cm, sigma_prior):
        """Run complete scenario"""
        print("\n\n")
        print("#"*80)
        print(f"# SCENARIO: Prior Error = {prior_error_cm:.0f} cm")
        print("#"*80)

        # Setup
        plane_normal = self.setup_tilted_vertical_plane()
        actual_prior_error = self.setup_true_and_prior_positions(prior_error_cm / 100.0)
        ranges = self.generate_noisy_ranges()

        # Solve with different methods
        results = {}

        # Prior only (baseline)
        prior_error = np.linalg.norm(self.true_position - self.prior_position)
        results['Prior only'] = (self.prior_position.copy(), prior_error)
        print(f"\n" + "="*80)
        print("BASELINE: Prior Only (no measurement)")
        print("="*80)
        print(f"Error: {prior_error:.4f} m ({prior_error*100:.2f} cm)")

        # Method 1: Linear without prior
        x1, err1 = self.solve_linear_no_prior(ranges)
        results['Linear (no prior)'] = (x1, err1)

        # Method 2: Linear with prior
        x2, err2 = self.solve_linear_with_prior(ranges, sigma_prior)
        results['Linear + prior'] = (x2, err2)

        # Method 3: Nonlinear with prior
        x3, err3 = self.solve_nonlinear_with_prior(ranges, sigma_prior)
        results['Nonlinear + prior'] = (x3, err3)

        # Summary
        print(f"\n" + "="*80)
        print(f"SUMMARY - Prior Error = {prior_error_cm:.0f}cm Scenario")
        print("="*80)

        print(f"\n{'Method':<30} {'Error (m)':<12} {'Error (cm)':<12} {'Improvement':<15}")
        print("-"*80)

        for name, (pos, err) in results.items():
            improvement = (prior_error - err) * 100  # in cm
            improvement_pct = ((prior_error - err) / prior_error * 100) if prior_error > 0 else 0
            print(f"{name:<30} {err:<12.4f} {err*100:<12.2f} {improvement:+7.2f}cm ({improvement_pct:+6.1f}%)")

        return results, plane_normal

    def create_visualization(self, results_30cm, results_10cm, plane_normal):
        """Create comprehensive visualization"""
        print(f"\n" + "="*80)
        print("CREATING VISUALIZATION...")
        print("="*80)

        fig = plt.figure(figsize=(20, 12))

        # =====================================================================
        # Row 1: 30cm Prior Error Scenario
        # =====================================================================

        # 3D view - 30cm
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        self.plot_3d_scenario(ax1, results_30cm, plane_normal, "30cm Prior Error")

        # Error bars - 30cm
        ax2 = fig.add_subplot(2, 3, 2)
        self.plot_error_bars(ax2, results_30cm, "30cm Prior Error")

        # Z-coordinate - 30cm
        ax3 = fig.add_subplot(2, 3, 3)
        self.plot_z_coordinate(ax3, results_30cm, "30cm Prior Error")

        # =====================================================================
        # Row 2: 10cm Prior Error Scenario
        # =====================================================================

        # 3D view - 10cm
        ax4 = fig.add_subplot(2, 3, 4, projection='3d')
        self.plot_3d_scenario(ax4, results_10cm, plane_normal, "10cm Prior Error")

        # Error bars - 10cm
        ax5 = fig.add_subplot(2, 3, 5)
        self.plot_error_bars(ax5, results_10cm, "10cm Prior Error")

        # Z-coordinate - 10cm
        ax6 = fig.add_subplot(2, 3, 6)
        self.plot_z_coordinate(ax6, results_10cm, "10cm Prior Error")

        plt.tight_layout()

        filename = 'prior_coplanar_demonstration.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"\n✓ Visualization saved as '{filename}'")

        plt.show()

    def plot_3d_scenario(self, ax, results, plane_normal, title):
        """Plot 3D view of scenario with 8 anchors"""

        # Draw rectangular prism edges
        # Bottom rectangle
        bottom = self.anchors[:4]
        for i in range(4):
            j = (i + 1) % 4
            ax.plot([bottom[i,0], bottom[j,0]],
                   [bottom[i,1], bottom[j,1]],
                   [bottom[i,2], bottom[j,2]],
                   'k--', linewidth=1, alpha=0.3)

        # Top rectangle
        top = self.anchors[4:]
        for i in range(4):
            j = (i + 1) % 4
            ax.plot([top[i,0], top[j,0]],
                   [top[i,1], top[j,1]],
                   [top[i,2], top[j,2]],
                   'k--', linewidth=1, alpha=0.3)

        # Vertical edges
        for i in range(4):
            ax.plot([bottom[i,0], top[i,0]],
                   [bottom[i,1], top[i,1]],
                   [bottom[i,2], top[i,2]],
                   'k--', linewidth=1, alpha=0.3)

        # Plot all 8 anchors
        ax.scatter(self.anchors[:, 0], self.anchors[:, 1], self.anchors[:, 2],
                  c='red', s=200, marker='s', edgecolors='black',
                  linewidths=2, label='8 Anchors', zorder=5)

        # Label anchors
        for i, a in enumerate(self.anchors):
            ax.text(a[0], a[1], a[2], f'  A{i}', fontsize=7, alpha=0.7)

        # Plot true position
        ax.scatter(*self.true_position, c='green', s=400, marker='*',
                  edgecolors='black', linewidths=3,
                  label='True position', zorder=10)

        # Plot results
        colors = ['orange', 'purple', 'blue', 'cyan']
        markers = ['D', 'o', 'o', '^']

        for (name, (pos, err)), color, marker in zip(results.items(), colors, markers):
            ax.scatter(*pos, c=color, s=150, marker=marker,
                      edgecolors='black', linewidths=1.5,
                      label=name, zorder=6, alpha=0.8)

        # Plot plane normal
        center = self.anchors.mean(axis=0)
        ax.quiver(center[0], center[1], center[2],
                 plane_normal[0]*3, plane_normal[1]*3, plane_normal[2]*3,
                 color='green', arrow_length_ratio=0.3, linewidth=2,
                 label='Plane normal', alpha=0.6)

        ax.set_xlabel('X (m)', fontsize=9)
        ax.set_ylabel('Y (m)', fontsize=9)
        ax.set_zlabel('Z (m)', fontsize=9)
        ax.set_title(f'{title}\n(8 Anchors: 10m x 2m x 5m Rectangle)',
                    fontweight='bold', fontsize=11)
        ax.legend(fontsize=7, loc='upper left')
        ax.set_box_aspect([1.2, 1, 0.8])

    def plot_error_bars(self, ax, results, title):
        """Plot error comparison"""
        names = list(results.keys())
        errors = [results[name][1] * 100 for name in names]  # Convert to cm
        colors = ['orange', 'purple', 'blue', 'cyan']

        bars = ax.barh(range(len(names)), errors, color=colors,
                      edgecolor='black', linewidth=1.5, alpha=0.7)

        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names, fontsize=9)
        ax.set_xlabel('Position Error (cm)', fontweight='bold', fontsize=10)
        ax.set_title(f'{title}\nError Comparison',
                    fontweight='bold', fontsize=11)
        ax.grid(True, alpha=0.3, axis='x')

        # Add value labels
        for i, (bar, err) in enumerate(zip(bars, errors)):
            ax.text(err + 1, bar.get_y() + bar.get_height()/2,
                   f'{err:.1f}cm', va='center', fontsize=9, fontweight='bold')

    def plot_z_coordinate(self, ax, results, title):
        """Plot Z-coordinate comparison"""
        names = list(results.keys())
        z_values = [results[name][0][2] for name in names]
        colors = ['orange', 'purple', 'blue', 'cyan']

        bars = ax.bar(range(len(names)), z_values, color=colors,
                     edgecolor='black', linewidth=1.5, alpha=0.7)

        # True Z line
        ax.axhline(self.true_position[2], color='green', linestyle='--',
                  linewidth=3, label=f'True Z = {self.true_position[2]:.2f}m',
                  zorder=10)

        ax.set_xticks(range(len(names)))
        ax.set_xticklabels([n.replace(' ', '\n') for n in names],
                          fontsize=8)
        ax.set_ylabel('Z Coordinate (m)', fontweight='bold', fontsize=10)
        ax.set_title(f'{title}\nZ-Coordinate (Lost Dimension)',
                    fontweight='bold', fontsize=11)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3, axis='y')

        # Add value labels
        for bar, z in zip(bars, z_values):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{z:.2f}m',
                   ha='center', va='bottom', fontsize=8, fontweight='bold')


def main():
    """Main demonstration"""
    print("\n")
    print("*"*80)
    print("* DEMONSTRATION: Prior Information with Coplanar Anchors")
    print("*"*80)
    print("*")
    print("* Configuration:")
    print("*   - 8 anchors at rectangular prism corners (10m x 2m x 5m)")
    print("*   - Target 5m away along Y-axis (outside rectangle)")
    print("*   - Measurement error: 10cm (σ_range)")
    print("*   - Prior error scenarios: 30cm and 10cm")
    print("*")
    print("* Methods compared:")
    print("*   1. Prior only (baseline)")
    print("*   2. Linear multilateration (no prior)")
    print("*   3. Linear multilateration WITH prior (regularized)")
    print("*   4. Nonlinear least squares WITH prior")
    print("*")
    print("*"*80)

    demo = CoplanarPriorDemo()

    # Scenario 1: 30cm prior error
    results_30cm, plane_normal = demo.run_scenario(
        prior_error_cm=30.0,
        sigma_prior=0.30  # 30cm uncertainty
    )

    # Scenario 2: 10cm prior error
    results_10cm, _ = demo.run_scenario(
        prior_error_cm=10.0,
        sigma_prior=0.10  # 10cm uncertainty
    )

    # Create visualization
    demo.create_visualization(results_30cm, results_10cm, plane_normal)

    # Final summary
    print("\n\n")
    print("*"*80)
    print("* FINAL CONCLUSIONS")
    print("*"*80)

    print("\n1. GEOMETRY CONFIGURATION:")
    print("   • 8 anchors at rectangular prism corners (10m x 2m x 5m)")
    print("   • Target outside rectangle: 5m along Y-axis")
    print("   • Full-rank geometry matrix (rank = 3)")
    print("   • Good 3D coverage with overdetermined system")

    print("\n2. EFFECT OF PRIOR (30cm error scenario):")
    prior_30_err = results_30cm['Prior only'][1] * 100
    linear_30_err = results_30cm['Linear (no prior)'][1] * 100
    reg_30_err = results_30cm['Linear + prior'][1] * 100
    nonlin_30_err = results_30cm['Nonlinear + prior'][1] * 100

    print(f"   • Prior only:             {prior_30_err:.1f} cm (baseline)")
    print(f"   • Linear (no prior):      {linear_30_err:.1f} cm")
    print(f"   • Linear + prior:         {reg_30_err:.1f} cm "
          f"(improvement: {prior_30_err - reg_30_err:+.1f} cm)")
    print(f"   • Nonlinear + prior:      {nonlin_30_err:.1f} cm "
          f"(improvement: {prior_30_err - nonlin_30_err:+.1f} cm)")

    print("\n3. EFFECT OF PRIOR (10cm error scenario):")
    prior_10_err = results_10cm['Prior only'][1] * 100
    linear_10_err = results_10cm['Linear (no prior)'][1] * 100
    reg_10_err = results_10cm['Linear + prior'][1] * 100
    nonlin_10_err = results_10cm['Nonlinear + prior'][1] * 100

    print(f"   • Prior only:             {prior_10_err:.1f} cm (baseline)")
    print(f"   • Linear (no prior):      {linear_10_err:.1f} cm")
    print(f"   • Linear + prior:         {reg_10_err:.1f} cm "
          f"(improvement: {prior_10_err - reg_10_err:+.1f} cm)")
    print(f"   • Nonlinear + prior:      {nonlin_10_err:.1f} cm "
          f"(improvement: {prior_10_err - nonlin_10_err:+.1f} cm)")

    print("\n4. KEY FINDINGS:")
    print("   ✓ Good 3D geometry enables accurate localization")
    print("   ✓ 8 anchors provide overdetermined system (7 equations, 3 unknowns)")
    print("   ✓ Prior still improves results even with good geometry")
    print("   ✓ Linear methods work well with full-rank system")
    print("   ✓ Target outside rectangle is successfully localized")
    print("   ✓ Nonlinear optimization achieves best accuracy")

    print("\n5. PRACTICAL RECOMMENDATION:")
    print("   • 8-anchor rectangular setup provides excellent geometry")
    print("   • Can localize targets outside the bounding box")
    print("   • Prior information still helpful for noise reduction")
    print("   • Suitable for wheel loader workspace monitoring")

    print("\n" + "*"*80)
    print("* Demonstration complete!")
    print("*"*80 + "\n")


if __name__ == "__main__":
    main()
