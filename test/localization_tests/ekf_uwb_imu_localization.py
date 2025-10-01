#!/usr/bin/env python3
"""
Redesigned Extended Kalman Filter for UWB+IMU Sensor Fusion
============================================================

Modular architecture with clear separation of concerns:
- Configuration management
- Sensor models (IMU, UWB)
- State estimation (EKF)
- Trajectory generation
- Baseline algorithms
- Simulation orchestration
- Visualization

Author: Redesigned implementation
Date: October 2025
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional, Dict, Any, List
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


# ============================================================================
# CONFIGURATION MODULE
# ============================================================================

@dataclass
class IMUConfig:
    """IMU sensor configuration (ADIS16470)"""
    rate: float = 100.0  # Hz
    gyro_bias_stability: float = 4.0  # deg/hr
    gyro_arw: float = 0.12  # deg/sqrt(hr)
    accel_bias_stability: float = 0.016  # mg (milli-g)
    accel_vrw: float = 0.029  # m/s/sqrt(hr)

    @property
    def dt(self) -> float:
        return 1.0 / self.rate

    @property
    def gyro_bias_rad(self) -> float:
        """Convert gyro bias from deg/hr to rad/s"""
        return self.gyro_bias_stability * np.pi / 180.0 / 3600.0

    @property
    def gyro_arw_rad(self) -> float:
        """Convert gyro ARW from deg/sqrt(hr) to rad/sqrt(s)"""
        return self.gyro_arw * np.pi / 180.0 / 60.0

    @property
    def accel_bias_mps2(self) -> float:
        """Convert accel bias from mg to m/s²"""
        return self.accel_bias_stability * 1e-3 * 9.81

    @property
    def accel_vrw_mps(self) -> float:
        """Convert accel VRW from m/s/sqrt(hr) to m/s/sqrt(s)"""
        return self.accel_vrw / 60.0


@dataclass
class UWBConfig:
    """UWB sensor configuration"""
    rate: float = 10.0  # Hz
    range_noise_std: float = 0.10  # meters
    max_range: float = 250.0  # meters
    min_range: float = 0.5  # meters

    @property
    def dt(self) -> float:
        return 1.0 / self.rate


@dataclass
class AnchorConfig:
    """UWB anchor array configuration"""
    rows: int = 2
    anchors_per_row: int = 21
    x_start: float = 0.0
    x_end: float = 200.0
    y_positions: Tuple[float, ...] = (0.0, 10.0)
    z_positions: Tuple[float, ...] = (6.0, 6.0)

    @property
    def total_anchors(self) -> int:
        return self.rows * self.anchors_per_row

    @property
    def x_spacing(self) -> float:
        return (self.x_end - self.x_start) / (self.anchors_per_row - 1)

    @property
    def vertical_baseline(self) -> float:
        return max(self.z_positions) - min(self.z_positions)

    @property
    def horizontal_baseline(self) -> float:
        return max(self.y_positions) - min(self.y_positions)

    def generate_positions(self) -> np.ndarray:
        """Generate anchor position array (42 anchors: 21 at Y=0m, 21 at Y=10m)"""
        positions = []
        x_coords = np.linspace(self.x_start, self.x_end, self.anchors_per_row)

        for i, (y, z) in enumerate(zip(self.y_positions, self.z_positions)):
            for x in x_coords:
                positions.append([x, y, z])

        return np.array(positions)


@dataclass
class FilterConfig:
    """EKF tuning parameters"""
    # Initial uncertainty
    pos_init_std: float = 1.0  # meters
    vel_init_std: float = 1.0  # m/s
    ori_init_std: float = 0.1  # radians

    # Process noise (per second)
    pos_process_std: float = 0.01  # meters/sqrt(s)
    vel_process_std: float = 0.1   # m/s/sqrt(s)
    ori_process_std: float = 0.001 # rad/sqrt(s)

    # Innovation gating
    use_gating: bool = True
    gate_threshold: float = 3.0  # sigma
    min_inliers: int = 4

    # Outlier rejection
    outlier_threshold: float = 0.5  # meters


@dataclass
class SimulationConfig:
    """Simulation parameters"""
    duration: float = 20.0  # seconds
    trajectory_velocity: float = 10.0  # m/s
    tag_position_y: float = 3.0  # meters from anchors
    tag_position_z: float = 5.0  # meters height (below anchors at Z=6m)
    y_amplitude: float = 0.5  # meters (sinusoidal motion)
    z_amplitude: float = 0.3  # meters (sinusoidal motion)
    random_seed: int = 42


@dataclass
class SystemConfig:
    """Complete system configuration"""
    imu: IMUConfig = field(default_factory=IMUConfig)
    uwb: UWBConfig = field(default_factory=UWBConfig)
    anchors: AnchorConfig = field(default_factory=AnchorConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)


# ============================================================================
# SENSOR MODELS
# ============================================================================

class IMUSensor:
    """ADIS16470 IMU sensor model with realistic noise"""

    def __init__(self, config: IMUConfig):
        self.config = config
        self.reset()

    def reset(self):
        """Reset biases to random initial values"""
        self.gyro_bias = np.random.normal(0, self.config.gyro_bias_rad, 3)
        self.accel_bias = np.random.normal(0, self.config.accel_bias_mps2, 3)

    def measure(self, angular_vel: np.ndarray, acceleration: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate IMU measurements with noise

        Args:
            angular_vel: True angular velocity (rad/s) in body frame
            acceleration: True specific force (m/s²) in body frame

        Returns:
            (gyro_measurement, accel_measurement)
        """
        # Add noise and bias
        gyro_noise = np.random.normal(0, self.config.gyro_arw_rad, 3)
        gyro_meas = angular_vel + self.gyro_bias + gyro_noise

        accel_noise = np.random.normal(0, self.config.accel_vrw_mps, 3)
        accel_meas = acceleration + self.accel_bias + accel_noise

        # Evolve biases (random walk)
        bias_evolution = 0.01 * self.config.dt
        self.gyro_bias += np.random.normal(0, self.config.gyro_bias_rad * bias_evolution, 3)
        self.accel_bias += np.random.normal(0, self.config.accel_bias_mps2 * bias_evolution, 3)

        return gyro_meas, accel_meas


class UWBSensor:
    """UWB range sensor model"""

    def __init__(self, config: UWBConfig, anchor_positions: np.ndarray):
        self.config = config
        self.anchors = anchor_positions
        self.n_anchors = len(anchor_positions)

    def measure(self, position: np.ndarray) -> np.ndarray:
        """
        Generate UWB range measurements with noise

        Args:
            position: Tag position [x, y, z]

        Returns:
            Noisy range measurements to all anchors
        """
        # Compute true ranges
        ranges = np.linalg.norm(self.anchors - position, axis=1)

        # Add Gaussian noise
        noise = np.random.normal(0, self.config.range_noise_std, self.n_anchors)
        noisy_ranges = ranges + noise

        # Apply physical constraints
        noisy_ranges = np.clip(noisy_ranges, self.config.min_range, self.config.max_range)

        return noisy_ranges


# ============================================================================
# STATE ESTIMATION
# ============================================================================

class StateVector:
    """Encapsulates the 9D state vector with named access"""

    def __init__(self, x: Optional[np.ndarray] = None):
        self._x = np.zeros(9) if x is None else x.copy()

    @property
    def position(self) -> np.ndarray:
        return self._x[:3]

    @position.setter
    def position(self, value: np.ndarray):
        self._x[:3] = value

    @property
    def velocity(self) -> np.ndarray:
        return self._x[3:6]

    @velocity.setter
    def velocity(self, value: np.ndarray):
        self._x[3:6] = value

    @property
    def orientation(self) -> np.ndarray:
        return self._x[6:9]

    @orientation.setter
    def orientation(self, value: np.ndarray):
        self._x[6:9] = self._normalize_angles(value)

    @staticmethod
    def _normalize_angles(angles: np.ndarray) -> np.ndarray:
        """Normalize angles to [-pi, pi]"""
        return np.arctan2(np.sin(angles), np.cos(angles))

    @property
    def vector(self) -> np.ndarray:
        return self._x.copy()

    def copy(self) -> 'StateVector':
        return StateVector(self._x)


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for UWB+IMU sensor fusion
    Clean, modular implementation
    """

    def __init__(self, config: SystemConfig):
        self.config = config
        self.state = StateVector()
        self.P = self._initialize_covariance()
        self.Q = self._compute_process_noise()

        # Sensor models
        anchor_positions = config.anchors.generate_positions()
        self.imu_sensor = IMUSensor(config.imu)
        self.uwb_sensor = UWBSensor(config.uwb, anchor_positions)

        # Statistics
        self.prediction_count = 0
        self.update_count = 0
        self.outliers_rejected = 0

    def _initialize_covariance(self) -> np.ndarray:
        """Initialize state covariance matrix P"""
        cfg = self.config.filter
        return np.diag([
            cfg.pos_init_std**2, cfg.pos_init_std**2, cfg.pos_init_std**2,
            cfg.vel_init_std**2, cfg.vel_init_std**2, cfg.vel_init_std**2,
            cfg.ori_init_std**2, cfg.ori_init_std**2, cfg.ori_init_std**2
        ])

    def _compute_process_noise(self) -> np.ndarray:
        """Compute process noise matrix Q"""
        cfg = self.config.filter
        return np.diag([
            cfg.pos_process_std**2, cfg.pos_process_std**2, cfg.pos_process_std**2,
            cfg.vel_process_std**2, cfg.vel_process_std**2, cfg.vel_process_std**2,
            cfg.ori_process_std**2, cfg.ori_process_std**2, cfg.ori_process_std**2
        ])

    def initialize(self, position: np.ndarray,
                   velocity: Optional[np.ndarray] = None,
                   orientation: Optional[np.ndarray] = None):
        """Initialize filter state"""
        self.state.position = position
        self.state.velocity = velocity if velocity is not None else np.zeros(3)
        self.state.orientation = orientation if orientation is not None else np.zeros(3)

    def predict_imu(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """
        IMU-based prediction step

        Args:
            gyro: Gyroscope measurement (rad/s) in body frame
            accel: Accelerometer measurement (m/s²) in body frame
            dt: Time step
        """
        # Extract current state
        pos = self.state.position
        vel = self.state.velocity
        euler = self.state.orientation

        # Transform acceleration to world frame
        R_body_to_world = Rotation.from_euler('xyz', euler).as_matrix()
        accel_world = R_body_to_world @ accel

        # Remove gravity (Z-up convention)
        accel_world[2] -= 9.81

        # Kinematic predictions
        pos_new = pos + vel * dt + 0.5 * accel_world * dt**2
        vel_new = vel + accel_world * dt
        euler_new = euler + gyro * dt

        # Update state
        self.state.position = pos_new
        self.state.velocity = vel_new
        self.state.orientation = euler_new

        # Propagate covariance
        F = self._compute_jacobian(dt)
        self.P = F @ self.P @ F.T + self.Q * dt

        self.prediction_count += 1

    def _compute_jacobian(self, dt: float) -> np.ndarray:
        """Compute state transition Jacobian F"""
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt  # Position depends on velocity
        return F

    def update_uwb(self, ranges: np.ndarray):
        """
        UWB measurement update step with outlier rejection

        Args:
            ranges: Measured ranges to anchors
        """
        # Innovation gating
        if self.config.filter.use_gating:
            inlier_mask = self._apply_innovation_gating(ranges)
            if np.sum(inlier_mask) < self.config.filter.min_inliers:
                return  # Skip update

            ranges = ranges[inlier_mask]
            anchors = self.uwb_sensor.anchors[inlier_mask]
        else:
            anchors = self.uwb_sensor.anchors

        # Compute innovation
        h_pred = self._measurement_function(self.state.vector, anchors)
        y = ranges - h_pred

        # Compute Kalman gain
        H = self._measurement_jacobian(self.state.vector, anchors)
        R = np.eye(len(ranges)) * self.config.uwb.range_noise_std**2

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        state_update = K @ y
        self.state._x += state_update
        self.state.orientation = self.state.orientation  # Normalize

        # Covariance update (Joseph form)
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self.update_count += 1

    def _measurement_function(self, state: np.ndarray, anchors: np.ndarray) -> np.ndarray:
        """Predict range measurements from state"""
        position = state[:3]
        return np.linalg.norm(anchors - position, axis=1)

    def _measurement_jacobian(self, state: np.ndarray, anchors: np.ndarray) -> np.ndarray:
        """Compute measurement Jacobian H"""
        position = state[:3]
        n_anchors = len(anchors)

        H = np.zeros((n_anchors, 9))
        for i, anchor in enumerate(anchors):
            delta = position - anchor
            r = np.linalg.norm(delta)
            if r > 1e-6:
                H[i, :3] = delta / r

        return H

    def _apply_innovation_gating(self, ranges: np.ndarray) -> np.ndarray:
        """Apply chi-squared test for outlier rejection"""
        h_pred = self._measurement_function(self.state.vector, self.uwb_sensor.anchors)
        innovations = np.abs(ranges - h_pred)

        threshold = self.config.filter.gate_threshold * self.config.uwb.range_noise_std
        mask = innovations < threshold

        self.outliers_rejected += np.sum(~mask)

        return mask

    @property
    def position_uncertainty(self) -> np.ndarray:
        """Get position standard deviations"""
        return np.sqrt(np.diag(self.P[:3, :3]))


# ============================================================================
# TRAJECTORY GENERATION
# ============================================================================

class TrajectoryGenerator:
    """Generate realistic wheel loader trajectories"""

    def __init__(self, config: SimulationConfig):
        self.config = config

    def generate(self, imu_dt: float, uwb_dt: float) -> Dict[str, np.ndarray]:
        """
        Generate complete trajectory

        Returns dict with all trajectory data
        """
        duration = self.config.duration
        times = np.arange(0, duration, imu_dt)
        n = len(times)

        # Pre-allocate arrays
        positions = np.zeros((n, 3))
        velocities = np.zeros((n, 3))
        accelerations = np.zeros((n, 3))
        orientations = np.zeros((n, 3))
        angular_velocities = np.zeros((n, 3))

        # Trajectory parameters
        v_forward = self.config.trajectory_velocity
        y_amp = self.config.y_amplitude
        z_amp = self.config.z_amplitude
        y_freq = 0.15  # Hz
        z_freq = 0.2   # Hz

        for i, t in enumerate(times):
            # Position
            positions[i, 0] = v_forward * t
            positions[i, 1] = self.config.tag_position_y + y_amp * np.sin(2*np.pi*y_freq*t)
            positions[i, 2] = self.config.tag_position_z + z_amp * np.sin(2*np.pi*z_freq*t)

            # Velocity
            velocities[i, 0] = v_forward
            velocities[i, 1] = y_amp * 2*np.pi*y_freq * np.cos(2*np.pi*y_freq*t)
            velocities[i, 2] = z_amp * 2*np.pi*z_freq * np.cos(2*np.pi*z_freq*t)

            # Acceleration
            accelerations[i, 1] = -y_amp * (2*np.pi*y_freq)**2 * np.sin(2*np.pi*y_freq*t)
            accelerations[i, 2] = -z_amp * (2*np.pi*z_freq)**2 * np.sin(2*np.pi*z_freq*t)

            # Orientation (small roll/pitch from motion)
            orientations[i, 0] = 0.03 * np.sin(2*np.pi*z_freq*t)  # roll
            orientations[i, 1] = 0.02 * np.sin(2*np.pi*y_freq*t)  # pitch
            orientations[i, 2] = 0.005 * t  # slow yaw drift

            # Angular velocity
            angular_velocities[i, 0] = 0.03 * 2*np.pi*z_freq * np.cos(2*np.pi*z_freq*t)
            angular_velocities[i, 1] = 0.02 * 2*np.pi*y_freq * np.cos(2*np.pi*y_freq*t)
            angular_velocities[i, 2] = 0.005

        # UWB sampling times
        uwb_times = np.arange(0, duration, uwb_dt)
        uwb_indices = (uwb_times / imu_dt).astype(int)
        uwb_indices = np.clip(uwb_indices, 0, n-1)

        return {
            'imu_times': times,
            'uwb_times': uwb_times,
            'uwb_indices': uwb_indices,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'orientations': orientations,
            'angular_velocities': angular_velocities
        }


# ============================================================================
# BASELINE ALGORITHM
# ============================================================================

class UWBTrilateration:
    """Standalone UWB position estimation"""

    def __init__(self, anchor_positions: np.ndarray):
        self.anchors = anchor_positions
        self.n_anchors = len(anchor_positions)

    def estimate_position(self, ranges: np.ndarray,
                          initial_guess: Optional[np.ndarray] = None) -> np.ndarray:
        """Estimate position from ranges using least squares"""
        if initial_guess is None:
            # Use centroid of 4 closest anchors
            indices = np.argsort(ranges)[:4]
            initial_guess = np.mean(self.anchors[indices], axis=0)

        def residuals(pos):
            predicted = np.linalg.norm(self.anchors - pos, axis=1)
            return ranges - predicted

        result = least_squares(residuals, initial_guess, method='lm')
        return result.x if result.success else initial_guess


# ============================================================================
# SIMULATION RUNNER
# ============================================================================

class SimulationRunner:
    """Orchestrates complete simulation"""

    def __init__(self, config: SystemConfig):
        self.config = config
        np.random.seed(config.simulation.random_seed)

        # Create components
        self.trajectory_gen = TrajectoryGenerator(config.simulation)
        self.ekf = ExtendedKalmanFilter(config)
        self.uwb_baseline = UWBTrilateration(config.anchors.generate_positions())

        self.results = {}

    def run(self) -> Dict[str, Any]:
        """Run complete simulation"""
        print("\n" + "="*80)
        print("EKF UWB+IMU LOCALIZATION - REDESIGNED IMPLEMENTATION")
        print("="*80)

        # Generate trajectory
        print("\n[1/4] Generating trajectory...")
        trajectory = self.trajectory_gen.generate(
            self.config.imu.dt,
            self.config.uwb.dt
        )
        print(f"  ✓ Generated {len(trajectory['imu_times'])} IMU samples")
        print(f"  ✓ Generated {len(trajectory['uwb_times'])} UWB samples")

        # Generate measurements
        print("\n[2/4] Simulating sensor measurements...")
        measurements = self._generate_measurements(trajectory)
        print(f"  ✓ IMU measurements: {len(measurements['gyro'])} samples")
        print(f"  ✓ UWB measurements: {len(measurements['ranges'])} samples")

        # Run EKF
        print("\n[3/4] Running Extended Kalman Filter...")
        ekf_results = self._run_ekf(trajectory, measurements)
        print(f"  ✓ Predictions: {self.ekf.prediction_count}")
        print(f"  ✓ Updates: {self.ekf.update_count}")
        print(f"  ✓ Outliers rejected: {self.ekf.outliers_rejected}")

        # Run baseline
        print("\n[4/4] Running UWB-only baseline...")
        baseline_results = self._run_baseline(trajectory, measurements)
        print(f"  ✓ Baseline estimates: {len(baseline_results['positions'])}")

        # Compute metrics
        print("\n[✓] Computing performance metrics...")
        metrics = self._compute_metrics(trajectory, ekf_results, baseline_results)

        self.results = {
            'trajectory': trajectory,
            'measurements': measurements,
            'ekf': ekf_results,
            'baseline': baseline_results,
            'metrics': metrics,
            'config': self.config
        }

        self._print_summary(metrics)

        return self.results

    def _generate_measurements(self, trajectory: Dict) -> Dict:
        """Generate all sensor measurements"""
        n_imu = len(trajectory['imu_times'])
        n_uwb = len(trajectory['uwb_times'])

        # IMU measurements
        gyro_measurements = np.zeros((n_imu, 3))
        accel_measurements = np.zeros((n_imu, 3))

        gravity_world = np.array([0, 0, 9.81])

        for i in range(n_imu):
            # Transform to body frame
            R = Rotation.from_euler('xyz', trajectory['orientations'][i])
            R_inv = R.inv().as_matrix()

            # Specific force = accel + gravity
            accel_body = R_inv @ trajectory['accelerations'][i]
            gravity_body = R_inv @ gravity_world
            specific_force = accel_body + gravity_body

            # Generate noisy measurements
            gyro_measurements[i], accel_measurements[i] = self.ekf.imu_sensor.measure(
                trajectory['angular_velocities'][i],
                specific_force
            )

        # UWB measurements
        uwb_indices = trajectory['uwb_indices']
        range_measurements = np.zeros((n_uwb, self.config.anchors.total_anchors))

        for i, idx in enumerate(uwb_indices):
            position = trajectory['positions'][idx]
            range_measurements[i] = self.ekf.uwb_sensor.measure(position)

        return {
            'gyro': gyro_measurements,
            'accel': accel_measurements,
            'ranges': range_measurements
        }

    def _run_ekf(self, trajectory: Dict, measurements: Dict) -> Dict:
        """Run EKF on measurements"""
        n_imu = len(trajectory['imu_times'])

        # Storage
        ekf_positions = np.zeros((n_imu, 3))
        ekf_velocities = np.zeros((n_imu, 3))
        ekf_orientations = np.zeros((n_imu, 3))
        ekf_uncertainties = np.zeros((n_imu, 3))

        # Initialize
        self.ekf.initialize(
            trajectory['positions'][0],
            trajectory['velocities'][0],
            trajectory['orientations'][0]
        )

        # Initial UWB update
        self.ekf.update_uwb(measurements['ranges'][0])

        ekf_positions[0] = self.ekf.state.position
        ekf_velocities[0] = self.ekf.state.velocity
        ekf_orientations[0] = self.ekf.state.orientation
        ekf_uncertainties[0] = self.ekf.position_uncertainty

        # Main loop
        uwb_idx = 1
        uwb_indices = trajectory['uwb_indices']

        for i in range(1, n_imu):
            # UWB update if available
            if uwb_idx < len(uwb_indices) and i == uwb_indices[uwb_idx]:
                self.ekf.update_uwb(measurements['ranges'][uwb_idx])
                uwb_idx += 1

            # IMU prediction
            self.ekf.predict_imu(
                measurements['gyro'][i],
                measurements['accel'][i],
                self.config.imu.dt
            )

            # Store
            ekf_positions[i] = self.ekf.state.position
            ekf_velocities[i] = self.ekf.state.velocity
            ekf_orientations[i] = self.ekf.state.orientation
            ekf_uncertainties[i] = self.ekf.position_uncertainty

        return {
            'positions': ekf_positions,
            'velocities': ekf_velocities,
            'orientations': ekf_orientations,
            'uncertainties': ekf_uncertainties
        }

    def _run_baseline(self, trajectory: Dict, measurements: Dict) -> Dict:
        """Run UWB-only baseline"""
        uwb_indices = trajectory['uwb_indices']
        n_uwb = len(uwb_indices)

        baseline_positions = np.zeros((n_uwb, 3))

        for i in range(n_uwb):
            initial_guess = baseline_positions[i-1] if i > 0 else None
            baseline_positions[i] = self.uwb_baseline.estimate_position(
                measurements['ranges'][i],
                initial_guess
            )

        return {'positions': baseline_positions}

    def _compute_metrics(self, trajectory: Dict, ekf_results: Dict,
                         baseline_results: Dict) -> Dict:
        """Compute performance metrics"""
        uwb_indices = trajectory['uwb_indices']

        # EKF errors
        ekf_pos_errors = np.linalg.norm(
            ekf_results['positions'][uwb_indices] - trajectory['positions'][uwb_indices],
            axis=1
        )

        # Baseline errors
        baseline_pos_errors = np.linalg.norm(
            baseline_results['positions'] - trajectory['positions'][uwb_indices],
            axis=1
        )

        # Velocity errors (full IMU rate for comprehensive evaluation)
        vel_errors = np.linalg.norm(
            ekf_results['velocities'] - trajectory['velocities'],
            axis=1
        )

        # Also compute velocity errors at UWB times for comparison
        vel_errors_uwb = np.linalg.norm(
            ekf_results['velocities'][uwb_indices] - trajectory['velocities'][uwb_indices],
            axis=1
        )

        # Per-axis
        pos_diff = ekf_results['positions'][uwb_indices] - trajectory['positions'][uwb_indices]

        # Full-rate position errors for comprehensive analysis
        ekf_pos_errors_full = np.linalg.norm(
            ekf_results['positions'] - trajectory['positions'],
            axis=1
        )
        pos_diff_full = ekf_results['positions'] - trajectory['positions']

        return {
            # UWB-rate metrics (at measurement times only)
            'ekf_pos_rmse_uwb': np.sqrt(np.mean(ekf_pos_errors**2)),
            'ekf_pos_mean_uwb': np.mean(ekf_pos_errors),
            'ekf_pos_max_uwb': np.max(ekf_pos_errors),
            'baseline_pos_rmse': np.sqrt(np.mean(baseline_pos_errors**2)),
            'baseline_pos_mean': np.mean(baseline_pos_errors),
            'baseline_pos_max': np.max(baseline_pos_errors),
            'vel_rmse_uwb': np.sqrt(np.mean(vel_errors_uwb**2)),
            'x_rmse_uwb': np.sqrt(np.mean(pos_diff[:, 0]**2)),
            'y_rmse_uwb': np.sqrt(np.mean(pos_diff[:, 1]**2)),
            'z_rmse_uwb': np.sqrt(np.mean(pos_diff[:, 2]**2)),

            # Full-rate metrics (comprehensive evaluation)
            'ekf_pos_rmse': np.sqrt(np.mean(ekf_pos_errors_full**2)),
            'ekf_pos_mean': np.mean(ekf_pos_errors_full),
            'ekf_pos_max': np.max(ekf_pos_errors_full),
            'vel_rmse': np.sqrt(np.mean(vel_errors**2)),
            'x_rmse': np.sqrt(np.mean(pos_diff_full[:, 0]**2)),
            'y_rmse': np.sqrt(np.mean(pos_diff_full[:, 1]**2)),
            'z_rmse': np.sqrt(np.mean(pos_diff_full[:, 2]**2))
        }

    def _print_summary(self, metrics: Dict):
        """Print performance summary"""
        print("\n" + "="*80)
        print("PERFORMANCE SUMMARY")
        print("="*80)

        print(f"\nEKF Position Errors (Full-Rate Analysis):")
        print(f"  RMSE: {metrics['ekf_pos_rmse']*100:.2f} cm")
        print(f"  Mean: {metrics['ekf_pos_mean']*100:.2f} cm")
        print(f"  Max:  {metrics['ekf_pos_max']*100:.2f} cm")

        print(f"\nEKF Position Errors (UWB-Rate Only):")
        print(f"  RMSE: {metrics['ekf_pos_rmse_uwb']*100:.2f} cm")
        print(f"  Mean: {metrics['ekf_pos_mean_uwb']*100:.2f} cm")
        print(f"  Max:  {metrics['ekf_pos_max_uwb']*100:.2f} cm")

        print(f"\nUWB-Only Baseline:")
        print(f"  RMSE: {metrics['baseline_pos_rmse']*100:.2f} cm")
        print(f"  Mean: {metrics['baseline_pos_mean']*100:.2f} cm")
        print(f"  Max:  {metrics['baseline_pos_max']*100:.2f} cm")

        improvement_full = (metrics['baseline_pos_rmse'] - metrics['ekf_pos_rmse']) / metrics['baseline_pos_rmse'] * 100
        improvement_uwb = (metrics['baseline_pos_rmse'] - metrics['ekf_pos_rmse_uwb']) / metrics['baseline_pos_rmse'] * 100
        print(f"\nImprovement (Full-Rate): {improvement_full:.1f}%")
        print(f"Improvement (UWB-Rate): {improvement_uwb:.1f}%")

        print(f"\nPer-Axis RMSE (Full-Rate):")
        print(f"  X: {metrics['x_rmse']*100:.2f} cm")
        print(f"  Y: {metrics['y_rmse']*100:.2f} cm")
        print(f"  Z: {metrics['z_rmse']*100:.2f} cm")

        print(f"\nVelocity RMSE (Full-Rate): {metrics['vel_rmse']*100:.2f} cm/s")
        print(f"Velocity RMSE (UWB-Rate): {metrics['vel_rmse_uwb']*100:.2f} cm/s")


# ============================================================================
# VISUALIZATION
# ============================================================================

class Visualizer:
    """Create comprehensive visualizations"""

    def __init__(self, results: Dict):
        self.results = results
        self.config = results['config']

    def create_plots(self, save_path: str = 'ekf_uwb_imu_localization.png'):
        """Create complete visualization matching reference style"""
        fig = plt.figure(figsize=(20, 13))

        # Set overall style
        plt.rcParams['font.size'] = 9
        plt.rcParams['axes.labelsize'] = 10
        plt.rcParams['axes.titlesize'] = 11
        plt.rcParams['legend.fontsize'] = 8

        # Row 1: 3D Trajectory | Per-Axis Position Errors | Velocity Errors
        ax1 = fig.add_subplot(3, 3, 1, projection='3d')
        self._plot_3d(ax1)

        ax2 = fig.add_subplot(3, 3, 2)
        self._plot_errors(ax2)

        ax3 = fig.add_subplot(3, 3, 3)
        self._plot_velocity_errors(ax3)

        # Row 2: Per-Axis Error Distribution | Position Uncertainty | Orientation
        ax4 = fig.add_subplot(3, 3, 4)
        self._plot_axis_errors(ax4)

        ax5 = fig.add_subplot(3, 3, 5)
        self._plot_uncertainty(ax5)

        ax6 = fig.add_subplot(3, 3, 6)
        self._plot_orientation(ax6)

        # Row 3: XY Trajectory | XZ Trajectory | Summary
        ax7 = fig.add_subplot(3, 3, 7)
        self._plot_xy(ax7)

        ax8 = fig.add_subplot(3, 3, 8)
        self._plot_xz(ax8)

        ax9 = fig.add_subplot(3, 3, 9)
        self._plot_summary(ax9)

        plt.tight_layout(pad=1.5)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n✓ Visualization saved to: {save_path}")
        plt.show()

    def _plot_3d(self, ax):
        """3D trajectory"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        baseline = self.results['baseline']
        anchors = self.config.anchors.generate_positions()

        # Plot anchors with two rows at different Z heights
        ax.scatter(anchors[:21, 0], anchors[:21, 1], anchors[:21, 2],
                  c='darkred', s=100, marker='s', alpha=0.8, label='UWB Anchors', edgecolors='black', linewidth=0.5)
        ax.scatter(anchors[21:, 0], anchors[21:, 1], anchors[21:, 2],
                  c='darkred', s=100, marker='s', alpha=0.8, edgecolors='black', linewidth=0.5)

        # Draw anchor plane
        xx, zz = np.meshgrid([0, 200], [3, 6])
        yy = np.zeros_like(xx)
        ax.plot_surface(xx, yy, zz, alpha=0.1, color='red', label='Anchor plane')

        # Plot trajectories
        ax.plot(traj['positions'][:, 0], traj['positions'][:, 1],
               traj['positions'][:, 2], 'darkgreen', label='True', linewidth=2.5, alpha=0.9)
        ax.plot(ekf['positions'][:, 0], ekf['positions'][:, 1],
               ekf['positions'][:, 2], 'darkblue', label='EKF+IMU', linewidth=2, alpha=0.8)

        # Plot baseline with scatter for visibility
        uwb_idx = traj['uwb_indices']
        ax.scatter(baseline['positions'][:, 0],
                  np.full(len(baseline['positions']), self.config.simulation.tag_position_y),
                  baseline['positions'][:, 2],
                  c='orange', s=20, alpha=0.6, label='UWB-only')

        # Mark start and end
        ax.scatter([traj['positions'][0, 0]], [traj['positions'][0, 1]], [traj['positions'][0, 2]],
                  c='green', s=200, marker='o', edgecolors='black', linewidth=2, label='Start', zorder=10)
        ax.scatter([traj['positions'][-1, 0]], [traj['positions'][-1, 1]], [traj['positions'][-1, 2]],
                  c='purple', s=200, marker='X', edgecolors='black', linewidth=2, label='End', zorder=10)

        ax.set_xlabel('X (m)', fontsize=10, labelpad=8)
        ax.set_ylabel('Y (m)', fontsize=10, labelpad=8)
        ax.set_zlabel('Z (m)', fontsize=10, labelpad=8)
        ax.set_title('3D Trajectory\n(42 anchors: 2 rows at Y=0m & Y=10m, Z=6m, 10m X-interval)',
                     fontweight='bold', fontsize=11)
        ax.legend(loc='upper left', fontsize=8, framealpha=0.9)
        ax.view_angle = 30
        ax.grid(True, alpha=0.3)

    def _plot_errors(self, ax):
        """Per-axis position errors over time with filled areas"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        uwb_times = traj['uwb_times']
        uwb_idx = traj['uwb_indices']

        # Calculate per-axis errors
        errors = ekf['positions'][uwb_idx] - traj['positions'][uwb_idx]
        x_errors = errors[:, 0] * 100  # cm
        y_errors = errors[:, 1] * 100  # cm
        z_errors = errors[:, 2] * 100  # cm

        # Plot with filled areas
        ax.fill_between(uwb_times, 0, x_errors, alpha=0.6, color='red', label='X-axis error', linewidth=0)
        ax.fill_between(uwb_times, 0, y_errors, alpha=0.6, color='green', label='Y-axis error', linewidth=0)
        ax.fill_between(uwb_times, 0, z_errors, alpha=0.7, color='blue', label='Z-axis error', linewidth=0)

        # Plot lines on top
        ax.plot(uwb_times, x_errors, 'darkred', linewidth=1.5, alpha=0.8)
        ax.plot(uwb_times, y_errors, 'darkgreen', linewidth=1.5, alpha=0.8)
        ax.plot(uwb_times, z_errors, 'darkblue', linewidth=1.5, alpha=0.8)

        # Reference lines
        ax.axhline(10, color='gray', linestyle=':', label='10cm reference', linewidth=1.5, alpha=0.7)
        ax.axhline(0, color='black', linestyle='-', linewidth=1, alpha=0.4)

        ax.set_xlabel('Time (s)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Position Error (cm)', fontweight='bold', fontsize=10)
        ax.set_title('Per-Axis Position Errors Over Time', fontweight='bold', fontsize=11)
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax.grid(True, alpha=0.3)
        # Set y-axis to show appropriate range
        max_error = max(np.max(np.abs(x_errors)), np.max(np.abs(y_errors)), np.max(np.abs(z_errors)))
        ax.set_ylim([0, min(max_error * 1.1, 250)])

    def _plot_velocity_errors(self, ax):
        """Velocity estimation errors over time"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        imu_times = traj['imu_times']

        # Calculate velocity errors
        vel_errors = np.linalg.norm(ekf['velocities'] - traj['velocities'], axis=1) * 100  # cm/s
        vel_mean = np.mean(vel_errors)

        # Plot with filled area
        ax.fill_between(imu_times, 0, vel_errors, alpha=0.5, color='royalblue', linewidth=0)
        ax.plot(imu_times, vel_errors, 'darkblue', linewidth=1, alpha=0.8, label='Velocity error')
        ax.axhline(vel_mean, color='red', linestyle='--', linewidth=2,
                  label=f'Mean: {vel_mean:.1f} cm/s', alpha=0.8)

        ax.set_xlabel('Time (s)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Velocity Error (cm/s)', fontweight='bold', fontsize=10)
        ax.set_title('Velocity Estimation Error Over Time', fontweight='bold', fontsize=11)
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax.grid(True, alpha=0.3)
        # Set y-axis to show appropriate range
        ax.set_ylim([0, min(np.max(vel_errors) * 1.1, 150)])

    def _plot_uncertainty(self, ax):
        """EKF position uncertainty (1σ) over time"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        uwb_times = traj['uwb_times']
        uwb_idx = traj['uwb_indices']

        # Get uncertainties at UWB times
        uncertainties = ekf['uncertainties'][uwb_idx]
        sigma_x = uncertainties[:, 0] * 100  # cm
        sigma_y = uncertainties[:, 1] * 100  # cm
        sigma_z = uncertainties[:, 2] * 100  # cm

        # Plot with filled areas
        ax.fill_between(uwb_times, 0, sigma_x, alpha=0.6, color='red', label='σ_x', linewidth=0)
        ax.fill_between(uwb_times, 0, sigma_y, alpha=0.6, color='green', label='σ_y', linewidth=0)
        ax.fill_between(uwb_times, 0, sigma_z, alpha=0.7, color='blue', label='σ_z', linewidth=0)

        # Plot lines
        ax.plot(uwb_times, sigma_x, 'darkred', linewidth=1.5, alpha=0.8)
        ax.plot(uwb_times, sigma_y, 'darkgreen', linewidth=1.5, alpha=0.8)
        ax.plot(uwb_times, sigma_z, 'darkblue', linewidth=1.5, alpha=0.8)

        # Mark UWB update times
        ax.vlines(uwb_times, 0, 5, colors='gray', linestyles=':', linewidth=0.5, alpha=0.5)

        ax.set_xlabel('Time (s)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Position Uncertainty (cm)', fontweight='bold', fontsize=10)
        ax.set_title('EKF Position Uncertainty (1σ)\\nVertical lines = UWB updates',
                    fontweight='bold', fontsize=11)
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)

    def _plot_orientation(self, ax):
        """Orientation estimation from IMU"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        imu_times = traj['imu_times']

        # Convert to degrees
        true_roll = np.rad2deg(traj['orientations'][:, 0])
        true_pitch = np.rad2deg(traj['orientations'][:, 1])
        true_yaw = np.rad2deg(traj['orientations'][:, 2])

        ekf_roll = np.rad2deg(ekf['orientations'][:, 0])
        ekf_pitch = np.rad2deg(ekf['orientations'][:, 1])
        ekf_yaw = np.rad2deg(ekf['orientations'][:, 2])

        # Plot true vs estimated
        ax.plot(imu_times, true_roll, 'r-', linewidth=1.5, alpha=0.7, label='True roll')
        ax.plot(imu_times, ekf_roll, 'r--', linewidth=1.5, alpha=0.9, label='EKF roll')

        ax.plot(imu_times, true_pitch, 'g-', linewidth=1.5, alpha=0.7, label='True pitch')
        ax.plot(imu_times, ekf_pitch, 'g--', linewidth=1.5, alpha=0.9, label='EKF pitch')

        ax.plot(imu_times, true_yaw, 'b-', linewidth=1.5, alpha=0.7, label='True yaw')
        ax.plot(imu_times, ekf_yaw, 'b--', linewidth=1.5, alpha=0.9, label='EKF yaw')

        ax.set_xlabel('Time (s)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Angle (degrees)', fontweight='bold', fontsize=10)
        ax.set_title('Orientation Estimation (from IMU)', fontweight='bold', fontsize=11)
        ax.legend(loc='upper left', ncol=2, fontsize=7, framealpha=0.9)
        ax.grid(True, alpha=0.3)

    def _plot_axis_errors(self, ax):
        """Per-axis error distribution with histogram"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        uwb_idx = traj['uwb_indices']

        errors = ekf['positions'][uwb_idx] - traj['positions'][uwb_idx]
        x_errors = errors[:, 0] * 100
        y_errors = errors[:, 1] * 100
        z_errors = errors[:, 2] * 100

        # Create histogram
        bins = np.linspace(0, 50, 40)
        ax.hist(np.abs(x_errors), bins=bins, alpha=0.7, color='red', label='X-axis', density=True)
        ax.hist(np.abs(y_errors), bins=bins, alpha=0.7, color='green', label='Y-axis', density=True)
        ax.hist(np.abs(z_errors), bins=bins, alpha=0.7, color='blue', label='Z-axis', density=True)

        # Add vertical lines for mean values
        ax.axvline(np.mean(np.abs(x_errors)), color='darkred', linestyle='--', linewidth=2, alpha=0.8)
        ax.axvline(np.mean(np.abs(y_errors)), color='darkgreen', linestyle='--', linewidth=2, alpha=0.8)
        ax.axvline(np.mean(np.abs(z_errors)), color='darkblue', linestyle='--', linewidth=2, alpha=0.8)

        ax.set_xlabel('Position Error (cm)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Probability Density', fontweight='bold', fontsize=10)
        ax.set_title('Per-Axis Error Distribution', fontweight='bold', fontsize=11)
        ax.legend(fontsize=8, framealpha=0.9)
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_xlim(left=0)

    def _plot_xy(self, ax):
        """XY trajectory view (top down)"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        baseline = self.results['baseline']
        anchors = self.config.anchors.generate_positions()

        # Plot anchors
        unique_x = np.unique(anchors[:, 0])
        ax.scatter(unique_x, np.zeros_like(unique_x), c='darkred', s=100,
                  marker='s', alpha=0.8, label='Anchors (Y=0)', edgecolors='black', linewidth=0.5, zorder=5)
        ax.axhline(0, color='red', linestyle='--', linewidth=1.5, alpha=0.5)

        # Mark start and end
        ax.scatter([traj['positions'][0, 0]], [traj['positions'][0, 1]],
                  c='green', s=200, marker='o', edgecolors='black', linewidth=2, label='Start', zorder=10)
        ax.scatter([traj['positions'][-1, 0]], [traj['positions'][-1, 1]],
                  c='purple', s=200, marker='X', edgecolors='black', linewidth=2, label='End', zorder=10)

        # Plot trajectories
        ax.plot(traj['positions'][:, 0], traj['positions'][:, 1],
               'darkgreen', label='True', linewidth=2.5, alpha=0.9, zorder=3)
        ax.plot(ekf['positions'][:, 0], ekf['positions'][:, 1],
               'darkblue', label='EKF+IMU', linewidth=2, alpha=0.8, zorder=4)

        # Plot baseline with scatter
        uwb_idx = traj['uwb_indices']
        ax.scatter(baseline['positions'][:, 0],
                  np.full(len(baseline['positions']), self.config.simulation.tag_position_y),
                  c='orange', s=20, alpha=0.6, label='UWB-only', zorder=2)

        ax.set_xlabel('X (m)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Y (m)', fontweight='bold', fontsize=10)
        ax.set_title('XY Trajectory (Top View)', fontweight='bold', fontsize=11)
        ax.legend(fontsize=8, framealpha=0.9, loc='upper left')
        ax.grid(True, alpha=0.3)

        # Set Y-axis range to show anchors at Y=0 and trajectory at Y≈3m
        y_min = min(0, np.min(traj['positions'][:, 1])) - 0.5
        y_max = max(np.max(traj['positions'][:, 1]), np.max(ekf['positions'][:, 1])) + 0.5
        ax.set_ylim(y_min, y_max)

        # Set X-axis range
        ax.set_xlim(0, 200)

    def _plot_xz(self, ax):
        """XZ trajectory view (side view)"""
        traj = self.results['trajectory']
        ekf = self.results['ekf']
        baseline = self.results['baseline']
        anchors = self.config.anchors.generate_positions()

        # Debug: Print actual Z ranges
        print(f"\nDEBUG XZ Plot:")
        print(f"Truth Z range: {np.min(traj['positions'][:, 2]):.2f} to {np.max(traj['positions'][:, 2]):.2f}")
        print(f"EKF Z range: {np.min(ekf['positions'][:, 2]):.2f} to {np.max(ekf['positions'][:, 2]):.2f}")
        print(f"Anchor Z values: {np.unique(anchors[:, 2])}")

        # Plot anchors at Z=6m (both rows)
        ax.scatter(anchors[:, 0], anchors[:, 2], c='darkred', s=100,
                  marker='s', alpha=0.8, label='Anchors (Z=6m)', edgecolors='black', linewidth=0.5, zorder=5)

        # Plot horizontal line at anchor height
        ax.axhline(y=6.0, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
        ax.axhline(y=5.0, color='gray', linestyle=':', linewidth=1.5, alpha=0.6, label='Tag center (Z=5m)')

        # Plot trajectories
        ax.plot(traj['positions'][:, 0], traj['positions'][:, 2],
               'darkgreen', label='True', linewidth=2.5, alpha=0.9, zorder=3)
        ax.plot(ekf['positions'][:, 0], ekf['positions'][:, 2],
               'darkblue', label='EKF+IMU', linewidth=2, alpha=0.8, zorder=4)

        # Plot baseline with scatter
        ax.scatter(baseline['positions'][:, 0], baseline['positions'][:, 2],
                  c='orange', s=20, alpha=0.6, label='UWB-only', zorder=2)

        # Set explicit Z axis limits to show proper range (4m to 7m)
        ax.set_ylim(4.0, 7.0)

        ax.set_xlabel('X (m)', fontweight='bold', fontsize=10)
        ax.set_ylabel('Z (m)', fontweight='bold', fontsize=10)
        ax.set_title('XZ Trajectory (Side View)', fontweight='bold', fontsize=11)
        ax.legend(fontsize=8, framealpha=0.9)
        ax.grid(True, alpha=0.3)

    def _plot_summary(self, ax):
        """Performance summary"""
        ax.axis('off')

        metrics = self.results['metrics']

        summary_text = f"""PERFORMANCE SUMMARY

UWB Configuration:
  • 42 anchors in 2 horizontal rows (X: 0-200m, interval: 10m)
  • Row 1: Y=0m, Z=6m
  • Row 2: Y=10m, Z=6m
  • Horizontal baseline: 10m (Y-direction)
  • Measurement noise: 18cm

IMU Configuration:
  • ADIS16470 integration error model
  • Gyro bias: 4.0°/hr
  • Accel bias: 0.016mg
  • Sample rate: 100Hz

EKF-IMU Results:
  • Position RMSE: {metrics['ekf_pos_rmse']*100:.2f} cm
  • Position Mean: {metrics['ekf_pos_mean']*100:.2f} cm
  • Position Max: {metrics['ekf_pos_max']*100:.2f} cm

UWB-Only Results:
  • Position RMSE: {metrics['baseline_pos_rmse']*100:.2f} cm
  • Position Mean: {metrics['baseline_pos_mean']*100:.2f} cm
  • Position Max: {metrics['baseline_pos_max']*100:.2f} cm
        """

        ax.text(0.05, 0.95, summary_text, transform=ax.transAxes,
               fontsize=8.5, family='monospace', verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8, pad=1))


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """Main execution"""
    print("\n" + "="*80)
    print("EKF UWB+IMU LOCALIZATION - REDESIGNED & MODULAR")
    print("="*80)
    print("\nKey Features:")
    print("  • Clean object-oriented design")
    print("  • Modular sensor models")
    print("  • Configuration-driven system")
    print("  • Comprehensive visualization")
    print("  • Statistical performance tracking")

    # Create configuration
    config = SystemConfig()

    # Run simulation
    runner = SimulationRunner(config)
    results = runner.run()

    # Visualize
    viz = Visualizer(results)
    viz.create_plots()

    print("\n" + "="*80)
    print("✓ SIMULATION COMPLETE!")
    print("="*80 + "\n")

    return results


if __name__ == "__main__":
    results = main()
