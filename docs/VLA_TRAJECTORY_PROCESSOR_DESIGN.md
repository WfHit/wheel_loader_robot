# VLA Trajectory Processor Design Document

## Overview

The VLA (Vision-Language-Action) Trajectory Processor is a central module that receives 7-DOF trajectory predictions from the VLA model, decomposes them into separate control channels, applies MPC-style trajectory smoothing with kinematic constraints, and publishes synchronized 50Hz setpoints for chassis, boom, and tilt controllers.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              VLA Model Output                                   │
│   [Δx, Δy, Δheading, steering_angle, bucket_height, tilt_angle, slip_rate]     │
│                    16 steps × 100ms = 1.6s horizon (Body Frame)                 │
└─────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         VlaTrajectoryProcessor Module                           │
│                              (Runs at 50Hz)                                     │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │  1. DECOMPOSITION STAGE                                                   │  │
│  │     ├─ Transform body-frame deltas to world-frame trajectory              │  │
│  │     ├─ Extract chassis trajectory (x, y, heading, steering)               │  │
│  │     ├─ Extract boom trajectory (bucket_height → boom_angle via IK)        │  │
│  │     └─ Extract tilt trajectory (tilt_angle)                               │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                        │                                        │
│                                        ▼                                        │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │  2. TRAJECTORY SMOOTHING STAGE (MPC-Style Optimization)                   │  │
│  │     ├─ Apply kinematic constraints per channel                            │  │
│  │     ├─ Compute time-scaling factor for synchronized motion                │  │
│  │     ├─ Generate jerk-limited smooth trajectories                          │  │
│  │     └─ Interpolate 100ms VLA points to 20ms (50Hz) control points         │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                        │                                        │
│                                        ▼                                        │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │  3. OUTPUT STAGE (50Hz Publication)                                       │  │
│  │     ├─ Publish ChassisTrajectorySetpoint (with slip_rate feedforward)     │  │
│  │     ├─ Publish BoomTrajectorySetpoint                                     │  │
│  │     └─ Publish TiltTrajectorySetpoint                                     │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘
                    │                    │                    │
                    ▼                    ▼                    ▼
        ┌───────────────────┐ ┌───────────────────┐ ┌───────────────────┐
        │ Flight Mode Mgr   │ │ Flight Mode Mgr   │ │ Flight Mode Mgr   │
        │ Chassis Control   │ │  Boom Control     │ │  Tilt Control     │
        │     (50Hz)        │ │     (50Hz)        │ │     (50Hz)        │
        └───────────────────┘ └───────────────────┘ └───────────────────┘
                    │                    │                    │
                    ▼                    ▼                    ▼
        ┌───────────────────┐ ┌───────────────────┐ ┌───────────────────┐
        │ Chassis Actuators │ │  Boom Actuators   │ │  Tilt Actuators   │
        │ (Drivetrain +     │ │  (Hydraulic Cyl)  │ │  (Hydraulic Cyl)  │
        │  Articulation)    │ │                   │ │                   │
        └───────────────────┘ └───────────────────┘ └───────────────────┘
```

---

## 1. Input: VLA Trajectory Message

### 1.1 New uORB Message: `VlaTrajectory.msg`

```msg
# VLA 7-DOF trajectory output message
# Contains 16-step trajectory prediction from VLA model
# All values are in BODY FRAME with DELTA semantics

uint64 timestamp                    # time since system start (microseconds)
uint64 vla_timestamp                # timestamp from VLA model inference

# Trajectory horizon configuration
uint8 num_steps                     # Number of valid steps (max 16)
float32 step_interval               # Time interval between steps [s] (typically 0.1s)

# 7-DOF Trajectory Data (16 steps max)
# Index 0 = current step, Index 15 = 1.5s into future

# Chassis motion (body frame deltas)
float32[16] delta_x                 # Delta X position in body frame [m]
float32[16] delta_y                 # Delta Y position in body frame [m]
float32[16] delta_heading           # Delta heading (yaw change) [rad]
float32[16] steering_angle          # Articulation angle [rad]

# End effector state (absolute values)
float32[16] bucket_height           # Bucket height from ground [m]
float32[16] tilt_angle              # Bucket tilt angle [rad]

# Traction control feedforward
float32[16] target_slip_rate        # Target tire slip rate [0.0-1.0]

# Confidence and validity
float32 confidence                  # VLA model confidence [0.0-1.0]
bool valid                          # Trajectory is valid for execution

# Sequence tracking
uint32 sequence_id                  # Sequence ID for trajectory tracking
```

### 1.2 Coordinate Frame Definitions

| Field | Frame | Semantics | Description |
|-------|-------|-----------|-------------|
| `delta_x` | Body | Delta | Forward movement relative to current pose |
| `delta_y` | Body | Delta | Lateral movement relative to current pose |
| `delta_heading` | Body | Delta | Heading change from current heading |
| `steering_angle` | Vehicle | Absolute | Articulation angle of center pivot |
| `bucket_height` | World | Absolute | Height of bucket tip from ground |
| `tilt_angle` | Boom-relative | Absolute | Bucket curl angle relative to boom |
| `target_slip_rate` | N/A | Absolute | Desired wheel slip ratio |

---

## 2. Decomposition Stage

### 2.1 Body-to-World Frame Transformation

Transform body-frame deltas to world-frame absolute trajectory:

```cpp
struct WorldTrajectoryPoint {
    float x;              // World X position [m]
    float y;              // World Y position [m]
    float heading;        // World heading [rad]
    float steering;       // Articulation angle [rad]
    float timestamp;      // Time from trajectory start [s]
};

void decomposeToWorldFrame(
    const VlaTrajectory& vla_traj,
    const VehicleState& current_state,
    WorldTrajectoryPoint* chassis_traj,  // Output: 16 points
    int& num_points)
{
    float x = current_state.x;
    float y = current_state.y;
    float heading = current_state.heading;

    for (int i = 0; i < vla_traj.num_steps; i++) {
        // Rotate body-frame delta to world frame
        float cos_h = cosf(heading);
        float sin_h = sinf(heading);

        float dx_world = vla_traj.delta_x[i] * cos_h - vla_traj.delta_y[i] * sin_h;
        float dy_world = vla_traj.delta_x[i] * sin_h + vla_traj.delta_y[i] * cos_h;

        // Accumulate position
        x += dx_world;
        y += dy_world;
        heading += vla_traj.delta_heading[i];

        // Normalize heading to [-π, π]
        heading = wrap_pi(heading);

        // Store world-frame trajectory point
        chassis_traj[i].x = x;
        chassis_traj[i].y = y;
        chassis_traj[i].heading = heading;
        chassis_traj[i].steering = vla_traj.steering_angle[i];
        chassis_traj[i].timestamp = i * vla_traj.step_interval;
    }
    num_points = vla_traj.num_steps;
}
```

### 2.2 Boom Inverse Kinematics

Convert bucket height to boom angle using wheel loader kinematics:

```cpp
struct BoomKinematics {
    float boom_length;          // Boom arm length [m] (param: VTP_BOOM_LEN)
    float boom_pivot_height;    // Boom pivot height from ground [m] (param: VTP_BOOM_PIVOT_H)
    float boom_angle_min;       // Minimum boom angle [rad] (param: VTP_BOOM_ANG_MIN)
    float boom_angle_max;       // Maximum boom angle [rad] (param: VTP_BOOM_ANG_MAX)
};

float computeBoomAngle(float bucket_height, const BoomKinematics& kin)
{
    // bucket_height = boom_pivot_height + boom_length * sin(boom_angle)
    // boom_angle = asin((bucket_height - boom_pivot_height) / boom_length)

    float height_delta = bucket_height - kin.boom_pivot_height;
    float sin_angle = height_delta / kin.boom_length;

    // Clamp to valid range
    sin_angle = math::constrain(sin_angle, -1.0f, 1.0f);

    float boom_angle = asinf(sin_angle);

    // Apply joint limits
    return math::constrain(boom_angle, kin.boom_angle_min, kin.boom_angle_max);
}
```

### 2.3 Decomposed Trajectory Structures

```cpp
// Chassis trajectory (world frame)
struct ChassisTrajectory {
    static constexpr int MAX_POINTS = 16;
    int num_points;
    float timestamp[MAX_POINTS];      // Time from start [s]
    float x[MAX_POINTS];              // World X [m]
    float y[MAX_POINTS];              // World Y [m]
    float heading[MAX_POINTS];        // World heading [rad]
    float steering[MAX_POINTS];       // Articulation angle [rad]
    float slip_rate[MAX_POINTS];      // Target slip rate
};

// Boom trajectory
struct BoomTrajectory {
    static constexpr int MAX_POINTS = 16;
    int num_points;
    float timestamp[MAX_POINTS];      // Time from start [s]
    float angle[MAX_POINTS];          // Boom angle [rad]
};

// Tilt trajectory
struct TiltTrajectory {
    static constexpr int MAX_POINTS = 16;
    int num_points;
    float timestamp[MAX_POINTS];      // Time from start [s]
    float angle[MAX_POINTS];          // Tilt angle [rad]
};
```

---

## 3. Trajectory Smoothing Stage (MPC-Style)

### 3.1 Kinematic Constraints (Parameters)

```cpp
// Chassis constraints
PARAM_DEFINE_FLOAT(VTP_CHS_VEL_MAX, 2.0f);      // Max chassis velocity [m/s]
PARAM_DEFINE_FLOAT(VTP_CHS_ACC_MAX, 1.0f);      // Max chassis acceleration [m/s²]
PARAM_DEFINE_FLOAT(VTP_CHS_JERK_MAX, 2.0f);     // Max chassis jerk [m/s³]
PARAM_DEFINE_FLOAT(VTP_CHS_YAW_MAX, 0.5f);      // Max yaw rate [rad/s]
PARAM_DEFINE_FLOAT(VTP_CHS_YACC_MAX, 0.5f);     // Max yaw acceleration [rad/s²]
PARAM_DEFINE_FLOAT(VTP_STR_ANG_MAX, 0.6f);      // Max steering angle [rad] (~35°)
PARAM_DEFINE_FLOAT(VTP_STR_RATE_MAX, 0.3f);     // Max steering rate [rad/s]

// Boom constraints
PARAM_DEFINE_FLOAT(VTP_BOOM_LEN, 3.0f);         // Boom length [m]
PARAM_DEFINE_FLOAT(VTP_BOOM_PIVOT_H, 1.5f);     // Boom pivot height [m]
PARAM_DEFINE_FLOAT(VTP_BOOM_ANG_MIN, -0.2f);    // Min boom angle [rad]
PARAM_DEFINE_FLOAT(VTP_BOOM_ANG_MAX, 1.2f);     // Max boom angle [rad] (~70°)
PARAM_DEFINE_FLOAT(VTP_BOOM_VEL_MAX, 0.3f);     // Max boom angular velocity [rad/s]
PARAM_DEFINE_FLOAT(VTP_BOOM_ACC_MAX, 0.5f);     // Max boom angular acceleration [rad/s²]
PARAM_DEFINE_FLOAT(VTP_BOOM_JERK_MAX, 1.0f);    // Max boom jerk [rad/s³]

// Tilt constraints
PARAM_DEFINE_FLOAT(VTP_TILT_ANG_MIN, -0.5f);    // Min tilt angle [rad] (~-30°)
PARAM_DEFINE_FLOAT(VTP_TILT_ANG_MAX, 0.8f);     // Max tilt angle [rad] (~45°)
PARAM_DEFINE_FLOAT(VTP_TILT_VEL_MAX, 0.5f);     // Max tilt angular velocity [rad/s]
PARAM_DEFINE_FLOAT(VTP_TILT_ACC_MAX, 0.8f);     // Max tilt angular acceleration [rad/s²]
PARAM_DEFINE_FLOAT(VTP_TILT_JERK_MAX, 1.5f);    // Max tilt jerk [rad/s³]

// Synchronization
PARAM_DEFINE_FLOAT(VTP_SYNC_MARGIN, 0.1f);      // Synchronization time margin [s]
```

### 3.2 Synchronized Time Scaling

All DOFs are scaled to arrive at the same time using velocity scaling:

```cpp
struct TrajectoryTiming {
    float chassis_time;     // Time needed for chassis at max constraints
    float boom_time;        // Time needed for boom at max constraints
    float tilt_time;        // Time needed for tilt at max constraints
    float sync_time;        // Synchronized time (max of all)
    float scale_factor;     // Time scale factor (>= 1.0)
};

TrajectoryTiming computeSynchronizedTiming(
    const ChassisTrajectory& chassis,
    const BoomTrajectory& boom,
    const TiltTrajectory& tilt,
    const KinematicConstraints& constraints)
{
    TrajectoryTiming timing;

    // Compute minimum time for each channel
    timing.chassis_time = computeMinimumTime(chassis, constraints.chassis);
    timing.boom_time = computeMinimumTime(boom, constraints.boom);
    timing.tilt_time = computeMinimumTime(tilt, constraints.tilt);

    // Synchronized time is the maximum (slowest channel)
    timing.sync_time = fmaxf(timing.chassis_time,
                             fmaxf(timing.boom_time, timing.tilt_time));

    // Add safety margin
    timing.sync_time += _param_vtp_sync_margin.get();

    // Scale factor for VLA trajectory (VLA assumes fixed 1.6s horizon)
    float vla_horizon = chassis.num_points * 0.1f;  // 16 * 0.1s = 1.6s
    timing.scale_factor = timing.sync_time / vla_horizon;

    return timing;
}

float computeMinimumTime(const ChassisTrajectory& traj,
                         const ChassisConstraints& c)
{
    float total_distance = 0.0f;
    float max_segment_time = 0.0f;

    for (int i = 1; i < traj.num_points; i++) {
        float dx = traj.x[i] - traj.x[i-1];
        float dy = traj.y[i] - traj.y[i-1];
        float dist = sqrtf(dx*dx + dy*dy);
        total_distance += dist;

        // Time limited by velocity
        float time_vel = dist / c.max_velocity;

        // Time limited by acceleration (simplified)
        float time_acc = 2.0f * sqrtf(dist / c.max_acceleration);

        max_segment_time = fmaxf(max_segment_time, fmaxf(time_vel, time_acc));
    }

    return max_segment_time * traj.num_points;
}
```

### 3.3 MPC-Style Trajectory Optimization

The MPC formulation optimizes trajectory smoothness while respecting constraints:

```cpp
/**
 * MPC Cost Function:
 *
 * J = Σ [ w_pos * ||p - p_ref||² +           // Position tracking
 *        w_vel * ||v||² +                     // Velocity minimization
 *        w_acc * ||a||² +                     // Acceleration smoothness
 *        w_jerk * ||j||² ]                    // Jerk minimization
 *
 * Subject to:
 *   |v| <= v_max
 *   |a| <= a_max
 *   |j| <= j_max
 *   angle_min <= angle <= angle_max
 */

class TrajectoryOptimizer {
public:
    struct OptimizationWeights {
        float position;     // Position tracking weight (VTP_W_POS)
        float velocity;     // Velocity weight (VTP_W_VEL)
        float acceleration; // Acceleration weight (VTP_W_ACC)
        float jerk;         // Jerk weight (VTP_W_JERK)
    };

    /**
     * Optimize single-DOF trajectory using iterative refinement
     * Uses gradient descent with constraint projection
     */
    void optimizeTrajectory(
        const float* input_points,      // Raw trajectory points
        int num_input_points,
        float input_dt,                  // Input timestep (0.1s)
        float* output_points,            // Smoothed trajectory (50Hz)
        float* output_velocities,
        float* output_accelerations,
        int& num_output_points,
        float output_dt,                 // Output timestep (0.02s)
        const Constraints1D& constraints,
        const OptimizationWeights& weights,
        float time_scale);

private:
    static constexpr int MAX_ITERATIONS = 10;
    static constexpr float CONVERGENCE_THRESHOLD = 1e-4f;

    void projectToConstraints(float* traj, int n, const Constraints1D& c);
    float computeCost(const float* traj, int n, const OptimizationWeights& w);
    void computeGradient(const float* traj, float* grad, int n, const OptimizationWeights& w);
};
```

### 3.4 Jerk-Limited Trajectory Generation

Using trapezoidal velocity profile with jerk limiting:

```cpp
class JerkLimitedGenerator {
public:
    struct State {
        float position;
        float velocity;
        float acceleration;
    };

    /**
     * Generate smooth trajectory from waypoints
     * Interpolates 100ms VLA points to 20ms control points
     */
    void generateSmoothTrajectory(
        const float* waypoints,
        int num_waypoints,
        float waypoint_dt,           // 0.1s (VLA interval)
        State initial_state,
        float* positions,            // Output positions (50Hz)
        float* velocities,           // Output velocities
        float* accelerations,        // Output accelerations
        int& num_output,
        float output_dt,             // 0.02s (50Hz)
        float max_vel,
        float max_acc,
        float max_jerk,
        float time_scale)
    {
        // Scale constraints by time_scale for synchronized motion
        float scaled_max_vel = max_vel / time_scale;
        float scaled_max_acc = max_acc / (time_scale * time_scale);
        float scaled_max_jerk = max_jerk / (time_scale * time_scale * time_scale);

        // Interpolation ratio: 100ms / 20ms = 5 output points per input
        int interp_ratio = static_cast<int>(waypoint_dt / output_dt + 0.5f);
        num_output = (num_waypoints - 1) * interp_ratio + 1;

        State state = initial_state;
        int output_idx = 0;

        for (int i = 0; i < num_waypoints - 1; i++) {
            float target = waypoints[i + 1];
            float segment_time = waypoint_dt * time_scale;

            // Generate jerk-limited profile for this segment
            for (int j = 0; j < interp_ratio; j++) {
                float t = j * output_dt;

                // Compute desired acceleration to reach target
                float pos_error = target - state.position;
                float desired_vel = computeDesiredVelocity(pos_error, segment_time - t,
                                                          scaled_max_vel);
                float desired_acc = (desired_vel - state.velocity) / output_dt;

                // Apply jerk limit
                float jerk = math::constrain(
                    (desired_acc - state.acceleration) / output_dt,
                    -scaled_max_jerk, scaled_max_jerk);

                // Update state
                state.acceleration += jerk * output_dt;
                state.acceleration = math::constrain(state.acceleration,
                                                     -scaled_max_acc, scaled_max_acc);
                state.velocity += state.acceleration * output_dt;
                state.velocity = math::constrain(state.velocity,
                                                 -scaled_max_vel, scaled_max_vel);
                state.position += state.velocity * output_dt;

                // Store output
                positions[output_idx] = state.position;
                velocities[output_idx] = state.velocity;
                accelerations[output_idx] = state.acceleration;
                output_idx++;
            }
        }
    }

private:
    float computeDesiredVelocity(float pos_error, float time_remaining, float max_vel)
    {
        if (time_remaining <= 0.0f) return 0.0f;

        // Simple proportional approach with velocity limit
        float desired = pos_error / time_remaining;
        return math::constrain(desired, -max_vel, max_vel);
    }
};
```

---

## 4. Output Stage

### 4.1 Smoothed Trajectory Buffer

```cpp
class SmoothedTrajectoryBuffer {
public:
    static constexpr int BUFFER_SIZE = 80;  // 80 points = 1.6s at 50Hz
    static constexpr float DT = 0.02f;      // 20ms = 50Hz

    struct TrajectoryPoint {
        float position;
        float velocity;
        float acceleration;
        float timestamp;
        bool valid;
    };

    // Separate buffers for each channel
    TrajectoryPoint chassis_x[BUFFER_SIZE];
    TrajectoryPoint chassis_y[BUFFER_SIZE];
    TrajectoryPoint chassis_heading[BUFFER_SIZE];
    TrajectoryPoint chassis_steering[BUFFER_SIZE];
    TrajectoryPoint boom[BUFFER_SIZE];
    TrajectoryPoint tilt[BUFFER_SIZE];
    float slip_rate[BUFFER_SIZE];

    int write_index;
    int read_index;
    int num_valid_points;

    /**
     * Get current setpoint and advance read index
     * Called at 50Hz
     */
    bool getNextSetpoint(
        ChassisTrajectorySetpoint& chassis_sp,
        BoomTrajectorySetpoint& boom_sp,
        TiltTrajectorySetpoint& tilt_sp);
};
```

### 4.2 Output Messages

#### 4.2.1 Enhanced ChassisTrajectorySetpoint

Add steering and slip rate fields to existing message:

```msg
# ChassisTrajectorySetpoint.msg (updated)
# Chassis trajectory setpoint for articulated wheel loader
# Published by VlaTrajectoryProcessor at 50Hz

uint64 timestamp                    # time since system start (microseconds)

# Position setpoint (world frame)
float32 x_position                  # X position in world frame [m]
float32 y_position                  # Y position in world frame [m]
float32 heading                     # Heading in world frame [rad]

# Velocity setpoint (world frame)
float32 x_velocity                  # X velocity [m/s]
float32 y_velocity                  # Y velocity [m/s]
float32 yaw_rate                    # Yaw rate [rad/s]

# Acceleration setpoint (world frame)
float32 x_acceleration              # X acceleration [m/s²]
float32 y_acceleration              # Y acceleration [m/s²]
float32 yaw_acceleration            # Yaw acceleration [rad/s²]

# Articulation control
float32 steering_angle              # Articulation angle setpoint [rad]
float32 steering_rate               # Articulation rate setpoint [rad/s]

# Traction control feedforward
float32 target_slip_rate            # Target slip rate for traction control [0.0-1.0]

# Trajectory info
float32 trajectory_progress         # Progress along trajectory [0.0-1.0]
uint32 trajectory_sequence          # Trajectory sequence ID

# Validity
bool valid                          # Setpoint is valid
```

#### 4.2.2 Enhanced BoomTrajectorySetpoint

```msg
# BoomTrajectorySetpoint.msg (updated)
# Boom trajectory setpoint for wheel loader lift arm
# Published by VlaTrajectoryProcessor at 50Hz

uint64 timestamp                    # time since system start (microseconds)

# Angular setpoint
float32 angle                       # Boom angle setpoint [rad]
float32 angular_velocity            # Angular velocity setpoint [rad/s]
float32 angular_acceleration        # Angular acceleration setpoint [rad/s²]

# Derived Cartesian (for monitoring)
float32 bucket_height               # Corresponding bucket height [m]

# Motion constraints (from VTP parameters)
float32 max_velocity                # Max angular velocity limit [rad/s]
float32 max_acceleration            # Max angular acceleration limit [rad/s²]

# Control mode
uint8 control_mode
uint8 MODE_TRAJECTORY = 0           # Following trajectory
uint8 MODE_POSITION = 1             # Position hold
uint8 MODE_VELOCITY = 2             # Velocity control
uint8 MODE_STOP = 3                 # Controlled stop

# Trajectory info
float32 trajectory_progress         # Progress [0.0-1.0]
uint32 trajectory_sequence          # Sequence ID

# Validity
bool valid
```

#### 4.2.3 New TiltTrajectorySetpoint

```msg
# TiltTrajectorySetpoint.msg
# Tilt (bucket curl) trajectory setpoint for wheel loader
# Published by VlaTrajectoryProcessor at 50Hz

uint64 timestamp                    # time since system start (microseconds)

# Angular setpoint
float32 angle                       # Tilt angle setpoint [rad]
float32 angular_velocity            # Angular velocity setpoint [rad/s]
float32 angular_acceleration        # Angular acceleration setpoint [rad/s²]

# Motion constraints
float32 max_velocity                # Max angular velocity limit [rad/s]
float32 max_acceleration            # Max angular acceleration limit [rad/s²]

# Control mode
uint8 control_mode
uint8 MODE_TRAJECTORY = 0           # Following trajectory
uint8 MODE_POSITION = 1             # Position hold
uint8 MODE_VELOCITY = 2             # Velocity control
uint8 MODE_STOP = 3                 # Controlled stop

# Trajectory info
float32 trajectory_progress         # Progress [0.0-1.0]
uint32 trajectory_sequence          # Sequence ID

# Validity
bool valid
```

---

## 5. Module Implementation

### 5.1 Class Structure

```cpp
/**
 * @file VlaTrajectoryProcessor.hpp
 *
 * VLA Trajectory Processor module
 * Decomposes 7-DOF VLA trajectory into chassis, boom, tilt channels
 * Applies MPC-style smoothing with kinematic constraints
 * Publishes synchronized 50Hz setpoints
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vla_trajectory.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/chassis_trajectory_setpoint.h>
#include <uORB/topics/boom_trajectory_setpoint.h>
#include <uORB/topics/tilt_trajectory_setpoint.h>
#include <lib/perf/perf_counter.h>

class VlaTrajectoryProcessor : public ModuleBase<VlaTrajectoryProcessor>,
                                public ModuleParams
{
public:
    VlaTrajectoryProcessor();
    ~VlaTrajectoryProcessor() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;

private:
    // === Subscriptions ===
    uORB::Subscription _sub_vla_trajectory{ORB_ID(vla_trajectory)};
    uORB::Subscription _sub_vehicle_local_position{ORB_ID(vehicle_local_position)};
    uORB::Subscription _sub_vehicle_attitude{ORB_ID(vehicle_attitude)};

    // === Publications ===
    uORB::Publication<chassis_trajectory_setpoint_s> _pub_chassis_setpoint{ORB_ID(chassis_trajectory_setpoint)};
    uORB::Publication<boom_trajectory_setpoint_s> _pub_boom_setpoint{ORB_ID(boom_trajectory_setpoint)};
    uORB::Publication<tilt_trajectory_setpoint_s> _pub_tilt_setpoint{ORB_ID(tilt_trajectory_setpoint)};

    // === Processing State ===
    enum class ProcessorState {
        IDLE,               // No valid trajectory
        PROCESSING,         // Processing new VLA trajectory
        EXECUTING           // Executing smoothed trajectory
    };
    ProcessorState _state{ProcessorState::IDLE};

    // Raw VLA trajectory
    vla_trajectory_s _vla_trajectory{};
    uint32_t _last_vla_sequence{0};

    // Decomposed trajectories
    ChassisTrajectory _chassis_traj{};
    BoomTrajectory _boom_traj{};
    TiltTrajectory _tilt_traj{};

    // Smoothed trajectory buffer
    SmoothedTrajectoryBuffer _trajectory_buffer{};

    // Trajectory generation
    JerkLimitedGenerator _jerk_generator{};
    TrajectoryOptimizer _optimizer{};

    // Current vehicle state
    VehicleState _vehicle_state{};

    // Timing
    hrt_abstime _trajectory_start_time{0};
    int _current_output_index{0};

    // Performance counters
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _processing_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": processing")};

    // === Core Methods ===
    void updateVehicleState();
    bool processNewVlaTrajectory();
    void decomposeTrajectory();
    void smoothTrajectories();
    void publishSetpoints();

    // === Parameters ===
    DEFINE_PARAMETERS(
        // Chassis constraints
        (ParamFloat<px4::params::VTP_CHS_VEL_MAX>) _param_vtp_chs_vel_max,
        (ParamFloat<px4::params::VTP_CHS_ACC_MAX>) _param_vtp_chs_acc_max,
        (ParamFloat<px4::params::VTP_CHS_JERK_MAX>) _param_vtp_chs_jerk_max,
        (ParamFloat<px4::params::VTP_CHS_YAW_MAX>) _param_vtp_chs_yaw_max,
        (ParamFloat<px4::params::VTP_CHS_YACC_MAX>) _param_vtp_chs_yacc_max,
        (ParamFloat<px4::params::VTP_STR_ANG_MAX>) _param_vtp_str_ang_max,
        (ParamFloat<px4::params::VTP_STR_RATE_MAX>) _param_vtp_str_rate_max,

        // Boom constraints
        (ParamFloat<px4::params::VTP_BOOM_LEN>) _param_vtp_boom_len,
        (ParamFloat<px4::params::VTP_BOOM_PIVOT_H>) _param_vtp_boom_pivot_h,
        (ParamFloat<px4::params::VTP_BOOM_ANG_MIN>) _param_vtp_boom_ang_min,
        (ParamFloat<px4::params::VTP_BOOM_ANG_MAX>) _param_vtp_boom_ang_max,
        (ParamFloat<px4::params::VTP_BOOM_VEL_MAX>) _param_vtp_boom_vel_max,
        (ParamFloat<px4::params::VTP_BOOM_ACC_MAX>) _param_vtp_boom_acc_max,
        (ParamFloat<px4::params::VTP_BOOM_JERK_MAX>) _param_vtp_boom_jerk_max,

        // Tilt constraints
        (ParamFloat<px4::params::VTP_TILT_ANG_MIN>) _param_vtp_tilt_ang_min,
        (ParamFloat<px4::params::VTP_TILT_ANG_MAX>) _param_vtp_tilt_ang_max,
        (ParamFloat<px4::params::VTP_TILT_VEL_MAX>) _param_vtp_tilt_vel_max,
        (ParamFloat<px4::params::VTP_TILT_ACC_MAX>) _param_vtp_tilt_acc_max,
        (ParamFloat<px4::params::VTP_TILT_JERK_MAX>) _param_vtp_tilt_jerk_max,

        // Synchronization
        (ParamFloat<px4::params::VTP_SYNC_MARGIN>) _param_vtp_sync_margin,

        // MPC weights
        (ParamFloat<px4::params::VTP_W_POS>) _param_vtp_w_pos,
        (ParamFloat<px4::params::VTP_W_VEL>) _param_vtp_w_vel,
        (ParamFloat<px4::params::VTP_W_ACC>) _param_vtp_w_acc,
        (ParamFloat<px4::params::VTP_W_JERK>) _param_vtp_w_jerk
    )
};
```

### 5.2 Main Run Loop

```cpp
void VlaTrajectoryProcessor::run()
{
    // Run at 50Hz
    px4_usleep(20000); // Initial delay

    while (!should_exit()) {
        perf_begin(_loop_perf);

        // Update vehicle state from EKF
        updateVehicleState();

        // Check for new VLA trajectory
        if (_sub_vla_trajectory.updated()) {
            _sub_vla_trajectory.copy(&_vla_trajectory);

            // Check if this is a new trajectory
            if (_vla_trajectory.sequence_id != _last_vla_sequence &&
                _vla_trajectory.valid) {

                perf_begin(_processing_perf);
                processNewVlaTrajectory();
                perf_end(_processing_perf);

                _last_vla_sequence = _vla_trajectory.sequence_id;
            }
        }

        // Publish setpoints at 50Hz
        publishSetpoints();

        perf_end(_loop_perf);

        // Sleep to maintain 50Hz rate
        px4_usleep(20000);
    }
}

bool VlaTrajectoryProcessor::processNewVlaTrajectory()
{
    // Step 1: Decompose 7-DOF trajectory into channels
    decomposeTrajectory();

    // Step 2: Compute synchronized timing
    TrajectoryTiming timing = computeSynchronizedTiming(
        _chassis_traj, _boom_traj, _tilt_traj, getConstraints());

    // Step 3: Apply MPC-style smoothing with time scaling
    smoothTrajectories(timing.scale_factor);

    // Step 4: Reset execution state
    _trajectory_start_time = hrt_absolute_time();
    _current_output_index = 0;
    _state = ProcessorState::EXECUTING;

    return true;
}
```

---

## 6. Integration with Flight Mode Manager

### 6.1 Data Flow

```
VlaTrajectoryProcessor                    Flight Mode Manager
        │                                         │
        │  chassis_trajectory_setpoint (50Hz)     │
        ├────────────────────────────────────────►│
        │                                         │
        │  boom_trajectory_setpoint (50Hz)        │  FlightTaskAutoVLA
        ├────────────────────────────────────────►│  (or dedicated task)
        │                                         │
        │  tilt_trajectory_setpoint (50Hz)        │
        ├────────────────────────────────────────►│
        │                                         │
                                                  │
                                                  ▼
                                         ┌───────────────┐
                                         │ Rate Control  │
                                         │   (50Hz)      │
                                         └───────┬───────┘
                                                 │
                    ┌────────────────────────────┼────────────────────────────┐
                    ▼                            ▼                            ▼
           ┌───────────────┐            ┌───────────────┐            ┌───────────────┐
           │   Drivetrain  │            │     Boom      │            │     Tilt      │
           │   Actuators   │            │   Actuator    │            │   Actuator    │
           └───────────────┘            └───────────────┘            └───────────────┘
```

### 6.2 FlightTask Subscription

The FlightTaskAutoVLA subscribes to the setpoints:

```cpp
// In FlightTaskAutoVLA
uORB::Subscription _sub_chassis_setpoint{ORB_ID(chassis_trajectory_setpoint)};
uORB::Subscription _sub_boom_setpoint{ORB_ID(boom_trajectory_setpoint)};
uORB::Subscription _sub_tilt_setpoint{ORB_ID(tilt_trajectory_setpoint)};

void FlightTaskAutoVLA::update()
{
    // Get pre-smoothed setpoints from VlaTrajectoryProcessor
    if (_sub_chassis_setpoint.update(&_chassis_sp)) {
        // Apply to vehicle control
        _position_setpoint(0) = _chassis_sp.x_position;
        _position_setpoint(1) = _chassis_sp.y_position;
        _velocity_setpoint(0) = _chassis_sp.x_velocity;
        _velocity_setpoint(1) = _chassis_sp.y_velocity;
        _yaw_setpoint = _chassis_sp.heading;
        _yawspeed_setpoint = _chassis_sp.yaw_rate;

        // Forward slip rate to traction control
        _traction_slip_feedforward = _chassis_sp.target_slip_rate;
    }

    // Similar for boom and tilt...
}
```

---

## 7. Traction Control Integration

### 7.1 Slip Rate Feedforward

The `target_slip_rate` from VLA is passed to the traction control system as a feedforward term:

```cpp
// In Traction Control Module
void TractionControl::update()
{
    // Get slip rate feedforward from trajectory processor
    if (_sub_chassis_setpoint.update(&_chassis_sp)) {
        _target_slip_rate = _chassis_sp.target_slip_rate;
    }

    // Modify slip controller target based on feedforward
    float slip_target = _param_tc_target_slip.get();

    if (_target_slip_rate > 0.0f) {
        // VLA has provided expected slip - use it as feedforward
        // Blend between default target and VLA prediction
        float blend_factor = 0.7f;  // 70% VLA, 30% default
        slip_target = blend_factor * _target_slip_rate +
                      (1.0f - blend_factor) * slip_target;
    }

    // Use slip_target in PID controller
    float slip_error = slip_target - _measured_slip;
    // ... rest of control loop
}
```

---

## 8. State Machine

```
                    ┌──────────────────────────────────────────┐
                    │                  IDLE                    │
                    │  - No valid VLA trajectory               │
                    │  - Publish hold setpoints                │
                    └──────────────────┬───────────────────────┘
                                       │
                                       │ New VLA trajectory received
                                       │ (valid = true)
                                       ▼
                    ┌──────────────────────────────────────────┐
                    │               PROCESSING                 │
                    │  - Decompose trajectory                  │
                    │  - Compute synchronized timing           │
                    │  - Generate smoothed trajectories        │
                    │  - Fill trajectory buffer                │
                    └──────────────────┬───────────────────────┘
                                       │
                                       │ Processing complete
                                       ▼
                    ┌──────────────────────────────────────────┐
          ┌────────│               EXECUTING                   │◄─────────┐
          │        │  - Publish setpoints from buffer (50Hz)   │          │
          │        │  - Advance buffer read index              │          │
          │        │  - Monitor trajectory progress            │          │
          │        └──────────────────┬────────────────────────┘          │
          │                           │                                   │
          │     New VLA trajectory    │ Trajectory complete               │
          │     (sequence changed)    │ (buffer empty)                    │
          │                           ▼                                   │
          │        ┌──────────────────────────────────────────┐          │
          │        │            TRAJECTORY_COMPLETE            │          │
          │        │  - Hold last setpoint                     │──────────┘
          └───────►│  - Wait for new trajectory                │   Timeout
                   └──────────────────────────────────────────┘   or new traj
```

---

## 9. File Structure

```
src/modules/vla_trajectory_processor/
├── CMakeLists.txt
├── VlaTrajectoryProcessor.hpp
├── VlaTrajectoryProcessor.cpp
├── vla_trajectory_processor_main.cpp
├── vla_trajectory_processor_params.c
├── trajectory_decomposer.hpp
├── trajectory_decomposer.cpp
├── trajectory_smoother.hpp
├── trajectory_smoother.cpp
├── boom_kinematics.hpp
├── boom_kinematics.cpp
└── Kconfig

msg/
├── VlaTrajectory.msg                    # New: 7-DOF VLA trajectory input
├── ChassisTrajectorySetpoint.msg        # Updated: Add steering, slip
├── BoomTrajectorySetpoint.msg           # Updated: Add bucket_height
└── TiltTrajectorySetpoint.msg           # New: Tilt setpoint
```

---

## 10. Summary

| Component | Frequency | Description |
|-----------|-----------|-------------|
| VLA Model | 10 Hz | Outputs 16-step, 7-DOF trajectory |
| VlaTrajectoryProcessor | 50 Hz | Decomposes, smooths, publishes |
| ChassisTrajectorySetpoint | 50 Hz | x, y, heading, steering, slip |
| BoomTrajectorySetpoint | 50 Hz | boom_angle, angular_velocity |
| TiltTrajectorySetpoint | 50 Hz | tilt_angle, angular_velocity |
| Flight Mode Manager | 50 Hz | Receives setpoints, generates actuator commands |
| Controllers | 50 Hz | Chassis, Boom, Tilt closed-loop control |

### Key Design Decisions

1. **Decomposition First**: Raw 7-DOF trajectory is split before smoothing
2. **Scaled Velocity Synchronization**: All DOFs slow down proportionally to arrive together
3. **50Hz Direct Publication**: Module runs at 50Hz, no interpolation in Flight Mode Manager
4. **MPC-Style Optimization**: Jerk-limited trajectory with constraint projection
5. **Slip Feedforward**: VLA slip prediction passed to traction control as feedforward
6. **Body Frame Input**: VLA outputs deltas in body frame, converted to world frame internally
