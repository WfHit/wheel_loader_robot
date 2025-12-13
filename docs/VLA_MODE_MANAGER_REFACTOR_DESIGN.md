# VLA Mode Manager Refactor Design

## Overview

This document describes the comprehensive refactoring of the VLA trajectory processing pipeline and renaming of `flight_mode_manager` to `mode_manager`.

## 1. Architecture Summary

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              VLA Model (External)                            │
│                    Outputs: 7-DOF × 16 steps × 100ms                        │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Autonomy Module                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     VlaTrajectoryTask (~10Hz)                        │   │
│  │  • Subscribes: vla_trajectory                                        │   │
│  │  • Body→World transform                                              │   │
│  │  • MPC-style smoothing with kinematic constraints                   │   │
│  │  • Publishes: VlaSetpointTriplet (previous/current/next)            │   │
│  │  • Note: bucket_height passed through, IK done in ModeVLA           │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                          VlaSetpointTriplet (10Hz)
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Mode Manager (renamed)                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        ModeVLA (50Hz)                                │   │
│  │  • Subscribes: VlaSetpointTriplet                                   │   │
│  │  • Boom IK (bucket_height → boom_angle)                             │   │
│  │  • Interpolates triplet to 50Hz setpoints                           │   │
│  │  • Publishes:                                                        │   │
│  │    - ChassisControlSetpoint (50Hz)                                  │   │
│  │    - BoomControlSetpoint (50Hz)                                     │   │
│  │    - TiltControlSetpoint (50Hz)                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
              ┌───────────────────────┼───────────────────────┐
              ▼                       ▼                       ▼
┌──────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ ChassisTrajectory    │  │ BoomControl          │  │ TiltControl        │
│ Follower (MPC)       │  │                      │  │ (uses Tilt msg)      │
└──────────────────────┘  └──────────────────────┘  └──────────────────────┘
```

---

## 2. Module Rename: flight_mode_manager → mode_manager

### 2.1 Directory Structure Change

**Before:**
```
src/modules/flight_mode_manager/
├── FlightModeManager.cpp
├── FlightModeManager.hpp
├── flight_mode_manager_params.c
├── CMakeLists.txt
├── Kconfig
├── generate_flight_tasks.py
├── Templates/
│   ├── FlightTasks_generated.cpp.em
│   └── FlightTasks_generated.hpp.em
└── tasks/
    ├── FlightTask/
    ├── Auto/
    ├── AutoFollowTarget/
    ├── AutoVLAEndEffector/  (DELETE)
    ├── Descend/
    ├── Failsafe/
    ├── ManualAcceleration/
    ├── ManualAccelerationSlow/
    ├── ManualAltitude/
    ├── ManualAltitudeSmoothVel/
    ├── ManualPosition/
    ├── Orbit/
    ├── Transition/
    └── Utility/
```

**After:**
```
src/modules/mode_manager/
├── ModeManager.cpp
├── ModeManager.hpp
├── mode_manager_params.c
├── CMakeLists.txt
├── Kconfig
├── generate_modes.py
├── Templates/
│   ├── Modes_generated.cpp.em
│   └── Modes_generated.hpp.em
└── modes/
    ├── Mode/                    (base class)
    ├── ModeAuto/
    ├── ModeAutoFollowTarget/
    ├── ModeVLA/                 (NEW - replaces AutoVLAEndEffector)
    ├── ModeDescend/
    ├── ModeFailsafe/
    ├── ModeManualAcceleration/
    ├── ModeManualAccelerationSlow/
    ├── ModeManualAltitude/
    ├── ModeManualAltitudeSmoothVel/
    ├── ModeManualPosition/
    ├── ModeOrbit/
    ├── ModeTransition/
    └── Utility/
```

### 2.2 Class Naming Changes

| Old Name | New Name |
|----------|----------|
| `FlightModeManager` | `ModeManager` |
| `FlightTask` | `Mode` |
| `FlightTaskAuto` | `ModeAuto` |
| `FlightTaskAutoFollowTarget` | `ModeAutoFollowTarget` |
| `FlightTaskAutoVLAEndEffector` | `ModeVLA` (redesigned) |
| `FlightTaskDescend` | `ModeDescend` |
| `FlightTaskFailsafe` | `ModeFailsafe` |
| `FlightTaskManualAcceleration` | `ModeManualAcceleration` |
| `FlightTaskManualAccelerationSlow` | `ModeManualAccelerationSlow` |
| `FlightTaskManualAltitude` | `ModeManualAltitude` |
| `FlightTaskManualAltitudeSmoothVel` | `ModeManualAltitudeSmoothVel` |
| `FlightTaskManualPosition` | `ModeManualPosition` |
| `FlightTaskOrbit` | `ModeOrbit` |
| `FlightTaskTransition` | `ModeTransition` |
| `FlightTaskIndex` | `ModeIndex` |
| `FlightTaskError` | `ModeError` |
| `TaskUnion` | `ModeUnion` |

### 2.3 Enum Changes

```cpp
// Old
enum class FlightTaskIndex : int {
    None = -1,
    Auto,
    AutoFollowTarget,
    AutoVLAEndEffector,  // REMOVE
    Descend,
    Failsafe,
    ManualAcceleration,
    ManualAccelerationSlow,
    ManualAltitude,
    ManualAltitudeSmoothVel,
    ManualPosition,
    Orbit,
    Transition,
    Count
};

// New
enum class ModeIndex : int {
    None = -1,
    Auto,
    AutoFollowTarget,
    VLA,                  // NEW (simplified name)
    Descend,
    Failsafe,
    ManualAcceleration,
    ManualAccelerationSlow,
    ManualAltitude,
    ManualAltitudeSmoothVel,
    ManualPosition,
    Orbit,
    Transition,
    Count
};
```

---

## 3. New Messages

### 3.1 VlaSetpointTriplet.msg (NEW)

```
# VLA Setpoint Triplet for wheel loader trajectory control
# Published by VlaTrajectoryTask at ~10Hz
# Subscribed by ModeVLA in mode_manager

uint64 timestamp                    # time since system start (microseconds)

# Triplet of setpoints (previous → current → next)
VlaSetpoint previous
VlaSetpoint current
VlaSetpoint next

# Trajectory metadata
uint32 trajectory_sequence          # Trajectory sequence ID
float32 vla_confidence              # VLA model confidence [0.0-1.0]
bool valid                          # Triplet is valid for execution
```

### 3.2 VlaSetpoint.msg (NEW - nested message)

```
# Single VLA setpoint containing full state for chassis, boom, and tilt
# Used as nested message in VlaSetpointTriplet

uint64 timestamp                    # time since system start (microseconds)

# === Chassis State (World Frame) ===
float32 chassis_x                   # X position [m]
float32 chassis_y                   # Y position [m]
float32 chassis_heading             # Heading [rad]
float32 chassis_vx                  # X velocity [m/s]
float32 chassis_vy                  # Y velocity [m/s]
float32 chassis_yaw_rate            # Yaw rate [rad/s]
float32 chassis_ax                  # X acceleration [m/s²]
float32 chassis_ay                  # Y acceleration [m/s²]
float32 chassis_yaw_acc             # Yaw acceleration [rad/s²]
float32 steering_angle              # Articulation angle [rad]
float32 steering_rate               # Articulation rate [rad/s]
float32 target_slip_rate            # Target slip rate [0.0-1.0]

# === Boom State ===
float32 boom_angle                  # Boom angle [rad]
float32 boom_angular_velocity       # Angular velocity [rad/s]
float32 boom_angular_acceleration   # Angular acceleration [rad/s²]

# === Tilt State ===
float32 tilt_angle                  # Tilt angle [rad]
float32 tilt_angular_velocity       # Angular velocity [rad/s]
float32 tilt_angular_acceleration   # Angular acceleration [rad/s²]

# === Constraints (optional, NaN = use defaults) ===
float32 max_chassis_velocity        # Max chassis velocity [m/s]
float32 max_chassis_acceleration    # Max chassis acceleration [m/s²]
float32 max_boom_velocity           # Max boom angular velocity [rad/s]
float32 max_boom_acceleration       # Max boom angular acceleration [rad/s²]
float32 max_tilt_velocity           # Max tilt angular velocity [rad/s]
float32 max_tilt_acceleration       # Max tilt angular acceleration [rad/s²]

# === Validity ===
bool valid                          # Setpoint is valid
float32 progress                    # Progress along trajectory [0.0-1.0]
```

### 3.3 Updated ChassisTrajectorySetpoint.msg

```
# Chassis trajectory setpoint in WORLD coordinates
# Published by ModeVLA at 50Hz
# Subscribed by chassis trajectory follower

uint64 timestamp                    # time since system start (microseconds)

# Position setpoint (world frame)
float32 x                           # X position [m]
float32 y                           # Y position [m]
float32 heading                     # Heading [rad]

# Velocity setpoint (world frame)
float32 vx                          # X velocity [m/s]
float32 vy                          # Y velocity [m/s]
float32 yaw_rate                    # Yaw rate [rad/s]

# Acceleration setpoint (world frame)
float32 ax                          # X acceleration [m/s²]
float32 ay                          # Y acceleration [m/s²]
float32 yaw_acceleration            # Yaw acceleration [rad/s²]

# Steering control
float32 steering_angle              # Articulation angle [rad]
float32 steering_rate               # Articulation rate [rad/s]

# Traction control feedforward
float32 target_slip_rate            # Target slip rate [0.0-1.0]

# Trajectory info
uint32 sequence                     # Trajectory sequence ID
bool valid                          # Setpoint is valid
```

### 3.4 Updated BoomTrajectorySetpoint.msg

```
# Boom trajectory setpoint
# Published by ModeVLA at 50Hz
# Subscribed by boom control module

uint64 timestamp                    # time since system start (microseconds)

# Angular setpoint
float32 angle                       # Target boom angle [rad]
float32 angular_velocity            # Target angular velocity [rad/s]
float32 angular_acceleration        # Target angular acceleration [rad/s²]

# Trajectory info
uint32 sequence                     # Trajectory sequence ID
bool valid                          # Setpoint is valid
```

### 3.5 Updated TiltTrajectorySetpoint.msg

```
# Tilt (bucket curl) trajectory setpoint for wheel loader
# Published by ModeVLA at 50Hz
# Subscribed by bucket/tilt controller

uint64 timestamp                    # time since system start (microseconds)

# Angular setpoint
float32 angle                       # Tilt angle [rad]
float32 angular_velocity            # Angular velocity [rad/s]
float32 angular_acceleration        # Angular acceleration [rad/s²]

# Trajectory info
uint32 sequence                     # Trajectory sequence ID
bool valid                          # Setpoint is valid
```

---

## 4. Message Cleanup

### 4.1 Delete Messages
- `bucket_trajectory_setpoint.msg` → Use `tilt_trajectory_setpoint.msg` instead
- `vla_end_effector_setpoint_triplet.msg` → Replaced by `VlaSetpointTriplet.msg`

### 4.2 Fields Removed from ChassisTrajectorySetpoint
- `x_position` → renamed to `x`
- `y_position` → renamed to `y`
- `x_velocity` → renamed to `vx`
- `y_velocity` → renamed to `vy`
- `x_acceleration` → renamed to `ax`
- `y_acceleration` → renamed to `ay`
- `trajectory_progress` → removed (use external tracking)
- `trajectory_sequence` → renamed to `sequence`

### 4.3 Fields Removed from BoomTrajectorySetpoint
- `bucket_height` → removed (IK is done upstream)
- `max_velocity` → removed (use parameters)
- `max_acceleration` → removed (use parameters)
- `control_mode` → removed (always trajectory mode)
- `trajectory_progress` → removed
- `trajectory_sequence` → renamed to `sequence`

### 4.4 Fields Removed from TiltTrajectorySetpoint
- `max_velocity` → removed (use parameters)
- `max_acceleration` → removed (use parameters)
- `control_mode` → removed (always trajectory mode)
- `trajectory_progress` → removed
- `trajectory_sequence` → renamed to `sequence`

---

## 5. VlaTrajectoryTask Changes

### 5.1 Rename Method
```cpp
// Old
void publishTrajectorySetpoints();

// New
void publishVlaSetpointTriplet();
```

### 5.2 New Publication
```cpp
// Old - direct 50Hz setpoints
uORB::Publication<chassis_trajectory_setpoint_s> _pub_chassis_setpoint;
uORB::Publication<boom_trajectory_setpoint_s> _pub_boom_setpoint;
uORB::Publication<tilt_trajectory_setpoint_s> _pub_tilt_setpoint;

// New - 10Hz triplet
uORB::Publication<vla_setpoint_triplet_s> _pub_vla_setpoint_triplet;
```

### 5.3 Output Change
- Before: Publishes 50Hz interpolated setpoints directly
- After: Publishes 10Hz smoothed triplet (previous/current/next waypoints)
- Interpolation to 50Hz moved to ModeVLA

---

## 6. ModeVLA Implementation

### 6.1 Purpose
Receives `VlaSetpointTriplet` from Autonomy and interpolates to 50Hz setpoints for controllers.

### 6.2 Key Methods
```cpp
class ModeVLA : public Mode
{
public:
    ModeVLA();
    ~ModeVLA() = default;

    bool activate(const trajectory_setpoint_s &last_setpoint) override;
    void reActivate() override;
    bool updateInitialize() override;
    bool update() override;

protected:
    // Interpolate triplet to current time
    void interpolateSetpoints(float dt);

    // Publish 50Hz setpoints
    void publishChassisSetpoint();
    void publishBoomSetpoint();
    void publishTiltSetpoint();

private:
    // Subscription
    uORB::Subscription _sub_vla_setpoint_triplet{ORB_ID(vla_setpoint_triplet)};

    // Publications
    uORB::Publication<chassis_trajectory_setpoint_s> _pub_chassis{ORB_ID(chassis_trajectory_setpoint)};
    uORB::Publication<boom_trajectory_setpoint_s> _pub_boom{ORB_ID(boom_trajectory_setpoint)};
    uORB::Publication<tilt_trajectory_setpoint_s> _pub_tilt{ORB_ID(tilt_trajectory_setpoint)};

    // Interpolation state
    vla_setpoint_triplet_s _triplet{};
    hrt_abstime _triplet_timestamp{0};
    float _interpolation_progress{0.0f};
};
```

### 6.3 Interpolation Logic
```cpp
void ModeVLA::interpolateSetpoints(float dt)
{
    // Progress through current→next setpoint based on time
    _interpolation_progress += dt / _triplet_interval;
    _interpolation_progress = math::constrain(_interpolation_progress, 0.0f, 1.0f);

    const auto &curr = _triplet.current;
    const auto &next = _triplet.next;
    float t = _interpolation_progress;

    // Linear interpolation (can upgrade to cubic/quintic)
    _chassis_setpoint.x = curr.chassis_x + t * (next.chassis_x - curr.chassis_x);
    _chassis_setpoint.y = curr.chassis_y + t * (next.chassis_y - curr.chassis_y);
    // ... etc for all fields
}
```

---

## 7. Files to Modify/Create

### 7.1 New Files
- `msg/VlaSetpoint.msg`
- `msg/VlaSetpointTriplet.msg`
- `src/modules/mode_manager/` (entire directory - renamed)
- `src/modules/mode_manager/modes/ModeVLA/ModeVLA.hpp`
- `src/modules/mode_manager/modes/ModeVLA/ModeVLA.cpp`
- `src/modules/mode_manager/modes/ModeVLA/CMakeLists.txt`

### 7.2 Delete Files
- `src/modules/flight_mode_manager/` (old directory)
- `src/modules/flight_mode_manager/tasks/AutoVLAEndEffector/` (replaced by ModeVLA)
- `msg/bucket_trajectory_setpoint.msg`
- `msg/vla_end_effector_setpoint_triplet.msg`

### 7.3 Modify Files
- `msg/ChassisTrajectorySetpoint.msg` (cleanup)
- `msg/BoomTrajectorySetpoint.msg` (cleanup)
- `msg/TiltTrajectorySetpoint.msg` (cleanup)
- `msg/CMakeLists.txt` (update message list)
- `src/modules/autonomy/vla_tasks/vla_trajectory_task.h` (new output)
- `src/modules/autonomy/vla_tasks/vla_trajectory_task.cpp` (new output)
- `src/modules/tilt_control/` (change subscription)
- All board configurations referencing flight_mode_manager
- CMakeLists.txt files referencing old module name

---

## 8. Implementation Order

### Phase 1: Messages
1. Create `VlaSetpoint.msg`
2. Create `VlaSetpointTriplet.msg`
3. Update `ChassisTrajectorySetpoint.msg`
4. Update `BoomTrajectorySetpoint.msg`
5. Update `TiltTrajectorySetpoint.msg`
6. Delete `bucket_trajectory_setpoint.msg`

### Phase 2: Module Rename
1. Copy `flight_mode_manager` → `mode_manager`
2. Rename all FlightTask* → Mode*
3. Update templates and code generator
4. Update CMakeLists.txt
5. Update Kconfig
6. Delete old `flight_mode_manager`

### Phase 3: ModeVLA Implementation
1. Delete old `AutoVLAEndEffector` task
2. Create new `ModeVLA` with triplet subscription
3. Implement 50Hz interpolation
4. Add to mode_manager registration

### Phase 4: VlaTrajectoryTask Update
1. Rename `publishTrajectorySetpoints` → `publishVlaSetpointTriplet`
2. Change output from direct setpoints to triplet
3. Update publication to `vla_setpoint_triplet`

### Phase 5: Controller Updates
1. Update `tilt_control` to subscribe to `tilt_trajectory_setpoint`
2. Update field name references in all controllers
3. Build and test

---

## 9. Risk Assessment

| Risk | Mitigation |
|------|------------|
| Large scope of rename | Incremental phases, thorough testing |
| Breaking existing functionality | Keep old messages as deprecated initially |
| Missing references | grep/search entire codebase |
| Board-specific linker scripts | Update symbol names |

---

## 10. Approval Required

Please review this design and confirm:
1. ✅ Module rename scope (flight_mode_manager → mode_manager)
2. ✅ Class rename scope (FlightTask* → Mode*)
3. ✅ New message structures
4. ✅ Message field cleanup
5. ✅ Implementation order

Proceed with implementation? (Y/N)
