# VLA Mode Manager Refactoring Summary

## Overview

This document summarizes the major refactoring work done to rename `flight_mode_manager` to `mode_manager`, introduce the VlaSetpointTriplet architecture, and implement ModeVLA for 50Hz trajectory interpolation with Boom IK.

## Architecture Changes

### Message Flow (New Architecture)

```
VLA Model (10Hz)
     │
     ▼
┌────────────────────────────────────────────┐
│         VlaTrajectoryTask (Automation)       │
│ - Receives VlaTrajectory                   │
│ - Decomposes 7-DOF trajectory              │
│ - Applies smoothing with kinematic limits  │
│ - Outputs: VlaSetpointTriplet (~10Hz)      │
│ - NO boom IK (bucket_height passed through)│
└────────────────────────────────────────────┘
                    │
                    ▼ vla_setpoint_triplet
┌────────────────────────────────────────────┐
│            ModeVLA (mode_manager)          │
│ - Subscribes to VlaSetpointTriplet         │
│ - Performs linear interpolation to 50Hz    │
│ - Computes Boom IK (bucket_height → angle) │
│ - Outputs: *ControlSetpoint (50Hz)         │
│   - chassis_control_setpoint               │
│   - boom_control_setpoint                  │
│   - tilt_control_setpoint                  │
└────────────────────────────────────────────┘
                    │
                    ▼ *_control_setpoint (50Hz)
┌────────────────────────────────────────────┐
│            Controllers                      │
│ - chassis_trajectory_follower              │
│ - boom_control                             │
│ - tilt_control                             │
└────────────────────────────────────────────┘
```

## New Messages Created

### VlaSetpoint.msg
Single waypoint containing full state for all DOFs:
- Chassis: x, y, heading, velocities, accelerations, steering, slip_rate
- Boom: bucket_height (meters) - IK done in ModeVLA
- Tilt: angle, angular velocity, angular acceleration

### VlaSetpointTriplet.msg
Triplet of VlaSetpoint for interpolation:
- previous, current, next VlaSetpoints
- sequence, vla_confidence, valid

### ChassisControlSetpoint.msg
50Hz control setpoint from ModeVLA:
- Position (x, y, heading)
- Velocity (vx, vy, yaw_rate)
- Acceleration (ax, ay, yaw_acc)
- Steering (angle, rate)
- target_slip_rate

### BoomControlSetpoint.msg
50Hz control setpoint from ModeVLA:
- angle (radians)
- angular_velocity
- angular_acceleration
- bucket_height (forward kinematics result)

### TiltControlSetpoint.msg
50Hz control setpoint from ModeVLA:
- angle (radians)
- angular_velocity
- angular_acceleration

## Module Rename: flight_mode_manager → mode_manager

### Files Renamed
- `src/modules/flight_mode_manager/` → `src/modules/mode_manager/`
- All `FlightTask*.hpp/.cpp` → `Mode*.hpp/.cpp`
- `FlightModeManager` class → `ModeManager`

### Config Updates
- All `CONFIG_MODULES_FLIGHT_MODE_MANAGER` → `CONFIG_MODULES_MODE_MANAGER`
- Updated in 100+ board config files (*.px4board)
- Updated in ROMFS init scripts (rc.mc_apps, rc.vtol_apps)
- Updated in posix-configs/*.config

## ModeVLA Implementation

### Location
`src/modules/mode_manager/modes/ModeVLA/`

### Files
- `ModeVLA.hpp` - Class definition with subscriptions, publications, parameters
- `ModeVLA.cpp` - Implementation with interpolation and boom IK
- `CMakeLists.txt` - Build configuration
- `mode_vla_params.c` - Boom IK parameters (VLA_BOOM_*)

### Key Methods
- `activate()` - Initialize state on mode activation
- `update()` - Main 50Hz update loop
- `isVlaTripletValid()` - Validate incoming triplet
- `computeBoomAngle(bucket_height)` - Inverse kinematics
- `interpolateSetpoints()` - Linear interpolation between waypoints
- `publishChassisSetpoint()` - Output to chassis controller
- `publishBoomSetpoint()` - Output to boom controller
- `publishTiltSetpoint()` - Output to tilt controller
- `publishHoldSetpoints()` - Hold position when no valid trajectory

### Parameters
- `VLA_BOOM_LEN` - Boom arm length (meters)
- `VLA_BOOM_PIVOT_H` - Boom pivot height above ground (meters)
- `VLA_BOOM_ANG_MIN` - Minimum boom angle (radians)
- `VLA_BOOM_ANG_MAX` - Maximum boom angle (radians)

### Navigation State
Activated when `nav_state == NAVIGATION_STATE_AUTO_VLA_TRAJECTORY`

## VlaTrajectoryTask Updates

### Changes Made
- Removed boom IK computation (moved to ModeVLA)
- Changed from publishing individual *_trajectory_setpoint messages at 50Hz
- Now publishes VlaSetpointTriplet at ~10Hz
- Added bucket height parameters (VTP_BKT_*)
- Removed boom angle parameters (VTP_BOOM_*)

### New Parameters
- `VTP_BKT_HGT_MIN` - Minimum bucket height (meters)
- `VTP_BKT_HGT_MAX` - Maximum bucket height (meters)
- `VTP_BKT_VEL_MAX` - Maximum bucket height velocity (m/s)
- `VTP_BKT_ACC_MAX` - Maximum bucket height acceleration (m/s²)
- `VTP_BKT_JERK_MAX` - Maximum bucket height jerk (m/s³)

## Controller Updates

### chassis_trajectory_follower
- Added subscription to `chassis_control_setpoint` (from ModeVLA)
- Maintains backward compatibility with `chassis_trajectory_setpoint`
- Priority: chassis_control_setpoint > chassis_trajectory_setpoint

## Backward Compatibility

### Restored Legacy Messages
The following legacy messages were restored for backward compatibility:
- `ChassisTrajectorySetpoint.msg`
- `BoomTrajectorySetpoint.msg`
- `TiltTrajectorySetpoint.msg`
- `BucketTrajectorySetpoint.msg`

These allow existing modules (VlaTrajectoryProcessor, operation_mode_manager, etc.) to continue working without changes.

## Build Notes

### Adding VLA to modes_all
In `src/modules/mode_manager/CMakeLists.txt`:
```cmake
list(APPEND modes_all
    ...
    VLA
)
```

### ModeVLA Registration
The mode is registered via:
1. Adding to `modes_all` list (auto-generates ModeIndex::VLA)
2. Adding navigation state check in `ModeManager::selectAndActivateMode()`

## Testing Considerations

1. Verify ModeVLA activates on `NAVIGATION_STATE_AUTO_VLA_TRAJECTORY`
2. Verify VlaSetpointTriplet is published at correct rate (~10Hz)
3. Verify 50Hz interpolation produces smooth control outputs
4. Verify Boom IK produces correct angles for given bucket heights
5. Verify backward compatibility with legacy trajectory setpoint messages
