# Vehicle Extensibility Guide

## Overview

This document describes how to extend the PX4-based wheel loader robot system to support additional vehicle types. The architecture has been designed with modularity in mind, allowing different vehicle configurations while sharing common components.

## Supported Vehicle Types

The system currently supports the following vehicle types defined in `VehicleStatus.msg`:

| Type | Value | Description |
|------|-------|-------------|
| `VEHICLE_TYPE_ROTARY_WING` | 1 | Multirotor/helicopter |
| `VEHICLE_TYPE_FIXED_WING` | 2 | Fixed-wing aircraft |
| `VEHICLE_TYPE_ROVER` | 3 | Ground vehicle rover |
| `VEHICLE_TYPE_WHEEL_LOADER` | 4 | Articulated wheel loader with boom and bucket |

## Architecture Overview

### Mode Manager

The `mode_manager` module is the central controller for vehicle operation modes. It:
1. Receives the current vehicle type from `vehicle_status`
2. Selects appropriate modes based on operation mode and vehicle type
3. Manages mode transitions and failsafes

### Vehicle-Specific Mode Selection

For wheel loaders, the `selectWheelLoaderMode()` function handles mode selection:

```cpp
void ModeManager::selectWheelLoaderMode()
{
    // Priority: Emergency -> VLA Auto -> Manual
    // Handles OPERATION_MODE_AUTO_VLA and OPERATION_MODE_MANUAL
}
```

### Standard Modes Integration

The `standard_modes.hpp` library provides mapping between standard modes and navigation states for each vehicle type:

```cpp
// Wheel loader specific modes
case StandardMode::WHEEL_LOADER_MANUAL:
    return vehicle_status_s::OPERATION_MODE_MANUAL;
case StandardMode::WHEEL_LOADER_VLA_AUTO:
    return vehicle_status_s::OPERATION_MODE_AUTO_VLA;
```

## Adding a New Vehicle Type

### Step 1: Define Vehicle Type Constant

Add a new constant in `msg/versioned/VehicleStatus.msg`:

```
uint8 VEHICLE_TYPE_YOUR_VEHICLE = 5  # Description of your vehicle
```

### Step 2: Create Vehicle-Specific Modes

Create new mode implementations in `src/modules/mode_manager/modes/`:

1. Create directory: `ModeYourVehicle/`
2. Implement the mode class inheriting from `Mode`:
   - `ModeYourVehicle.hpp`
   - `ModeYourVehicle.cpp`
   - `CMakeLists.txt`
   - Parameters file (optional)

### Step 3: Register the Mode

Add your mode to `src/modules/mode_manager/CMakeLists.txt`:

```cmake
list(APPEND modes_all
    ...
    YourVehicle
)
```

### Step 4: Add Mode Selection Logic

Update `ModeManager.cpp` to handle your vehicle type:

```cpp
void ModeManager::selectAndActivateMode()
{
    const bool is_your_vehicle = (_vehicle_status_sub.get().vehicle_type == 
                                   vehicle_status_s::VEHICLE_TYPE_YOUR_VEHICLE);
    
    if (is_your_vehicle) {
        selectYourVehicleMode();
        return;
    }
    // ... existing code
}

void ModeManager::selectYourVehicleMode()
{
    // Your vehicle-specific mode selection logic
}
```

### Step 5: Update Standard Modes (Optional)

Add your vehicle's standard modes to `src/lib/modes/standard_modes.hpp`:

```cpp
enum class StandardMode : uint8_t {
    // ... existing modes
    YOUR_VEHICLE_MODE1 = 110,
    YOUR_VEHICLE_MODE2 = 111,
};
```

## Wheel Loader Implementation Details

### Mode Structure

The wheel loader uses two primary modes:

1. **ModeManualWheelLoader**: Manual RC/joystick control
   - Maps stick inputs to velocity, steering, boom, and tilt commands
   - Applies smoothing and safety limits
   - Publishes control setpoints at 50Hz

2. **ModeVLA**: Vision-Language-Action autonomous mode
   - Receives VlaSetpointTriplet from automation module
   - Interpolates waypoints to 50Hz control setpoints
   - Publishes chassis, boom, and tilt control commands

### Control Setpoint Messages

| Message | Purpose |
|---------|---------|
| `ChassisControlSetpoint` | Position, velocity, steering control |
| `BoomControlSetpoint` | Bucket height control |
| `TiltControlSetpoint` | Bucket tilt/curl control |

### Subsystem Integration

```
┌──────────────────┐     ┌─────────────────────┐
│  mode_manager    │────▶│ Control Setpoints   │
│  ModeVLA or      │     │ (50Hz)              │
│  ModeManualWL    │     └─────────────────────┘
└──────────────────┘               │
                                   ▼
┌──────────────────┐     ┌─────────────────────┐
│ articulated_     │◀────│ boom_control        │
│ chassis          │     │ tilt_control        │
└──────────────────┘     └─────────────────────┘
```

## Best Practices

### 1. Use Existing Base Classes

Inherit from the `Mode` base class to get:
- Time management (`_time_stamp_last`, `_deltatime`)
- Position/velocity state (`_position`, `_velocity`, `_yaw`)
- Parameter handling infrastructure

### 2. Follow Message Naming Conventions

- Control setpoints: `<subsystem>_control_setpoint`
- Status messages: `<subsystem>_status`
- Trajectory setpoints: `<subsystem>_trajectory_setpoint`

### 3. Implement Safety Considerations

- Always check for valid input before acting
- Implement timeout handling
- Provide graceful degradation modes
- Use the `safety_manager` module for safety monitoring

### 4. Document Your Vehicle Type

Create documentation in `docs/` explaining:
- Vehicle capabilities and limitations
- Control mapping for manual mode
- Autonomous operation requirements
- Safety considerations

## Testing

1. **Unit Tests**: Test individual mode classes
2. **Integration Tests**: Test mode switching and control flow
3. **Hardware-in-Loop**: Test with simulated or real hardware
4. **Safety Tests**: Verify emergency stop and failsafe behavior

## Troubleshooting

### Mode Not Activating

1. Check `vehicle_status.vehicle_type` is set correctly
2. Verify mode is registered in CMakeLists.txt
3. Check mode selection logic in ModeManager

### Control Setpoints Not Published

1. Verify subscriptions in your mode class
2. Check for valid input data
3. Review timeout handling

### Build Errors

1. Ensure all headers are included
2. Verify CMakeLists.txt dependencies
3. Check parameter registration

## Future Enhancements

- Vehicle capability abstraction layer
- Dynamic mode registration
- Plugin-based vehicle support
- Simulation integration per vehicle type

---

*Last Updated: 2024*
*Version: 1.0*
