# Vehicle Extensibility Guide

## Overview

This document describes how to extend the PX4-based wheel loader robot system to support additional vehicle types. The architecture has been designed with modularity in mind, allowing different vehicle configurations while sharing common components.

## Architecture Overview

### Vehicle Type Strategy Pattern

The system uses a **Strategy Pattern** to encapsulate vehicle-specific behavior. Each vehicle type implements the `VehicleTypeStrategy` interface, which defines:

1. **Supported Operation Modes** - Which modes are valid for this vehicle type
2. **Mode Selection Logic** - How to select and fall back between modes
3. **Control Mode Flags** - What controllers should be active in each mode
4. **Mode Requirements** - What sensors/systems are required for each mode
5. **Failsafe Behavior** - How to handle failures
6. **Automation Task Support** - Which automation tasks are available

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      VehicleTypeStrategy (Base Interface)                │
│  - getVehicleType()                                                      │
│  - isModeSupported(operation_mode)                                       │
│  - getControlModeFlags(operation_mode, ...)                              │
│  - setModeRequirements(flags)                                            │
│  - getFallbackMode(failed_mode, is_armed)                                │
│  - getAutomationTaskForMode(operation_mode)                              │
└─────────────────────────────────────────────────────────────────────────┘
                    ▲                                      ▲
                    │                                      │
    ┌───────────────┴───────────────┐    ┌─────────────────┴──────────────┐
    │     RotaryWingStrategy        │    │      WheelLoaderStrategy       │
    │  - Manual, Stab, Alt, Pos     │    │  - Manual (direct control)     │
    │  - Auto: Mission, RTL, Land   │    │  - Auto VLA (trajectory)       │
    │  - Requires stabilization     │    │  - No stabilization needed     │
    │  - Failsafe: Descend          │    │  - Failsafe: Manual            │
    └───────────────────────────────┘    └────────────────────────────────┘
```

### Vehicle Type Registry

The `VehicleTypeRegistry` provides singleton access to vehicle type strategies:

```cpp
#include <lib/vehicle_type/VehicleTypeRegistry.hpp>

// Get strategy for current vehicle type
const auto* strategy = vehicle_type::VehicleTypeRegistry::getStrategy(vehicle_type);

// Check if mode is supported
if (strategy->isModeSupported(operation_mode)) {
    // Use strategy for this mode
}
```

## Supported Vehicle Types

The system currently supports the following vehicle types defined in `VehicleStatus.msg`:

| Type | Value | Description | Strategy |
|------|-------|-------------|----------|
| `VEHICLE_TYPE_ROTARY_WING` | 1 | Multirotor/helicopter | `RotaryWingStrategy` |
| `VEHICLE_TYPE_FIXED_WING` | 2 | Fixed-wing aircraft | (uses default) |
| `VEHICLE_TYPE_ROVER` | 3 | Ground vehicle rover | (uses default) |
| `VEHICLE_TYPE_WHEEL_LOADER` | 4 | Articulated wheel loader | `WheelLoaderStrategy` |

## Mode Manager

The `mode_manager` module is the central controller for vehicle operation modes. It:
1. Receives the current vehicle type from `vehicle_status`
2. Uses the vehicle type strategy to select appropriate modes
3. Manages mode transitions and failsafes

### Vehicle-Specific Mode Selection

Mode selection is delegated to the vehicle type strategy:

```cpp
void ModeManager::selectAndActivateMode()
{
    // Get strategy for current vehicle type
    if (selectModeForVehicleType(vehicle_type)) {
        return;  // Strategy handled mode selection
    }
    
    // Fall back to default handling for unsupported types
    // ...
}
```

## Adding a New Vehicle Type

### Step 1: Define Vehicle Type Constant

Add a new constant in `msg/versioned/VehicleStatus.msg`:

```
uint8 VEHICLE_TYPE_YOUR_VEHICLE = 5  # Description of your vehicle
```

### Step 2: Create Vehicle Type Strategy

Create a new strategy class in `src/lib/vehicle_type/`:

```cpp
// YourVehicleStrategy.hpp
#pragma once
#include "VehicleTypeStrategy.hpp"

namespace vehicle_type {

class YourVehicleStrategy : public VehicleTypeStrategy
{
public:
    uint8_t getVehicleType() const override {
        return vehicle_status_s::VEHICLE_TYPE_YOUR_VEHICLE;
    }
    
    const char *getVehicleTypeName() const override {
        return "Your Vehicle";
    }
    
    bool isModeSupported(uint8_t operation_mode) const override {
        // Define which modes your vehicle supports
    }
    
    void getControlModeFlags(uint8_t operation_mode,
                             const offboard_control_mode_s &offboard_control_mode,
                             vehicle_control_mode_s &control_mode) const override {
        // Set control mode flags for each operation mode
    }
    
    void setModeRequirements(failsafe_flags_s &flags) const override {
        // Define sensor requirements for each mode
    }
    
    uint8_t getFallbackMode(uint8_t failed_mode, bool is_armed) const override {
        // Define fallback behavior
    }
    
    // ... implement other virtual methods
};

} // namespace vehicle_type
```

### Step 3: Register the Strategy

Add your strategy to `VehicleTypeRegistry.hpp`:

```cpp
#include "YourVehicleStrategy.hpp"

class VehicleTypeRegistry {
public:
    static const VehicleTypeStrategy *getStrategy(uint8_t vehicle_type) {
        switch (vehicle_type) {
        case vehicle_status_s::VEHICLE_TYPE_YOUR_VEHICLE:
            return &_your_vehicle_strategy;
        // ... other cases
        }
    }
    
private:
    static YourVehicleStrategy _your_vehicle_strategy;
};
```

### Step 4: Create Vehicle-Specific Modes (if needed)

If your vehicle needs custom mode implementations, create them in `src/modules/mode_manager/modes/`:

1. Create directory: `ModeYourVehicle/`
2. Implement the mode class inheriting from `Mode`:
   - `ModeYourVehicle.hpp`
   - `ModeYourVehicle.cpp`
   - `CMakeLists.txt`
   - Parameters file (optional)

### Step 5: Add Mode Selection Logic

If your vehicle needs specialized mode selection in `ModeManager`, add a handler:

```cpp
void ModeManager::selectYourVehicleMode()
{
    const vehicle_type::YourVehicleStrategy *strategy = 
        static_cast<const vehicle_type::YourVehicleStrategy*>(
            vehicle_type::VehicleTypeRegistry::getStrategy(
                vehicle_status_s::VEHICLE_TYPE_YOUR_VEHICLE));
    
    // Use strategy to guide mode selection
    uint8_t operation_mode = _vehicle_status_sub.get().operation_mode;
    
    if (operation_mode == vehicle_status_s::OPERATION_MODE_YOUR_AUTO_MODE) {
        switchTask(ModeIndex::YourAutoMode);
    } else {
        switchTask(ModeIndex::YourManualMode);
    }
}
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

### Wheel Loader Strategy

The `WheelLoaderStrategy` defines:

- **Supported modes**: `OPERATION_MODE_MANUAL`, `OPERATION_MODE_AUTO_VLA`, `OPERATION_MODE_TERMINATION`
- **No stabilization required**: Ground vehicles don't need attitude control
- **Failsafe to manual**: Unlike aircraft, wheel loaders can safely stop with manual control
- **VLA trajectory task**: Autonomous operation uses VLA trajectories, not waypoint missions

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

### 1. Use the Strategy Pattern

Always implement `VehicleTypeStrategy` for new vehicle types to ensure:
- Consistent interface across all vehicle types
- Easy addition of new vehicle types
- Clear separation of vehicle-specific logic

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

1. **Unit Tests**: Test individual strategy classes
2. **Integration Tests**: Test mode switching and control flow
3. **Hardware-in-Loop**: Test with simulated or real hardware
4. **Safety Tests**: Verify emergency stop and failsafe behavior

## Troubleshooting

### Mode Not Activating

1. Check `vehicle_status.vehicle_type` is set correctly
2. Verify strategy is registered in `VehicleTypeRegistry`
3. Check `isModeSupported()` returns true for the mode
4. Review mode selection logic in ModeManager

### Control Setpoints Not Published

1. Verify subscriptions in your mode class
2. Check for valid input data
3. Review timeout handling

### Build Errors

1. Ensure all headers are included
2. Verify CMakeLists.txt dependencies
3. Check strategy implementation is complete

---

*Last Updated: 2024*
*Version: 2.0*
