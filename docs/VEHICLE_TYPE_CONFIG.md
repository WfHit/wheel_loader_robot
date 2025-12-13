# Vehicle Type Configuration System

## Overview

This document describes the vehicle type configuration system that enables different vehicle types to have different operation modes, automation tasks, and control logic. The system uses a special message (`VehicleTypeConfig`) published by the `system_manager` module to inform `mode_manager` and `automation` about vehicle-specific settings.

## Architecture

```
┌─────────────────┐                 ┌─────────────────────────────────┐
│  system_manager │────────────────▶│  vehicle_type_config (message)  │
└─────────────────┘                 └─────────────────────────────────┘
                                                  │
                     ┌────────────────────────────┼────────────────────────────┐
                     │                            │                            │
                     ▼                            ▼                            ▼
            ┌─────────────────┐          ┌─────────────────┐          ┌─────────────────┐
            │   mode_manager  │          │   automation    │          │  other modules  │
            └─────────────────┘          └─────────────────┘          └─────────────────┘
```

## VehicleTypeConfig Message

The `VehicleTypeConfig` message is published by `system_manager` and contains:

### Vehicle Type Identification
- `vehicle_type`: Current vehicle type constant

### Mode Configuration
- `available_modes_mask`: Bitmask of operation modes available for this vehicle type
- `default_mode`: Default operation mode when no specific mode is requested
- `failsafe_mode`: Mode to switch to during failsafe conditions

### Automation Task Configuration
- `available_automation_tasks_mask`: Bitmask defining which automation tasks are available

### Mode Change Logic
- `mode_change_logic`: Determines how mode changes are processed
  - `MODE_CHANGE_LOGIC_STANDARD` (0): Standard aircraft mode change logic
  - `MODE_CHANGE_LOGIC_WHEEL_LOADER` (1): Wheel loader specific mode change logic
  - `MODE_CHANGE_LOGIC_ROVER` (2): Rover specific mode change logic

### Control Capabilities
- `supports_altitude_control`: Vehicle supports altitude control
- `supports_position_control`: Vehicle supports position control
- `supports_velocity_control`: Vehicle supports velocity control
- `supports_attitude_control`: Vehicle supports attitude control
- `supports_manual_control`: Vehicle supports manual/RC control
- `supports_autonomous_control`: Vehicle supports autonomous control
- `supports_boom_control`: Vehicle has boom actuator (wheel loader)
- `supports_tilt_control`: Vehicle has tilt/bucket actuator (wheel loader)
- `supports_articulated_steering`: Vehicle uses articulated steering

### Safety Configuration
- `emergency_stop_decel`: Maximum deceleration for emergency stop (m/s²)
- `max_velocity`: Maximum allowed velocity (m/s)
- `max_steering_rate`: Maximum steering rate (rad/s)

## Vehicle Type Configurations

### Wheel Loader (`VEHICLE_TYPE_WHEEL_LOADER`)

**Available Modes:**
- `OPERATION_MODE_MANUAL`
- `OPERATION_MODE_AUTO_VLA`

**Available Automation Tasks:**
- `AUTOMATION_TASK_VLA_TRAJECTORY`

**Mode Change Logic:** `MODE_CHANGE_LOGIC_WHEEL_LOADER`

**Control Capabilities:**
- Position control: Yes
- Velocity control: Yes
- Altitude control: No
- Boom control: Yes
- Tilt control: Yes
- Articulated steering: Yes

### Rover (`VEHICLE_TYPE_ROVER`)

**Available Modes:**
- `OPERATION_MODE_MANUAL`
- `OPERATION_MODE_POSCTL`
- `OPERATION_MODE_AUTO_MISSION`
- `OPERATION_MODE_AUTO_RTL`
- `OPERATION_MODE_AUTO_LOITER`

**Available Automation Tasks:**
- `AUTOMATION_TASK_MISSION`
- `AUTOMATION_TASK_LOITER`
- `AUTOMATION_TASK_RTL`

**Mode Change Logic:** `MODE_CHANGE_LOGIC_ROVER`

### Rotary Wing (`VEHICLE_TYPE_ROTARY_WING`)

**Available Modes:**
- `OPERATION_MODE_MANUAL`
- `OPERATION_MODE_ALTCTL`
- `OPERATION_MODE_POSCTL`
- `OPERATION_MODE_AUTO_MISSION`
- `OPERATION_MODE_AUTO_RTL`
- `OPERATION_MODE_AUTO_LOITER`
- `OPERATION_MODE_AUTO_TAKEOFF`
- `OPERATION_MODE_AUTO_LAND`
- `OPERATION_MODE_ORBIT`
- `OPERATION_MODE_DESCEND`

**Available Automation Tasks:**
- `AUTOMATION_TASK_MISSION`
- `AUTOMATION_TASK_LOITER`
- `AUTOMATION_TASK_RTL`
- `AUTOMATION_TASK_TAKEOFF`
- `AUTOMATION_TASK_LAND`
- `AUTOMATION_TASK_PRECLAND`

**Mode Change Logic:** `MODE_CHANGE_LOGIC_STANDARD`

### Fixed Wing (`VEHICLE_TYPE_FIXED_WING`)

Uses standard mode change logic with full aircraft mode support.

## Usage

### In Mode Manager

```cpp
// Subscribe to vehicle type config
uORB::SubscriptionData<vehicle_type_config_s> _vehicle_type_config_sub{ORB_ID(vehicle_type_config)};

void selectAndActivateMode() {
    _vehicle_type_config_sub.update();
    const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();
    
    if (vtc.config_valid) {
        switch (vtc.mode_change_logic) {
        case vehicle_type_config_s::MODE_CHANGE_LOGIC_WHEEL_LOADER:
            selectWheelLoaderMode();
            return;
        // ...
        }
    }
}

// Check if mode is available
bool isModeAvailableForVehicleType(uint8_t operation_mode) const {
    const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();
    if (!vtc.config_valid || operation_mode >= 32) {
        return true; // Fallback
    }
    return (vtc.available_modes_mask & (1u << operation_mode)) != 0;
}
```

### In Automation Module

```cpp
// Subscribe to vehicle type config
uORB::SubscriptionData<vehicle_type_config_s> _vehicle_type_config_sub{ORB_ID(vehicle_type_config)};

// Check if automation task is available
bool isAutomationTaskAvailable(uint8_t task_type) {
    const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();
    if (!vtc.config_valid) {
        return true; // Fallback
    }
    return (vtc.available_automation_tasks_mask & (1u << task_type)) != 0;
}
```

## Adding a New Vehicle Type

1. Add the new vehicle type constant in `VehicleStatus.msg`
2. Add a new case in `SystemManager::updateVehicleTypeConfig()` with the appropriate configuration
3. Create vehicle-specific mode selection function in `ModeManager` (e.g., `selectYourVehicleMode()`)
4. Update mode_change_logic constant in `VehicleTypeConfig.msg` if needed

## Benefits

1. **Centralized Configuration**: All vehicle-specific settings are defined in one place (`system_manager`)
2. **Runtime Flexibility**: Configuration can be changed at runtime based on vehicle type
3. **Modularity**: `mode_manager` and `automation` don't need hardcoded vehicle type checks
4. **Extensibility**: Easy to add new vehicle types without modifying multiple modules
5. **Safety**: Each vehicle type has appropriate failsafe modes and safety limits

## Message Flow

1. `system_manager` detects vehicle type from MAV_TYPE parameter
2. `system_manager` publishes `vehicle_type_config` with appropriate settings
3. `mode_manager` subscribes and uses `mode_change_logic` to select mode selection strategy
4. `automation` subscribes and checks `available_automation_tasks_mask` before activating tasks

---

*Last Updated: 2024*
*Version: 1.0*
