# Vehicle-Type Subclass Architecture Design

## Overview

This document describes the vehicle-type-specific subclass architecture for the PX4 Wheel Loader Robot project. The architecture allows each vehicle type (Wheel Loader, Rotary Wing, Fixed Wing, Rover) to have its own subclass with specialized behavior while sharing common functionality through base classes.

## Architecture Principle

**Base class REPLACES original, not extends it.**

The original classes (`SystemManager`, `ModeManager`, `Automation`) are refactored such that:
1. The original class becomes the abstract `*Base` class
2. All common functionality remains in the base class
3. Vehicle-specific behavior is defined in pure virtual or virtual methods
4. Vehicle-specific subclasses inherit from the base and override virtual methods
5. A factory function creates the correct subclass based on `SYS_VEHICLE_TYPE` parameter

## Vehicle Types

| Type | Value | Description |
|------|-------|-------------|
| ROTARY_WING | 1 | Multicopter/Helicopter |
| FIXED_WING | 2 | Airplane |
| ROVER | 3 | Ground vehicle |
| WHEEL_LOADER | 4 | Articulated wheel loader |

## Class Hierarchies

### SystemManager Hierarchy

```
SystemManagerBase (replaces original SystemManager)
├── SystemManagerWheelLoader   - Wheel loader specific
├── SystemManagerRotaryWing    - Multicopter specific
├── SystemManagerFixedWing     - Fixed wing specific
└── SystemManagerRover         - Ground vehicle specific
```

**Virtual Methods:**
- `getVehicleType()` - Pure virtual: returns vehicle type constant
- `getVehicleTypeName()` - Pure virtual: returns human-readable name
- `shouldRejectCommand()` - Vehicle-specific command filtering
- `getFailsafeAction()` - Vehicle-specific failsafe behavior
- `vehicleSpecificPrearmCheck()` - Additional prearm checks
- `getAvailableModesMask()` - Bitmask of available modes

### ModeManager Hierarchy

```
ModeManagerBase (replaces original ModeManager)
├── ModeManagerWheelLoader   - VLA + Manual modes only
├── ModeManagerRotaryWing    - Full multicopter modes
├── ModeManagerFixedWing     - Fixed wing modes
└── ModeManagerRover         - Ground vehicle modes
```

**Virtual Methods:**
- `getVehicleType()` - Pure virtual: returns vehicle type constant
- `getVehicleTypeName()` - Pure virtual: returns human-readable name
- `selectVehicleSpecificMode()` - Pure virtual: mode selection logic
- `isModeAvailable()` - Check if a mode is available
- `getDefaultModeIndex()` - Get default mode for this vehicle
- `getFailsafeModeIndex()` - Get failsafe mode for this vehicle

### Automation (Navigator) Hierarchy

```
AutomationBase (replaces original Automation)
├── AutomationWheelLoader   - VLA trajectory + Mission
├── AutomationRotaryWing    - Full aerial tasks
├── AutomationFixedWing     - Fixed wing tasks
└── AutomationRover         - Ground vehicle tasks
```

**Virtual Methods:**
- `getVehicleType()` - Pure virtual: returns vehicle type constant
- `getVehicleTypeName()` - Pure virtual: returns human-readable name
- `selectVehicleSpecificTask()` - Pure virtual: task selection logic
- `isTaskAvailable()` - Check if a task is available
- `getDefaultTask()` - Get default task for this vehicle
- `getFailsafeTask()` - Get failsafe task for this vehicle
- `getAcceptanceRadius()` - Vehicle-specific waypoint acceptance
- `getCruiseSpeed()` - Vehicle-specific cruise speed

## Factory Pattern

Each module uses a factory to create the correct subclass at runtime:

```cpp
// Example: SystemManagerFactory::createFromParam()
SystemManagerBase* SystemManagerFactory::createFromParam() {
    param_t param = param_find("SYS_VEHICLE_TYPE");
    int32_t vehicle_type = 0;
    param_get(param, &vehicle_type);
    return create(vehicle_type);
}

SystemManagerBase* SystemManagerFactory::create(int vehicle_type_param) {
    switch (vehicle_type_param) {
    case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
        return new SystemManagerRotaryWing();
    case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
        return new SystemManagerFixedWing();
    case vehicle_status_s::VEHICLE_TYPE_ROVER:
        return new SystemManagerRover();
    case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
        return new SystemManagerWheelLoader();
    default:
        PX4_WARN("Unknown vehicle type %d, using RotaryWing", vehicle_type_param);
        return new SystemManagerRotaryWing();
    }
}
```

## Module Entry Point Modification

Each module's `instantiate()` method uses the factory:

```cpp
// system_manager.cpp
SystemManager *SystemManager::instantiate(int argc, char *argv[])
{
    // Use factory to create vehicle-specific instance
    SystemManagerBase *instance = SystemManagerFactory::createFromParam();

    if (instance && argc >= 2 && !strcmp(argv[1], "-h")) {
        instance->enable_hil();
    }

    return instance;
}
```

## Vehicle-Specific Behaviors

### Wheel Loader
- **Rejected Commands**: Takeoff, Land, Orbit, Follow-Me, Precision Landing
- **Failsafe Action**: Emergency Stop (halt all motion)
- **Available Modes**: Manual, VLA (Vision-Language-Action)
- **Available Tasks**: VLA Trajectory, Mission
- **Waypoint Acceptance**: 2.0m radius
- **Cruise Speed**: 2.0 m/s

### Rotary Wing (Multicopter)
- **Rejected Commands**: None (full aerial support)
- **Failsafe Action**: RTL (Return to Launch)
- **Available Modes**: All flight modes including transition
- **Available Tasks**: Takeoff, Land, RTL, Mission, Loiter, Orbit, Precision Land
- **Waypoint Acceptance**: Parameter-based
- **Cruise Speed**: Parameter-based

### Fixed Wing
- **Rejected Commands**: Precision Landing (typically)
- **Failsafe Action**: RTL with circle
- **Available Modes**: Fixed wing specific modes
- **Available Tasks**: Takeoff, Land, RTL, Mission, Loiter
- **Waypoint Acceptance**: Larger radius for turns
- **Cruise Speed**: Parameter-based

### Rover
- **Rejected Commands**: Takeoff, Land, Aerial modes
- **Failsafe Action**: Hold position (stop)
- **Available Modes**: Manual, Mission, RTL
- **Available Tasks**: Mission, RTL, Hold
- **Waypoint Acceptance**: Parameter-based
- **Cruise Speed**: Ground speed parameter

## File Locations

### SystemManager Files
- `src/modules/system_manager/SystemManagerBase.hpp` - Base class
- `src/modules/system_manager/SystemManagerBase.cpp` - Factory + common impl
- `src/modules/system_manager/SystemManagerWheelLoader.hpp` - Wheel loader
- `src/modules/system_manager/SystemManagerRotaryWing.hpp` - Multicopter
- `src/modules/system_manager/SystemManagerFixedWing.hpp` - Fixed wing
- `src/modules/system_manager/SystemManagerRover.hpp` - Rover

### ModeManager Files
- `src/modules/mode_manager/ModeManagerBase.hpp` - Base class
- `src/modules/mode_manager/ModeManagerBase.cpp` - Factory + common impl
- `src/modules/mode_manager/ModeManagerWheelLoader.hpp` - Wheel loader
- `src/modules/mode_manager/ModeManagerRotaryWing.hpp` - Multicopter
- `src/modules/mode_manager/ModeManagerFixedWing.hpp` - Fixed wing
- `src/modules/mode_manager/ModeManagerRover.hpp` - Rover

### Automation Files
- `src/modules/automation/AutomationBase.hpp` - Base class
- `src/modules/automation/AutomationBase.cpp` - Factory + common impl
- `src/modules/automation/AutomationWheelLoader.hpp` - Wheel loader
- `src/modules/automation/AutomationRotaryWing.hpp` - Multicopter
- `src/modules/automation/AutomationFixedWing.hpp` - Fixed wing
- `src/modules/automation/AutomationRover.hpp` - Rover

## Integration Steps

1. **Update Base Classes**: Ensure *Base classes have all virtual methods and factory
2. **Create Subclasses**: Implement all four vehicle-type subclasses
3. **Update CMakeLists.txt**: Add *Base.cpp files to SRCS
4. **Update Module Entry Points**: Use factory in `instantiate()` methods
5. **Remove Old Strategy Files**: Clean up strategy-pattern files from previous approach
6. **Test**: Build and verify each vehicle type works correctly

## Migration from Strategy Pattern

The previous strategy pattern implementation should be deprecated in favor of this subclass architecture:

**Files to Remove/Deprecate:**
- `src/lib/vehicle_strategy/` - Old strategy library
- `src/modules/*/strategies/` - Old strategy registries
- Strategy-related CMake dependencies

**Reason for Change:**
The subclass architecture is cleaner because:
1. All vehicle-specific logic is in one subclass file
2. No runtime strategy switching overhead
3. Compiler can optimize virtual calls better
4. Easier to understand class hierarchy
5. Follows standard OOP inheritance patterns

## Configuration

Set the vehicle type in vehicle configuration:
```
param set SYS_VEHICLE_TYPE 4  # Wheel Loader
```

The factory will automatically instantiate the correct subclass at module startup.
