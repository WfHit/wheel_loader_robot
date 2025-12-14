# Unified Vehicle Type Architecture

## Overview

This document describes the unified architecture for supporting different vehicle types (Wheel Loader, Rover, Rotary Wing, Fixed Wing) across the PX4 modules: system_manager, mode_manager, and automation (navigator).

## Design Principles

1. **Strategy Pattern** - Vehicle-specific behavior is encapsulated in strategy classes
2. **Single Source of Truth** - Each vehicle type's behavior is defined once in `lib/vehicle_type/`
3. **Composition over Inheritance** - No parallel class hierarchies, modules use strategies via registry
4. **Clean Module Boundaries** - Clear separation between safety (system_manager), modes (mode_manager), and tasks (automation)

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        lib/vehicle_type/                                     │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              VehicleTypeStrategy (Interface)                         │   │
│  │                                                                      │   │
│  │  • getVehicleType() / getName()                                     │   │
│  │  • getAvailableModesMask() / getAvailableAutomationTasksMask()      │   │
│  │  • shouldRejectCommand(cmd) / getTargetModeForCommand(cmd)          │   │
│  │  • getEventReaction(event) / getFailsafeMode()                      │   │
│  │  • fillConfig(config)                                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│           │                    │                    │                    │   │
│           ▼                    ▼                    ▼                    ▼   │
│  ┌────────────────┐ ┌────────────────┐ ┌────────────────┐ ┌────────────────┐│
│  │ WheelLoader    │ │ Rover          │ │ RotaryWing     │ │ FixedWing      ││
│  │ Strategy       │ │ Strategy       │ │ Strategy       │ │ Strategy       ││
│  └────────────────┘ └────────────────┘ └────────────────┘ └────────────────┘│
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              VehicleTypeRegistry (Static Singleton)                  │   │
│  │                                                                      │   │
│  │  • getStrategy(vehicle_type) → VehicleTypeStrategy*                 │   │
│  │  • shouldRejectCommand(vehicle_type, cmd) → bool                    │   │
│  │  • getTargetModeForCommand(vehicle_type, cmd) → uint8_t             │   │
│  │  • fillConfig(config, vehicle_type)                                 │   │
│  │  • isModeAvailable(vehicle_type, mode) → bool                       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                    │                          │                     │
                    ▼                          ▼                     ▼
         ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
         │   SystemManager     │  │    ModeManager      │  │     Automation      │
         │                     │  │                     │  │    (Navigator)      │
         │ Uses for:           │  │ Uses for:           │  │ Uses for:           │
         │ • Command rejection │  │ • Mode availability │  │ • Task availability │
         │ • Event reactions   │  │ • Mode selection    │  │ • Task behavior     │
         │ • Vehicle config    │  │ • Control setpoints │  │ • VLA handling      │
         └─────────────────────┘  └─────────────────────┘  └─────────────────────┘
```

## File Structure

```
src/
├── lib/
│   └── vehicle_type/                    # Single source of vehicle behavior
│       ├── VehicleTypeStrategy.hpp      # Strategy interface
│       ├── VehicleTypeRegistry.hpp      # Static registry
│       ├── VehicleTypeRegistry.cpp      # Static instances
│       ├── WheelLoaderStrategy.hpp      # Wheel loader specific
│       ├── RoverStrategy.hpp            # Rover specific
│       ├── RotaryWingStrategy.hpp       # Multicopter specific
│       └── FixedWingStrategy.hpp        # Fixed wing specific
│
├── modules/
│   ├── system_manager/                  # Safety & arming
│   │   ├── system_manager.cpp           # Uses VehicleTypeRegistry
│   │   ├── system_manager.hpp
│   │   └── ...                          # No vehicle-specific subclasses
│   │
│   ├── mode_manager/                    # Mode selection
│   │   ├── ModeManager.cpp              # Uses VehicleTypeRegistry
│   │   ├── ModeManager.hpp
│   │   └── ...                          # No vehicle-specific subclasses
│   │
│   └── automation/                      # Task execution
│       ├── Navigator.cpp                # Uses VehicleTypeRegistry
│       └── ...
│
└── msg/
    ├── vehicle_type_config.msg          # Published by system_manager
    ├── mode_change_request.msg          # For mode change requests
    └── mode_change_result.msg           # Mode change result
```

## Key Changes Made

### 1. Enhanced VehicleTypeStrategy Interface

Added new methods to support command processing:

```cpp
// Check if command should be rejected for this vehicle
virtual bool shouldRejectCommand(const vehicle_command_s &cmd) const = 0;

// Get target mode for a command (e.g., NAV_TAKEOFF → OPERATION_MODE_AUTO_TAKEOFF)
virtual uint8_t getTargetModeForCommand(uint16_t command) const = 0;
```

### 2. Refactored SystemManager::shouldRejectCommandForVehicleType()

**Before:** ~80 lines switch-case with duplicated logic
```cpp
bool SystemManager::shouldRejectCommandForVehicleType(const vehicle_command_s &cmd) const
{
    switch (_vehicle_status.vehicle_type) {
    case VEHICLE_TYPE_WHEEL_LOADER:
        switch (cmd.command) {
        case VEHICLE_CMD_NAV_TAKEOFF: return true;
        // ... 15+ more cases
        }
        break;
    case VEHICLE_TYPE_ROVER:
        // ... duplicate logic
    // ... 3 more vehicle types
    }
}
```

**After:** 1 line delegation
```cpp
bool SystemManager::shouldRejectCommandForVehicleType(const vehicle_command_s &cmd) const
{
    return vehicle_type::VehicleTypeRegistry::shouldRejectCommand(
        _vehicle_status.vehicle_type, cmd);
}
```

### 3. Vehicle-Specific Command Logic in Strategies

Each strategy now implements its own command handling:

```cpp
// WheelLoaderStrategy.hpp
bool shouldRejectCommand(const vehicle_command_s &cmd) const override
{
    switch (cmd.command) {
    case VEHICLE_CMD_NAV_TAKEOFF:
    case VEHICLE_CMD_NAV_LAND:
    case VEHICLE_CMD_DO_ORBIT:
        return true;  // Ground vehicle - reject aerial commands
    default:
        return false;
    }
}

uint8_t getTargetModeForCommand(uint16_t command) const override
{
    switch (command) {
    case VEHICLE_CMD_DO_REPOSITION:
        return OPERATION_MODE_AUTO_VLA;  // Wheel loader uses VLA
    case VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
        return OPERATION_MODE_MANUAL;    // No RTL for ground vehicles
    default:
        return OPERATION_MODE_MAX;       // No mode change
    }
}
```

## Communication via uORB

### New Messages

**mode_change_request.msg** - Request mode change from system_manager to mode_manager:
```
uint64 timestamp
uint8 requested_mode
uint8 source                    # USER, MODE_EXECUTOR, FAILSAFE, COMMAND
bool allow_fallback
bool force
uint8 cmd_source_system         # For ACK routing
uint8 cmd_source_component
uint32 cmd_command
```

**mode_change_result.msg** - Response from mode_manager:
```
uint64 timestamp
uint8 requested_mode
uint8 result_mode               # Actual mode entered
uint8 result                    # ACCEPTED, DENIED, etc.
```

## Benefits of This Architecture

1. **No Code Duplication** - Vehicle logic defined once per vehicle type
2. **Easy to Add Vehicle Types** - Create one new strategy file
3. **Testable** - Strategies can be unit tested independently
4. **Clear Boundaries** - Each module has defined responsibilities
5. **No Inheritance Hierarchies** - Single SystemManager, ModeManager, Navigator classes
6. **Static Allocation** - No dynamic memory for strategies

## Adding a New Vehicle Type

1. Create `NewVehicleStrategy.hpp` in `src/lib/vehicle_type/`
2. Implement all `VehicleTypeStrategy` pure virtual methods
3. Add static instance in `VehicleTypeRegistry.cpp`
4. Add case in `VehicleTypeRegistry::getStrategy()`
5. Add vehicle type constant in `vehicle_status.msg` if needed
6. Done! All modules automatically support the new vehicle

## Migration Notes

- Old vehicle-specific subclasses (SystemManagerWheelLoader, ModeManagerRover, etc.) are no longer needed
- The `vehicle_strategy/` folder under system_manager has been removed
- All vehicle behavior is now in `lib/vehicle_type/`

## Next Steps (TODO)

1. **Move UserModeIntention to mode_manager** - Mode selection belongs in mode_manager, not system_manager
2. **Implement mode_change_request handling** - ModeManager subscribes to requests and publishes results
3. **Refactor handle_command()** - Use `getTargetModeForCommand()` to determine mode changes
4. **Remove old vehicle-specific module files** - ModeManagerWheelLoader.hpp, etc.
