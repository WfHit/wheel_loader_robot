# Command Processing Architecture

## Implementation Status

| Component | Status | Notes |
|-----------|--------|-------|
| `VehicleTypeStrategy` interface | ✅ Complete | Added `shouldRejectCommand()`, `getTargetModeForCommand()` |
| Vehicle strategies (4 types) | ✅ Complete | WheelLoader, Rover, RotaryWing, FixedWing |
| `VehicleTypeRegistry` helpers | ✅ Complete | Static singleton with helper methods |
| `mode_change_request.msg` | ✅ Complete | uORB message for mode requests |
| `mode_change_result.msg` | ✅ Complete | uORB message for results/ACK |
| `UserModeIntention` in mode_manager | ✅ Complete | New class in `mode_manager/` |
| `ModeChangePublisher` helper | ✅ Complete | Helper for system_manager |
| `ModeManager` integration | ✅ Complete | Added UserModeIntention member |
| SystemManager refactoring | 🔄 Pending | Need to use ModeChangePublisher |

## Overview

This document describes the redesigned architecture for `vehicle_command_s` processing that supports different vehicle types cleanly through the Strategy Pattern.

## Problems with Current Design

1. **Monolithic switch-case in `handle_command()`** - A 600+ line function with vehicle-type checks scattered throughout
2. **`_user_mode_intention` in SystemManager** - Mode intention management belongs in mode_manager, not system_manager
3. **Tight coupling** - Command handling, mode changes, and arming logic are intertwined
4. **Hard to extend** - Adding a new vehicle type requires modifying the central switch-case

## New Architecture

### Module Responsibilities

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          SystemManager                                       │
│                                                                              │
│  Core Responsibilities:                                                      │
│  • Arming/disarming state machine                                           │
│  • Failsafe orchestration                                                   │
│  • Health and safety monitoring                                             │
│  • Vehicle status publication                                               │
│                                                                              │
│  Command Processing (delegated):                                            │
│  • Dispatch commands to CommandProcessor                                    │
│  • Execute arm/disarm based on result                                       │
│  • Forward mode change requests to mode_manager                             │
└────────────────────────────────────────────────────┬────────────────────────┘
                                                     │
                    ┌────────────────────────────────┼────────────────────────┐
                    │                                │                        │
                    ▼                                ▼                        ▼
┌───────────────────────────┐ ┌───────────────────────────┐ ┌─────────────────────────┐
│     CommandProcessor      │ │       ModeManager         │ │      Automation         │
│                           │ │                           │ │      (Navigator)        │
│ • Command validation      │ │ • Mode selection          │ │                         │
│ • Vehicle-type dispatch   │ │ • UserModeIntention       │ │ • Mission execution     │
│ • Command result          │ │ • Control setpoint gen    │ │ • RTL, Land, etc.       │
│                           │ │ • Mode availability       │ │ • VLA trajectory        │
└───────────────────────────┘ └───────────────────────────┘ └─────────────────────────┘
```

### Command Processing Flow

```
vehicle_command_s
       │
       ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                         SystemManager::handle_command()                       │
│                                                                               │
│  1. Check target system/component                                            │
│  2. Check vehicle-type command support via VehicleTypeRegistry               │
│  3. Dispatch to appropriate handler:                                         │
│     • ARM/DISARM → internal arm()/disarm()                                   │
│     • MODE CHANGE → publish mode_change_request                              │
│     • NAVIGATION → forward to CommandProcessor                               │
│     • ACTUATOR → handle internally                                           │
│  4. Answer command with result                                               │
└──────────────────────────────────────────────────────────────────────────────┘
                                       │
            ┌──────────────────────────┼──────────────────────────┐
            │                          │                          │
            ▼                          ▼                          ▼
    ┌───────────────┐        ┌─────────────────┐        ┌─────────────────┐
    │ Arm/Disarm    │        │ mode_change_    │        │ CommandProcessor│
    │ (internal)    │        │ request (uORB)  │        │ (for nav cmds)  │
    └───────────────┘        └────────┬────────┘        └─────────────────┘
                                      │
                                      ▼
                             ┌─────────────────┐
                             │   ModeManager   │
                             │                 │
                             │ UserModeIntention
                             │ validates and   │
                             │ changes mode    │
                             └─────────────────┘
```

### New uORB Messages

#### mode_change_request.msg
```
uint64 timestamp
uint8 requested_mode          # OPERATION_MODE_* constant
uint8 source                  # ModeChangeSource enum
bool allow_fallback           # Allow fallback to lower mode
bool force                    # Force mode change (skip checks)
uint8 source_system           # MAVLink source system
uint8 source_component        # MAVLink source component
```

#### mode_change_result.msg
```
uint64 timestamp
uint8 requested_mode
uint8 result_mode             # Actual mode entered (may differ if fallback)
uint8 result                  # CMD_RESULT_*
```

### CommandProcessor Strategy

The `CommandProcessor` uses the existing `VehicleTypeStrategy` to determine command handling:

```cpp
// In lib/vehicle_type/VehicleTypeStrategy.hpp

/**
 * @brief Process a navigation/action command
 *
 * Override this to provide vehicle-specific command handling.
 * Return CommandResult indicating how command was handled.
 */
virtual CommandResult processCommand(
    const vehicle_command_s &cmd,
    CommandContext &context) const
{
    return CommandResult::Delegated;  // Use default handling
}

/**
 * @brief Check if command requires mode change
 * @return target mode, or OPERATION_MODE_MAX if no mode change needed
 */
virtual uint8_t getTargetModeForCommand(uint16_t command) const = 0;
```

### Implementation in VehicleTypeStrategy

Each vehicle type implements command-specific behavior:

```cpp
// WheelLoaderStrategy.hpp

uint8_t getTargetModeForCommand(uint16_t command) const override
{
    switch (command) {
    case VEHICLE_CMD_DO_REPOSITION:
        return vehicle_status_s::OPERATION_MODE_AUTO_VLA;  // Wheel loader uses VLA for positioning

    case VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
        return vehicle_status_s::OPERATION_MODE_MANUAL;    // No RTL for ground vehicles, go manual

    default:
        return vehicle_status_s::OPERATION_MODE_MAX;       // No mode change
    }
}

bool shouldRejectCommand(const vehicle_command_s &cmd) const override
{
    switch (cmd.command) {
    case VEHICLE_CMD_NAV_TAKEOFF:
    case VEHICLE_CMD_NAV_LAND:
    case VEHICLE_CMD_DO_ORBIT:
        return true;   // Ground vehicle - reject aerial commands
    default:
        return false;
    }
}
```

### UserModeIntention Migration

Move `UserModeIntention` from `system_manager/` to `mode_manager/`:

**Before:**
```
system_manager/
├── system_manager.cpp
├── user_mode_intention.hpp   ← HERE
└── user_mode_intention.cpp
```

**After:**
```
mode_manager/
├── ModeManager.cpp
├── UserModeIntention.hpp     ← MOVED HERE
├── UserModeIntention.cpp
└── ...
```

### Communication Between Modules

```
SystemManager                      ModeManager
    │                                   │
    │  ───── mode_change_request ─────► │
    │                                   │
    │  ◄──── vehicle_status.mode ────── │
    │  ◄──── mode_change_result ─────── │
    │                                   │
```

SystemManager no longer directly manipulates mode state. It:
1. Publishes `mode_change_request` when command requires mode change
2. Reads mode from `vehicle_status` (published by mode_manager)
3. Optionally receives `mode_change_result` for command ACK

## Benefits

1. **Clean separation** - SystemManager handles safety/arming, ModeManager handles modes
2. **Extensible** - New vehicle types implement strategy methods, no central switch-case
3. **Testable** - Each strategy can be unit tested independently
4. **Single source of truth** - Vehicle behavior defined once in strategy class
5. **Reduced coupling** - Modules communicate via uORB, not direct method calls

## Migration Steps

1. ✅ Create `mode_change_request.msg` and `mode_change_result.msg`
2. ✅ Move `UserModeIntention` to mode_manager (new implementation)
3. ✅ Add `getTargetModeForCommand()` to `VehicleTypeStrategy`
4. 🔄 Update `SystemManager::handle_command()` to:
   - ✅ Use `VehicleTypeRegistry::shouldRejectCommand()` for validation
   - ⏳ Publish `mode_change_request` instead of calling `_user_mode_intention`
5. ✅ Update `ModeManager` to:
   - Subscribe to `mode_change_request`
   - Own `UserModeIntention` instance
   - Publish `mode_change_result`
6. ⏳ Remove `_user_mode_intention` from SystemManager (keep for backward compatibility during transition)

## Next Steps

To complete the migration, update `SystemManager::handle_command()`:

```cpp
// Before:
if (_user_mode_intention.change(vehicle_status_s::OPERATION_MODE_AUTO_RTL, getSourceFromCommand(cmd))) {
    cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
} else {
    cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}

// After:
_mode_change_publisher.requestModeChangeFromCommand(
    vehicle_status_s::OPERATION_MODE_AUTO_RTL,
    cmd);
// ACK will be routed via mode_change_result message
```
