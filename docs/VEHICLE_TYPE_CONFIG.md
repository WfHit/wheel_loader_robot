# Vehicle Type Configuration System

## Overview

This document describes the vehicle type configuration system that enables different vehicle types to have different operation modes, automation tasks, command sets, event reactions, and control logic. The system uses a **Strategy Pattern** architecture where each vehicle type has its own strategy class that defines its specific capabilities and behaviors.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Vehicle Type Library                               │
│   src/lib/vehicle_type/                                                     │
│                                                                             │
│   ┌─────────────────────────────────────┐                                   │
│   │      VehicleTypeStrategy            │ (Abstract Interface)              │
│   │  - getAvailableModes()              │                                   │
│   │  - getAutomationTasks()             │                                   │
│   │  - getCapabilities()                │                                   │
│   │  - getSafetyLimits()                │                                   │
│   │  - getSupportedCommandsMask()       │  <-- Command Set                  │
│   │  - isCommandSupported(cmd)          │                                   │
│   │  - handleCommand(cmd)               │                                   │
│   │  - getEventReactionsMask()          │  <-- Event Reactions              │
│   │  - getEventReaction(event)          │                                   │
│   │  - handleEvent(event)               │                                   │
│   │  - getRcInputMode()                 │  <-- RC Configuration             │
│   └───────────┬─────────────────────────┘                                   │
│               │                                                             │
│   ┌───────────┼───────────────────────────────────┐                         │
│   │           │           │           │           │                         │
│   ▼           ▼           ▼           ▼           ▼                         │
│ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐                 │
│ │WheelLdr │ │ Rover   │ │RotWing │ │FixedWng │ │ Custom  │                 │
│ │Strategy │ │Strategy │ │Strategy │ │Strategy │ │Strategy │                 │
│ └─────────┘ └─────────┘ └─────────┘ └─────────┘ └─────────┘                 │
│                                                                             │
│   ┌─────────────────────────┐                                               │
│   │  VehicleTypeRegistry    │ (Factory/Registry)                           │
│   │  - getStrategy(type)    │                                               │
│   │  - fillConfig(config)   │                                               │
│   └─────────────────────────┘                                               │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ fillConfig()
                                    ▼
┌─────────────────┐                 ┌─────────────────────────────────┐
│  system_manager │────────────────▶│  vehicle_type_config (message)  │
│                 │                 │  - available_modes_mask         │
│  handleCommand()◄────────────────│  - supported_commands_mask      │
│  handleEvent()  │                 │  - event_reactions_mask         │
│                 │                 │  - rc_input_mode                │
└─────────────────┘                 └─────────────────────────────────┘
                                                  │
                     ┌────────────────────────────┼────────────────────────────┐
                     │                            │                            │
                     ▼                            ▼                            ▼
            ┌─────────────────┐          ┌─────────────────┐          ┌─────────────────┐
            │   mode_manager  │          │   automation    │          │  other modules  │
            └─────────────────┘          └─────────────────┘          └─────────────────┘
```

## Strategy Pattern Implementation

### VehicleTypeStrategy (Abstract Interface)

Located at `src/lib/vehicle_type/VehicleTypeStrategy.hpp`, this defines the interface that all vehicle type strategies must implement:

```cpp
class VehicleTypeStrategy {
public:
    //========== Basic Information ==========
    virtual uint8_t getVehicleType() const = 0;
    virtual const char* getName() const = 0;
    
    //========== Mode Configuration ==========
    virtual uint32_t getAvailableModesMask() const = 0;
    virtual uint32_t getAvailableAutomationTasksMask() const = 0;
    virtual uint8_t getDefaultMode() const = 0;
    virtual uint8_t getFailsafeMode() const = 0;
    virtual uint8_t getModeChangeLogic() const = 0;
    
    //========== Capabilities & Safety ==========
    virtual ControlCapabilities getControlCapabilities() const = 0;
    virtual SafetyLimits getSafetyLimits() const = 0;
    
    //========== Command Set ==========
    virtual uint32_t getSupportedCommandsMask() const = 0;
    virtual bool isCommandSupported(uint16_t command) const = 0;
    virtual CommandResult handleCommand(const vehicle_command_s& cmd) const;
    
    //========== Event Reactions ==========
    virtual uint32_t getEventReactionsMask() const = 0;
    virtual EventAction getEventReaction(EventType event) const = 0;
    virtual EventAction handleEvent(EventType event) const;
    
    //========== RC Configuration ==========
    virtual uint8_t getRcInputMode() const = 0;
    
    // Helper methods
    bool isModeAvailable(uint8_t mode) const;
    bool isAutomationTaskAvailable(uint8_t task) const;
    bool isCommandCategorySupported(uint8_t category) const;
    bool isEventReactionEnabled(uint8_t reaction) const;
    void fillConfig(vehicle_type_config_s& config) const;
};
```

### Concrete Strategy Classes

Each vehicle type has its own strategy class:

| Class | File | Vehicle Type |
|-------|------|--------------|
| `WheelLoaderStrategy` | `WheelLoaderStrategy.hpp` | Articulated wheel loaders with boom/bucket |
| `RoverStrategy` | `RoverStrategy.hpp` | Ground rovers |
| `RotaryWingStrategy` | `RotaryWingStrategy.hpp` | Multicopters/helicopters |
| `FixedWingStrategy` | `FixedWingStrategy.hpp` | Fixed-wing aircraft |

### VehicleTypeRegistry (Factory)

The registry provides a simple factory pattern to get the appropriate strategy:

```cpp
// Get strategy for a vehicle type
const VehicleTypeStrategy* strategy = VehicleTypeRegistry::getStrategy(vehicle_type);

// Fill configuration directly
VehicleTypeRegistry::fillConfig(config, vehicle_type);

// Check availability
bool available = VehicleTypeRegistry::isModeAvailable(vehicle_type, mode);
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

### Command Set Configuration
- `supported_commands_mask`: Bitmask of supported command categories
  - `CMD_CATEGORY_ARM_DISARM` (0): Arm/disarm commands
  - `CMD_CATEGORY_TAKEOFF_LAND` (1): Takeoff/land commands
  - `CMD_CATEGORY_NAVIGATION` (2): Navigation/waypoint commands
  - `CMD_CATEGORY_MISSION` (3): Mission commands
  - `CMD_CATEGORY_GEOFENCE` (4): Geofence commands
  - `CMD_CATEGORY_GIMBAL` (5): Gimbal/camera commands
  - `CMD_CATEGORY_ACTUATOR` (6): Direct actuator commands
  - `CMD_CATEGORY_BOOM_BUCKET` (7): Boom/bucket commands (wheel loader)
  - `CMD_CATEGORY_VLA` (8): VLA trajectory commands
  - `CMD_CATEGORY_PAYLOAD` (9): Payload commands

### Event Reaction Configuration
- `event_reactions_mask`: Bitmask of enabled event reactions
  - `EVENT_REACT_RC_LOSS_RTL` (0): RTL on RC loss
  - `EVENT_REACT_RC_LOSS_LAND` (1): Land on RC loss
  - `EVENT_REACT_RC_LOSS_HOLD` (2): Hold on RC loss
  - `EVENT_REACT_RC_LOSS_ESTOP` (3): E-Stop on RC loss (ground vehicles)
  - `EVENT_REACT_DATALINK_LOSS_RTL` (4): RTL on datalink loss
  - `EVENT_REACT_DATALINK_LOSS_LAND` (5): Land on datalink loss
  - `EVENT_REACT_DATALINK_LOSS_CONTINUE` (6): Continue mission on datalink loss
  - `EVENT_REACT_LOW_BATTERY_RTL` (7): RTL on low battery
  - `EVENT_REACT_LOW_BATTERY_LAND` (8): Land on low battery
  - `EVENT_REACT_GEOFENCE_RTL` (9): RTL on geofence breach
  - `EVENT_REACT_GEOFENCE_HOLD` (10): Hold on geofence breach
  - `EVENT_REACT_COLLISION_AVOID` (11): Enable collision avoidance
  - `EVENT_REACT_OBSTACLE_STOP` (12): Stop on obstacle (ground vehicles)

### RC Input Configuration
- `rc_input_mode`: RC input interpretation mode
  - `RC_INPUT_MODE_STANDARD` (0): Standard stick mapping
  - `RC_INPUT_MODE_WHEEL_LOADER` (1): Wheel loader specific (includes boom/bucket)
  - `RC_INPUT_MODE_ROVER` (2): Rover specific (no altitude)

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

## Vehicle Type Comparison

| Feature | Wheel Loader | Rover | Rotary Wing | Fixed Wing |
|---------|--------------|-------|-------------|------------|
| **Modes** | Manual, VLA | Manual, PosCtl, Mission, RTL, Loiter | Full flight modes | Full flight modes |
| **Takeoff/Land** | ❌ | ❌ | ✅ | ✅ |
| **Navigation** | ❌ | ✅ | ✅ | ✅ |
| **Boom/Bucket** | ✅ | ❌ | ❌ | ❌ |
| **RC Loss Action** | E-Stop | RTL | RTL | RTL |
| **Datalink Loss** | Continue | Continue | RTL | RTL |
| **Obstacle Reaction** | Stop | Stop | Avoid | N/A |
| **RC Input Mode** | Wheel Loader | Rover | Standard | Standard |

## Adding a New Vehicle Type

### Step 1: Create a New Strategy Class

Create `src/lib/vehicle_type/YourVehicleStrategy.hpp`:

```cpp
#pragma once
#include "VehicleTypeStrategy.hpp"

namespace vehicle_type {

class YourVehicleStrategy : public VehicleTypeStrategy {
public:
    uint8_t getVehicleType() const override {
        return vehicle_status_s::VEHICLE_TYPE_YOUR_VEHICLE;
    }

    const char* getName() const override {
        return "Your Vehicle";
    }

    uint32_t getAvailableModesMask() const override {
        return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
               (1u << vehicle_status_s::OPERATION_MODE_YOUR_MODE);
    }

    uint32_t getAvailableAutomationTasksMask() const override {
        return (1u << vehicle_type_config_s::AUTOMATION_TASK_MISSION);
    }

    uint8_t getDefaultMode() const override {
        return vehicle_status_s::OPERATION_MODE_MANUAL;
    }

    uint8_t getFailsafeMode() const override {
        return vehicle_status_s::OPERATION_MODE_MANUAL;
    }

    uint8_t getModeChangeLogic() const override {
        return vehicle_type_config_s::MODE_CHANGE_LOGIC_YOUR_VEHICLE;
    }

    ControlCapabilities getControlCapabilities() const override {
        ControlCapabilities caps{};
        // Set your capabilities
        return caps;
    }

    SafetyLimits getSafetyLimits() const override {
        SafetyLimits limits{};
        // Set your limits
        return limits;
    }
};

} // namespace vehicle_type
```

### Step 2: Register in VehicleTypeRegistry

Update `VehicleTypeRegistry.hpp`:

```cpp
#include "YourVehicleStrategy.hpp"

// Add to getStrategy() switch
case vehicle_status_s::VEHICLE_TYPE_YOUR_VEHICLE:
    return &_your_vehicle_strategy;

// Add static member
static YourVehicleStrategy _your_vehicle_strategy;
```

### Step 3: Add Mode Change Logic (if needed)

If your vehicle needs custom mode selection, add a new constant to `VehicleTypeConfig.msg` and implement `selectYourVehicleMode()` in `ModeManager`.

## Benefits of Strategy Pattern

1. **Open/Closed Principle**: Add new vehicle types without modifying existing code
2. **Single Responsibility**: Each strategy class handles one vehicle type
3. **Testability**: Strategy classes can be unit tested independently
4. **No Switch Statements**: Configuration retrieval uses polymorphism instead of switch
5. **Centralized Configuration**: All vehicle configs in one library
6. **Type Safety**: Compile-time checking of interface implementation

## Usage Examples

### In System Manager

```cpp
#include <vehicle_type/VehicleTypeRegistry.hpp>

void SystemManager::updateVehicleTypeConfig() {
    // Use registry to fill config based on vehicle type
    vehicle_type::VehicleTypeRegistry::fillConfig(
        _vehicle_type_config, 
        _vehicle_status.vehicle_type
    );
    
    _vehicle_type_config.timestamp = hrt_absolute_time();
    _vehicle_type_config_pub.publish(_vehicle_type_config);
}
```

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
```

### In Automation Module

```cpp
bool isAutomationTaskAvailable(uint8_t task_type) {
    const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();
    if (!vtc.config_valid) {
        return true; // Fallback
    }
    return (vtc.available_automation_tasks_mask & (1u << task_type)) != 0;
}
```

## File Structure

```
src/lib/vehicle_type/
├── CMakeLists.txt
├── VehicleTypeStrategy.hpp     # Abstract interface
├── VehicleTypeRegistry.hpp     # Factory/registry
├── VehicleTypeRegistry.cpp     # Static instance definitions
├── WheelLoaderStrategy.hpp     # Wheel loader implementation
├── RoverStrategy.hpp           # Rover implementation
├── RotaryWingStrategy.hpp      # Multicopter implementation
└── FixedWingStrategy.hpp       # Fixed wing implementation
```

---

*Last Updated: 2024*
*Version: 2.0*

