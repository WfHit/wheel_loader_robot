# Vehicle type configuration message published by system_manager
# to inform mode_manager and automation about vehicle-specific settings.
# This enables different vehicle types to have different modes, automation tasks,
# and control logic without hardcoding vehicle type checks everywhere.

uint64 timestamp                    # time since system start (microseconds)

# Vehicle type identification (matches VehicleStatus.msg)
uint8 vehicle_type                  # Current vehicle type
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3
uint8 VEHICLE_TYPE_WHEEL_LOADER = 4

# Available operation modes bitmask for this vehicle type
# Each bit corresponds to an OPERATION_MODE value from VehicleStatus
uint32 available_modes_mask

# Default operation mode for this vehicle type
uint8 default_mode

# Failsafe mode for this vehicle type
uint8 failsafe_mode

# Automation task availability bitmask
# Defines which automation tasks are available for this vehicle type
uint32 available_automation_tasks_mask

# Automation task constants (bit positions)
uint8 AUTOMATION_TASK_MISSION = 0
uint8 AUTOMATION_TASK_LOITER = 1
uint8 AUTOMATION_TASK_RTL = 2
uint8 AUTOMATION_TASK_TAKEOFF = 3
uint8 AUTOMATION_TASK_LAND = 4
uint8 AUTOMATION_TASK_PRECLAND = 5
uint8 AUTOMATION_TASK_VLA_TRAJECTORY = 6

# Mode change logic type
uint8 mode_change_logic             # Determines how mode changes are processed
uint8 MODE_CHANGE_LOGIC_STANDARD = 0   # Standard aircraft mode change logic
uint8 MODE_CHANGE_LOGIC_WHEEL_LOADER = 1  # Wheel loader specific mode change logic
uint8 MODE_CHANGE_LOGIC_ROVER = 2      # Rover specific mode change logic

# Control capabilities for this vehicle type
bool supports_altitude_control      # Vehicle supports altitude control
bool supports_position_control      # Vehicle supports position control
bool supports_velocity_control      # Vehicle supports velocity control
bool supports_attitude_control      # Vehicle supports attitude control
bool supports_manual_control        # Vehicle supports manual/RC control
bool supports_autonomous_control    # Vehicle supports autonomous control
bool supports_boom_control          # Vehicle has boom actuator (wheel loader)
bool supports_tilt_control          # Vehicle has tilt/bucket actuator (wheel loader)
bool supports_articulated_steering  # Vehicle uses articulated steering

# Safety configuration
float32 emergency_stop_decel        # Max deceleration for emergency stop (m/s^2)
float32 max_velocity                # Maximum allowed velocity (m/s)
float32 max_steering_rate           # Maximum steering rate (rad/s)

# Status flags
bool config_valid                   # Configuration is valid and can be used
uint32 config_version               # Version number for configuration changes
