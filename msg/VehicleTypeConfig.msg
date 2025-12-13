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
uint8 MODE_CHANGE_LOGIC_ROTARY_WING = 3  # Rotary wing (multicopter) specific mode change logic
uint8 MODE_CHANGE_LOGIC_FIXED_WING = 4   # Fixed wing specific mode change logic

# Supported commands bitmask - defines which MAVLink commands are supported
# Each bit corresponds to a command category
uint32 supported_commands_mask

# Command category bit positions
uint8 CMD_CATEGORY_ARM_DISARM = 0      # Arm/disarm commands
uint8 CMD_CATEGORY_TAKEOFF_LAND = 1    # Takeoff/land commands
uint8 CMD_CATEGORY_NAVIGATION = 2      # Navigation/waypoint commands
uint8 CMD_CATEGORY_MISSION = 3         # Mission commands
uint8 CMD_CATEGORY_GEOFENCE = 4        # Geofence commands
uint8 CMD_CATEGORY_GIMBAL = 5          # Gimbal/camera commands
uint8 CMD_CATEGORY_ACTUATOR = 6        # Direct actuator commands
uint8 CMD_CATEGORY_BOOM_BUCKET = 7     # Boom/bucket commands (wheel loader)
uint8 CMD_CATEGORY_VLA = 8             # VLA trajectory commands
uint8 CMD_CATEGORY_PAYLOAD = 9         # Payload commands

# Event reaction configuration bitmask
# Defines how the vehicle reacts to certain events
uint32 event_reactions_mask

# Event reaction bit positions
uint8 EVENT_REACT_RC_LOSS_RTL = 0          # React to RC loss with RTL
uint8 EVENT_REACT_RC_LOSS_LAND = 1         # React to RC loss with Land
uint8 EVENT_REACT_RC_LOSS_HOLD = 2         # React to RC loss with Position Hold
uint8 EVENT_REACT_RC_LOSS_ESTOP = 3        # React to RC loss with E-Stop (ground vehicles)
uint8 EVENT_REACT_DATALINK_LOSS_RTL = 4    # React to datalink loss with RTL
uint8 EVENT_REACT_DATALINK_LOSS_LAND = 5   # React to datalink loss with Land
uint8 EVENT_REACT_DATALINK_LOSS_CONTINUE = 6 # Continue mission on datalink loss
uint8 EVENT_REACT_LOW_BATTERY_RTL = 7      # React to low battery with RTL
uint8 EVENT_REACT_LOW_BATTERY_LAND = 8     # React to low battery with Land
uint8 EVENT_REACT_GEOFENCE_RTL = 9         # React to geofence breach with RTL
uint8 EVENT_REACT_GEOFENCE_HOLD = 10       # React to geofence breach with Hold
uint8 EVENT_REACT_COLLISION_AVOID = 11     # Enable collision avoidance
uint8 EVENT_REACT_OBSTACLE_STOP = 12       # Stop on obstacle detection (ground vehicles)

# RC input configuration
uint8 rc_input_mode                 # RC input interpretation mode
uint8 RC_INPUT_MODE_STANDARD = 0    # Standard stick mapping
uint8 RC_INPUT_MODE_WHEEL_LOADER = 1 # Wheel loader specific (includes boom/bucket)
uint8 RC_INPUT_MODE_ROVER = 2       # Rover specific (no altitude)

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
