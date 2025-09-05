# Operation mode status message for wheel loader robot

uint64 timestamp                    # time since system start (microseconds)

# Current system state
uint8 current_mode                 # Currently active operation mode
uint8 requested_mode               # Requested operation mode (if switching)
bool mode_switch_pending           # Mode switch is in progress

# Safety status
bool emergency_stop                # Emergency stop active
bool system_armed                  # System is armed and ready
bool position_valid                # Position estimation is valid
bool manual_control_valid          # Manual control input is valid
bool vla_trajectory_setpoint_valid # VLA trajectory setpoint is valid

# Operation mode constants
uint8 MODE_MANUAL = 0
uint8 MODE_VLA = 1
uint8 MODE_ESTOP = 2
