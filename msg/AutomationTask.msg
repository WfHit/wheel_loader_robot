# Automation task request
# Used by command_processor to request automation tasks from navigator/automation modules
# This decouples command processing from direct vehicle_status manipulation

uint64 timestamp                    # time since system start (microseconds)

# Task type constants
uint8 TASK_NONE = 0
uint8 TASK_MISSION = 1              # Start/continue mission
uint8 TASK_RTL = 2                  # Return to launch
uint8 TASK_LAND = 3                 # Land at current position
uint8 TASK_TAKEOFF = 4              # Takeoff to altitude
uint8 TASK_LOITER = 5               # Loiter at position
uint8 TASK_GOTO = 6                 # Go to specific position
uint8 TASK_ORBIT = 7                # Orbit around point
uint8 TASK_FOLLOW = 8               # Follow target
uint8 TASK_PRECLAND = 9             # Precision landing
uint8 TASK_VLA_TRAJECTORY = 10      # VLA 7-DOF trajectory (wheel loader)
uint8 TASK_DESCEND = 11             # Emergency descend
uint8 TASK_VTOL_TAKEOFF = 12        # VTOL takeoff
uint8 TASK_REPOSITION = 13          # Reposition to coordinates
uint8 TASK_CHANGE_ALTITUDE = 14     # Change altitude only
uint8 TASK_FIGURE_EIGHT = 15        # Figure-8 pattern
uint8 TASK_HOME_SET = 16            # Set home position

uint8 task_type                     # TASK_* constant

# Task parameters
float32 param1                      # Task-specific parameter 1
float32 param2                      # Task-specific parameter 2
float32 param3                      # Task-specific parameter 3
float32 param4                      # Task-specific parameter 4 (often yaw)
float64 latitude                    # Target latitude (deg) - NAN if not used
float64 longitude                   # Target longitude (deg) - NAN if not used
float32 altitude                    # Target altitude (m) - NAN if not used

# Mission specific
int32 mission_item_index            # Mission item to start from (-1 for current)

# Source information for ACK routing
uint8 cmd_source_system             # MAVLink source system (0 if not from command)
uint8 cmd_source_component          # MAVLink source component
uint32 cmd_command                  # Original command ID (0 if not from command)

# Flags
bool from_external                  # True if from external source (MAVLink)
bool force                          # Force task even if conditions not met
