# Mode change request from system_manager to mode_manager
# Used to decouple arming/safety logic from mode selection logic

uint64 timestamp                    # time since system start (microseconds)

uint8 requested_mode                # Requested OPERATION_MODE_* constant

# Source of the mode change request
uint8 SOURCE_USER = 0               # RC stick or MAVLink from user
uint8 SOURCE_MODE_EXECUTOR = 1      # Internal mode executor
uint8 SOURCE_FAILSAFE = 2           # Failsafe triggered mode change
uint8 SOURCE_COMMAND = 3            # Vehicle command triggered
uint8 source

bool allow_fallback                 # Allow fallback to lower mode if requested unavailable
bool force                          # Force mode change (skip availability checks)

# Original command source for ACK routing
uint8 cmd_source_system             # MAVLink source system (0 if not from command)
uint8 cmd_source_component          # MAVLink source component
uint32 cmd_command                  # Original command ID (0 if not from command)
