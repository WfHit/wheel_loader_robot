# Arming request from command_processor to system_manager
# Used to decouple command processing from direct arming calls

uint64 timestamp                    # time since system start (microseconds)

bool request_arm                    # Request to arm the vehicle
bool request_disarm                 # Request to disarm the vehicle
bool force                          # Force the action (skip some checks)

# Original command source for ACK routing
uint8 cmd_source_system             # MAVLink source system (0 if not from command)
uint8 cmd_source_component          # MAVLink source component
uint32 cmd_command                  # Original command ID (0 if not from command)
