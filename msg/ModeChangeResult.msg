# Mode change result from mode_manager back to system_manager
# Used for command ACK and state synchronization

uint64 timestamp                    # time since system start (microseconds)

uint8 requested_mode                # The mode that was requested
uint8 result_mode                   # The mode that was actually entered (may differ with fallback)

# Result codes matching vehicle_command_ack
uint8 RESULT_ACCEPTED = 0           # Mode change successful
uint8 RESULT_TEMPORARILY_REJECTED = 1  # Mode unavailable now, try again
uint8 RESULT_DENIED = 2             # Mode not available for vehicle type
uint8 RESULT_UNSUPPORTED = 3        # Mode not supported
uint8 RESULT_FAILED = 4             # Mode change failed
uint8 result

# Echo back command info for ACK routing
uint8 cmd_source_system
uint8 cmd_source_component
uint32 cmd_command
