# Automation task result
# Published by navigator/automation when task is accepted/rejected
# Used for command ACK routing back through command_processor

uint64 timestamp                    # time since system start (microseconds)

# Result constants
uint8 RESULT_ACCEPTED = 0           # Task accepted and will execute
uint8 RESULT_DENIED = 1             # Task denied (not allowed)
uint8 RESULT_TEMPORARILY_REJECTED = 2  # Task temporarily rejected
uint8 RESULT_UNSUPPORTED = 3        # Task not supported
uint8 RESULT_FAILED = 4             # Task failed to start
uint8 RESULT_IN_PROGRESS = 5        # Task in progress
uint8 RESULT_COMPLETED = 6          # Task completed successfully

uint8 task_type                     # Original TASK_* type
uint8 result                        # RESULT_* constant

# Source information for ACK routing back to command source
uint8 cmd_source_system
uint8 cmd_source_component
uint32 cmd_command
