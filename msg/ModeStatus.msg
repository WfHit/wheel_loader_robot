# Mode status published by mode_manager
# Informs system_manager and other modules about current mode state

uint64 timestamp				# time since system start (microseconds)

uint8 current_mode				# Current operation mode (vehicle_status_s::OPERATION_MODE_*)
uint8 user_intended_mode			# Mode the user requested
uint8 previous_mode				# Previous mode before last change

int8 mode_executor_in_charge			# Who controls: -1 = no executor, 0 = autopilot, 1+ = external executor ID
int8 MODE_EXECUTOR_NONE = -1
int8 MODE_EXECUTOR_AUTOPILOT = 0

uint32 valid_modes_mask				# Bitmask of modes that are available for this vehicle type
uint32 can_set_modes_mask			# Bitmask of modes that can be set right now (considering arming state, etc.)

bool mode_change_in_progress			# A mode transition is currently happening
bool failsafe_mode_active			# Current mode is due to failsafe action
bool external_mode_active			# Current mode is an external mode (EXTERNAL1-8)

# Mode change result for feedback
uint8 last_change_result			# Result of last mode change attempt
uint8 RESULT_SUCCESS = 0
uint8 RESULT_DENIED = 1
uint8 RESULT_UNSUPPORTED = 2
uint8 RESULT_FAILED = 3
uint8 RESULT_IN_PROGRESS = 4
