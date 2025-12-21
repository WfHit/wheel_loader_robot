# Proxy mode status published by mode_manager (ProxyModeManagement)
# Provides system_manager with proxy mode registration info for failsafe resolution

uint64 timestamp				# time since system start (microseconds)

# Proxy mode registration info (up to 8 proxy modes)
uint8 MAX_PROXY_MODES = 8

uint8[8] proxy_mode_nav_states		# Nav state for each registered proxy mode (vehicle_status_s::OPERATION_MODE_EXTERNAL1-8)
uint8[8] replaces_operation_mode		# Which internal mode each external replaces (0xff = none)
bool[8] mode_valid				# Whether each slot has a valid registered mode
bool[8] mode_unresponsive			# Whether each registered mode is unresponsive
char[25] mode_names_0				# Name of proxy mode 0 (max 24 chars + null)
char[25] mode_names_1				# Name of proxy mode 1
char[25] mode_names_2				# Name of proxy mode 2
char[25] mode_names_3				# Name of proxy mode 3
char[25] mode_names_4				# Name of proxy mode 4
char[25] mode_names_5				# Name of proxy mode 5
char[25] mode_names_6				# Name of proxy mode 6
char[25] mode_names_7				# Name of proxy mode 7

# Executor info
int8 executor_in_charge				# Who is currently in charge (-1=none, 0=autopilot, 1+=executor ID)
int8 EXECUTOR_NONE = -1
int8 EXECUTOR_AUTOPILOT = 0

# Mode bitmasks for UI/GCS
uint32 valid_operation_mode_mask		# Bitmask of all valid modes (internal + external)
uint32 can_set_operation_mode_mask		# Bitmask of modes that can be set right now

# Arming check registration IDs (for ProxyChecks to map)
int8[8] arming_check_registration_id		# Arming check registration ID for each mode (-1 if none)

# Config overrides currently active
bool disable_auto_disarm
bool defer_failsafes
float32 defer_failsafes_timeout_s

# Flags
bool failsafe_action_active			# Whether failsafe is currently overriding executor control
