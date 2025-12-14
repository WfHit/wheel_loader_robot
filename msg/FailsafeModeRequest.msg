# Failsafe mode request from system_manager to mode_manager
# Used when failsafe conditions require a mode change

uint64 timestamp				# time since system start (microseconds)

uint8 requested_mode				# Failsafe mode to switch to (vehicle_status_s::OPERATION_MODE_*)
uint8 failsafe_action				# The failsafe action that triggered this (from FailsafeBase::Action)

bool force					# Force mode change regardless of validation
bool allow_user_override			# User can override this failsafe mode via RC/stick

# Failsafe severity level
uint8 severity
uint8 SEVERITY_LOW = 0				# Warning only, user can easily override
uint8 SEVERITY_MEDIUM = 1			# Mode change recommended
uint8 SEVERITY_HIGH = 2				# Mode change required, limited override
uint8 SEVERITY_CRITICAL = 3			# Mode change mandatory, no override

# Source of failsafe trigger
uint8 source
uint8 SOURCE_NONE = 0
uint8 SOURCE_RC_LOSS = 1
uint8 SOURCE_DATALINK_LOSS = 2
uint8 SOURCE_GEOFENCE = 3
uint8 SOURCE_BATTERY = 4
uint8 SOURCE_MISSION = 5
uint8 SOURCE_POSITION_LOSS = 6
uint8 SOURCE_ALTITUDE_LOSS = 7
uint8 SOURCE_ATTITUDE_FAILURE = 8
uint8 SOURCE_IMBALANCED_PROP = 9
uint8 SOURCE_MOTOR_FAILURE = 10
uint8 SOURCE_OTHER = 255
