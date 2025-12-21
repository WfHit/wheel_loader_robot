# Mode status published by mode_manager
# Informs system_manager and other modules about current mode state

uint64 timestamp				# time since system start (microseconds)

# Operation mode constants
uint8 OPERATION_MODE_MANUAL = 0               # Manual mode
uint8 OPERATION_MODE_ALTCTL = 1               # Altitude control mode
uint8 OPERATION_MODE_POSCTL = 2               # Position control mode
uint8 OPERATION_MODE_AUTO_MISSION = 3         # Auto mission mode
uint8 OPERATION_MODE_AUTO_LOITER = 4          # Auto loiter mode
uint8 OPERATION_MODE_AUTO_RTL = 5             # Auto return to launch mode
uint8 OPERATION_MODE_POSITION_SLOW = 6
uint8 OPERATION_MODE_FREE5 = 7
uint8 OPERATION_MODE_FREE4 = 8
uint8 OPERATION_MODE_FREE3 = 9
uint8 OPERATION_MODE_ACRO = 10                # Acro mode
uint8 OPERATION_MODE_FREE2 = 11
uint8 OPERATION_MODE_DESCEND = 12             # Descend mode (no position control)
uint8 OPERATION_MODE_TERMINATION = 13         # Termination mode
uint8 OPERATION_MODE_OFFBOARD = 14
uint8 OPERATION_MODE_STAB = 15                # Stabilized mode
uint8 OPERATION_MODE_FREE1 = 16
uint8 OPERATION_MODE_AUTO_TAKEOFF = 17        # Takeoff
uint8 OPERATION_MODE_AUTO_LAND = 18           # Land
uint8 OPERATION_MODE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
uint8 OPERATION_MODE_AUTO_PRECLAND = 20       # Precision land with landing target
uint8 OPERATION_MODE_ORBIT = 21               # Orbit in a circle
uint8 OPERATION_MODE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
uint8 OPERATION_MODE_AUTO_VLA = 23            # Auto VLA 7-DOF control (chassis + boom + tilt)
uint8 OPERATION_MODE_EXTERNAL1 = 24           # External mode 1 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL2 = 25           # External mode 2 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL3 = 26           # External mode 3 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL4 = 27           # External mode 4 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL5 = 28           # External mode 5 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL6 = 29           # External mode 6 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL7 = 30           # External mode 7 (proxy mode)
uint8 OPERATION_MODE_EXTERNAL8 = 31           # External mode 8 (proxy mode)
uint8 OPERATION_MODE_MAX = 32

uint8 current_mode				# Current operation mode (OPERATION_MODE_*)
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
