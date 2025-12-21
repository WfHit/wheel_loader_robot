# Encodes the system state of the vehicle published by commander

uint32 MESSAGE_VERSION = 1

uint64 timestamp # time since system start (microseconds)

uint64 armed_time # Arming timestamp (microseconds)
uint64 takeoff_time # Takeoff timestamp (microseconds)

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED    = 2

uint8 latest_arming_reason
uint8 latest_disarming_reason
uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
uint8 ARM_DISARM_REASON_STICK_GESTURE = 1
uint8 ARM_DISARM_REASON_RC_SWITCH = 2
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
uint8 ARM_DISARM_REASON_MISSION_START = 5
uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6
uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
uint8 ARM_DISARM_REASON_KILL_SWITCH = 9
uint8 ARM_DISARM_REASON_LOCKDOWN = 10
uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11
uint8 ARM_DISARM_REASON_SHUTDOWN = 12
uint8 ARM_DISARM_REASON_UNIT_TEST = 13

# Bitmask of detected failures
uint16 failure_detector_status
uint16 FAILURE_NONE = 0
uint16 FAILURE_ROLL = 1              # (1 << 0)
uint16 FAILURE_PITCH = 2             # (1 << 1)
uint16 FAILURE_ALT = 4               # (1 << 2)
uint16 FAILURE_EXT = 8               # (1 << 3)
uint16 FAILURE_ARM_ESC = 16          # (1 << 4)
uint16 FAILURE_BATTERY = 32          # (1 << 5)
uint16 FAILURE_IMBALANCED_PROP = 64  # (1 << 6)
uint16 FAILURE_MOTOR = 128           # (1 << 7)

uint8 hil_state
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe

bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control
uint8 failsafe_defer_state # one of FAILSAFE_DEFER_STATE_*

# Link loss
bool gcs_connection_lost              # datalink to GCS lost
uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events
bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

# VTOL flags
bool is_vtol             # True if the system is VTOL capable
bool is_vtol_tailsitter  # True if the system performs a 90° pitch down rotation during transition from MC to FW
bool in_transition_mode  # True if VTOL is doing a transition
bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW

bool safety_button_available # Set to true if a safety button is connected
bool safety_off # Set to true if safety is off

bool power_input_valid                            # set if input power is valid
bool usb_connected                                # set to true (never cleared) once telemetry received from usb link

bool open_drone_id_system_present
bool open_drone_id_system_healthy

bool parachute_system_present
bool parachute_system_healthy

bool rc_calibration_in_progress
bool calibration_enabled

bool pre_flight_checks_pass             # true if all checks necessary to arm pass
