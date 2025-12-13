# Mode completion result, published by an active mode.
# The possible values of operation_mode are defined in the VehicleStatus msg.
# Note that this is not always published (e.g. when a user switches modes or on
# failsafe activation)

uint32 MESSAGE_VERSION = 0

uint64 timestamp				 # time since system start (microseconds)


uint8 RESULT_SUCCESS = 0
# [1-99]: reserved
uint8 RESULT_FAILURE_OTHER = 100 # Mode failed (generic error)

uint8 result                     # One of RESULT_*

uint8 operation_mode                  # Source mode (values in VehicleStatus)
