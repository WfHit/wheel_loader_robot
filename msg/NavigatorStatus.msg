# Current status of a Navigator mode
# The possible values of operation_mode are defined in the VehicleStatus msg.
uint64 timestamp  # time since system start (microseconds)

uint8 operation_mode   # Source mode (values in VehicleStatus)
uint8 failure     # Navigator failure enum

uint8 FAILURE_NONE = 0
uint8 FAILURE_HAGL = 1 # Target altitude exceeds maximum height above ground
