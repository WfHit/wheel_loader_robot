# Set home position request message from CommandProcessor to SystemManager

uint64 timestamp		# time since system start (microseconds)

bool use_current_position	# true to use current position, false to use specified coordinates

# Home position coordinates (used when use_current_position is false)
float64 latitude		# Latitude in degrees
float64 longitude		# Longitude in degrees
float32 altitude		# Altitude in meters (AMSL)

uint8 source_system		# Source system ID
uint8 source_component		# Source component ID
