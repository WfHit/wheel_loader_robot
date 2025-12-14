# Storage/Parameter request message from CommandProcessor to SystemManager

uint64 timestamp		# time since system start (microseconds)

# Storage operation types
uint8 STORAGE_OP_LOAD_DEFAULT = 0	# Load parameters from default
uint8 STORAGE_OP_SAVE_DEFAULT = 1	# Save parameters to default
uint8 STORAGE_OP_RESET_ALL_CONFIG = 2	# Reset all config parameters
uint8 STORAGE_OP_RESET_SENSOR_FACTORY = 3	# Reset sensor factory calibration
uint8 STORAGE_OP_RESET_ALL = 4		# Reset all parameters

uint8 operation			# Storage operation requested

uint8 source_system		# Source system ID
uint8 source_component		# Source component ID
