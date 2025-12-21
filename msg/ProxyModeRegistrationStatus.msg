# Proxy mode registration status published by ProxyChecks (system_manager)
# Informs mode_manager about arming check registration health and responsiveness

uint64 timestamp				# time since system start (microseconds)

uint8 MAX_REGISTRATIONS = 8

# Registration status arrays (indexed by registration_id)
uint8 active_registrations_mask			# Bitmask of active registrations
bool[8] registration_valid			# Whether each registration slot is valid
bool[8] registration_unresponsive		# Whether each registration is unresponsive
int8[8] nav_mode_id				# Associated nav mode for each registration (-1 if generic arming check)
int8[8] replaces_operation_mode			# Which mode each registration replaces (-1 if none)

# Allow mode updates while armed (from COM_MODE_ARM_CHK param)
bool allow_update_while_armed
