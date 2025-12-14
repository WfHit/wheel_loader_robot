# Flight termination request message from CommandProcessor to SystemManager

uint64 timestamp		# time since system start (microseconds)

bool terminate			# true to activate flight termination, false to deactivate

uint8 source_system		# Source system ID
uint8 source_component		# Source component ID
