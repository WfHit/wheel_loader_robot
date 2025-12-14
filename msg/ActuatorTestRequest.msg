# Actuator test request message from CommandProcessor to SystemManager

uint64 timestamp		# time since system start (microseconds)

uint16 function			# Actuator function (from actuator_test.msg)
float32 value			# Test value (-1..1 or NaN for release)
float32 timeout_ms		# Test timeout in milliseconds

uint8 source_system		# Source system ID
uint8 source_component		# Source component ID
