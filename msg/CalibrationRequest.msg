# Calibration request message from CommandProcessor to SystemManager

uint64 timestamp		# time since system start (microseconds)

# Calibration types
uint8 CALIBRATION_TYPE_GYRO = 0
uint8 CALIBRATION_TYPE_MAG = 1
uint8 CALIBRATION_TYPE_BARO = 2
uint8 CALIBRATION_TYPE_RC_START = 3
uint8 CALIBRATION_TYPE_RC_TRIM = 4
uint8 CALIBRATION_TYPE_RC_END = 5
uint8 CALIBRATION_TYPE_ACCEL = 6
uint8 CALIBRATION_TYPE_LEVEL = 7
uint8 CALIBRATION_TYPE_ACCEL_QUICK = 8
uint8 CALIBRATION_TYPE_AIRSPEED = 9
uint8 CALIBRATION_TYPE_ESC = 10
uint8 CALIBRATION_TYPE_MAG_QUICK = 11
uint8 CALIBRATION_TYPE_TEMPERATURE = 12

uint8 calibration_type		# Type of calibration requested

# Parameters for MAG_QUICK calibration
float32 mag_heading_rad		# Heading in radians for quick mag calibration
float32 latitude		# Latitude for quick mag calibration
float32 longitude		# Longitude for quick mag calibration

uint8 source_system		# Source system ID
uint8 source_component		# Source component ID
