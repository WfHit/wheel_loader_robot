# Boom trajectory setpoint message
# Published by operation modes, subscribed by boom control module

uint64 timestamp                # time since system start (microseconds)

# Target boom motion
float32 angle                   # Target boom angle [rad] (relative to horizontal)
float32 angular_velocity        # Target angular velocity [rad/s]
float32 angular_acceleration    # Target angular acceleration [rad/s^2]

# Motion constraints
float32 max_velocity            # Maximum allowed angular velocity [rad/s]
float32 max_acceleration        # Maximum allowed angular acceleration [rad/s^2]

# Control mode
uint8 control_mode              # Control mode (see constants below)

# Trajectory timing
float32 trajectory_time         # Total trajectory duration [s]
float32 time_from_start         # Time from start of trajectory [s]

# Control flags
uint8 priority                  # Trajectory priority (0: lowest, 255: highest)
bool valid                      # Validity flag

# Control mode constants
uint8 MODE_TRAJECTORY = 0       # Trajectory following mode
uint8 MODE_POSITION = 1         # Position hold mode
uint8 MODE_STOP = 2            # Stop command
uint8 MODE_EMERGENCY_STOP = 3   # Emergency stop
uint8 MODE_CALIBRATE = 4       # Calibration mode
