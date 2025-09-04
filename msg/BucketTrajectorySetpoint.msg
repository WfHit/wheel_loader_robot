# Bucket trajectory setpoint message
# Published by operation modes, subscribed by bucket control module

uint64 timestamp                # time since system start (microseconds)

# Control mode
uint8 control_mode              # Control mode (see constants below)

# Chassis-relative position control
float32 target_angle            # Target bucket angle [rad] (chassis-relative)
float32 angular_velocity        # Target angular velocity [rad/s]
float32 angular_acceleration    # Target angular acceleration [rad/s^2]

# Direct actuator control
float32 actuator_position       # Direct actuator position [mm]

# World frame trajectory control
float32 x_position              # X position in world frame [m]
float32 y_position              # Y position in world frame [m]
float32 z_position              # Z position in world frame [m]

# World frame orientation (quaternion)
float32 q_w                     # Quaternion w component
float32 q_x                     # Quaternion x component
float32 q_y                     # Quaternion y component
float32 q_z                     # Quaternion z component

# Motion constraints
float32 max_velocity            # Maximum allowed angular velocity [rad/s]
float32 max_acceleration        # Maximum allowed angular acceleration [rad/s^2]

# Trajectory timing
float32 trajectory_time         # Total trajectory duration [s]
float32 time_from_start         # Time from start of trajectory [s]

# Control flags
uint8 priority                  # Trajectory priority (0: lowest, 255: highest)
bool valid                      # Validity flag

# Control mode constants
uint8 MODE_POSITION = 0         # Chassis-relative position control
uint8 MODE_DIRECT = 1          # Direct actuator position control
uint8 MODE_TRAJECTORY = 2      # World frame trajectory following
uint8 MODE_STOP = 3            # Stop command
uint8 MODE_EMERGENCY_STOP = 4   # Emergency stop
uint8 MODE_CALIBRATE = 5       # Calibration mode
