# End effector trajectory setpoint message
# Published by high-level trajectory planners, subscribed by end effector trajectory follower
# Contains 6DOF pose setpoints for the wheel loader's end effector (bucket tip)

uint64 timestamp                    # time since system start (microseconds)

# End effector position in reference frame
float32 x                          # Target X position [m]
float32 y                          # Target Y position [m]
float32 z                          # Target Z position [m]

# End effector velocity in reference frame
float32 vx                         # Target X velocity [m/s]
float32 vy                         # Target Y velocity [m/s]
float32 vz                         # Target Z velocity [m/s]

# End effector acceleration in reference frame
float32 ax                         # Target X acceleration [m/s^2]
float32 ay                         # Target Y acceleration [m/s^2]
float32 az                         # Target Z acceleration [m/s^2]

# End effector orientation (quaternion)
float32[4] q                       # Quaternion [w, x, y, z] (unit quaternion)

# End effector angular velocity
float32 wx                         # Angular velocity around X axis [rad/s]
float32 wy                         # Angular velocity around Y axis [rad/s]
float32 wz                         # Angular velocity around Z axis [rad/s]

# Reference frame specification
uint8 reference_frame              # Reference frame for positions/orientations

# Motion constraints
float32 max_linear_velocity        # Maximum allowed linear velocity [m/s]
float32 max_angular_velocity       # Maximum allowed angular velocity [rad/s]
float32 max_linear_acceleration    # Maximum allowed linear acceleration [m/s^2]
float32 max_angular_acceleration   # Maximum allowed angular acceleration [rad/s^2]

# Trajectory timing
float32 trajectory_time            # Total trajectory duration [s]
float32 time_from_start           # Time from start of trajectory [s]

# Control mode and flags
uint8 control_mode                 # Control mode (see constants below)
uint8 priority                     # Trajectory priority (0: lowest, 255: highest)
bool valid                         # Validity flag
bool enable_orientation_control    # Enable orientation control (false = position only)
bool hold_position                 # Hold position when trajectory completes
bool emergency_stop                # Emergency stop command

# Trajectory validation
float32 confidence_score           # Planner confidence in trajectory [0.0-1.0]
uint32 sequence_id                 # Sequence identifier for tracking
bool sequence_complete             # Whether current sequence is complete

# Reference frame constants
uint8 FRAME_LOCAL_NED = 0         # Local NED frame (relative to home)
uint8 FRAME_BODY = 1              # Vehicle body frame
uint8 FRAME_CHASSIS = 2           # Chassis frame (for wheel loader)
uint8 FRAME_WORLD = 3             # World/global frame

# Control mode constants
uint8 MODE_TRAJECTORY = 0         # Full 6DOF trajectory following
uint8 MODE_POSITION = 1          # Position control only
uint8 MODE_ORIENTATION = 2       # Orientation control only
uint8 MODE_STOP = 3              # Stop command
uint8 MODE_EMERGENCY_STOP = 4    # Emergency stop
uint8 MODE_HOLD = 5              # Hold current pose
