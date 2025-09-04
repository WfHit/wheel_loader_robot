# VLA trajectory setpoint message for wheel loader autonomous operation
# Contains bucket position and pose trajectory setpoints from VLA algorithm

uint64 timestamp                    # time since system start (microseconds)

# Bucket trajectory setpoint outputs from VLA
float32 bucket_position_x           # Target bucket position X (m)
float32 bucket_position_y           # Target bucket position Y (m)
float32 bucket_position_z           # Target bucket position Z (m)
float32 bucket_orientation_roll     # Target bucket roll angle (rad)
float32 bucket_orientation_pitch    # Target bucket pitch angle (rad)
float32 bucket_orientation_yaw      # Target bucket yaw angle (rad)

# Trajectory timing and constraints
float32 trajectory_time             # Total trajectory duration [s]
float32 time_from_start            # Time from start of trajectory [s]
float32 max_velocity               # Maximum allowed velocity [m/s]
float32 max_acceleration           # Maximum allowed acceleration [m/s^2]

# Control flags
bool enable_trajectory             # Enable trajectory following
bool hold_position                # Hold current position when trajectory completes
bool emergency_stop                # Emergency stop command from VLA

# Control confidence and validation
float32 confidence_score            # VLA confidence in output [0.0-1.0]
bool valid_output                   # Whether output is valid for execution

# Sequence information
uint32 sequence_id                  # Sequence identifier for tracking
uint8 sequence_step                 # Current step in sequence
bool sequence_complete              # Whether current sequence is complete

# Control mode
uint8 control_mode                 # Control mode (trajectory, position, stop)

# Control mode constants
uint8 MODE_TRAJECTORY = 0          # Trajectory following mode
uint8 MODE_POSITION = 1           # Position control mode
uint8 MODE_STOP = 2               # Stop mode
uint8 MODE_EMERGENCY_STOP = 3     # Emergency stop mode
