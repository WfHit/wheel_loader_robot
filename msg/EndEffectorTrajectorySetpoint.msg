# End effector trajectory setpoint message
# Published by high-level trajectory planners, subscribed by end effector trajectory follower
# Contains 2DOF joint angle setpoints for the wheel loader's boom and bucket

uint64 timestamp                    # time since system start (microseconds)

# Joint angles in chassis frame
float32 boom_angle                 # Boom angle relative to chassis [rad]
float32 bucket_angle               # Bucket angle relative to boom [rad]

# Joint angle rates
float32 boom_angle_rate            # Boom angle rate [rad/s]
float32 bucket_angle_rate          # Bucket angle rate [rad/s]

# Joint limits and constraints
float32 max_boom_angle_rate        # Maximum allowed boom angle rate [rad/s]
float32 max_bucket_angle_rate      # Maximum allowed bucket angle rate [rad/s]

# Control flags
bool valid                         # Validity flag
uint8 priority                     # Trajectory priority (0: lowest, 255: highest)
bool hold_position                 # Hold position when trajectory completes
bool emergency_stop                # Emergency stop command

# Trajectory tracking
uint32 sequence_id                 # Sequence identifier for tracking
bool sequence_complete             # Whether current sequence is complete
float32 confidence_score           # Planner confidence in trajectory [0.0-1.0]

# Control mode constants
uint8 MODE_TRAJECTORY = 0          # Joint trajectory following
uint8 MODE_POSITION = 1           # Joint position control
uint8 MODE_STOP = 2               # Stop command
uint8 MODE_EMERGENCY_STOP = 3     # Emergency stop
uint8 MODE_HOLD = 4               # Hold current joint positions
