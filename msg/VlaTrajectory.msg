# VLA 7-DOF trajectory output message
# Contains 16-step trajectory prediction from VLA model
# All motion values are in BODY FRAME with DELTA semantics
# End effector values are ABSOLUTE

uint64 timestamp                    # time since system start (microseconds)
uint64 vla_timestamp                # timestamp from VLA model inference

# Trajectory horizon configuration
uint8 MAX_TRAJECTORY_POINTS = 16    # Maximum trajectory points
uint8 num_steps                     # Number of valid steps (max 16)
float32 step_interval               # Time interval between steps [s] (typically 0.1s)

# 7-DOF Trajectory Data (16 steps max)
# Index 0 = current step, Index 15 = 1.5s into future

# Chassis motion (body frame deltas)
float32[16] delta_x                 # Delta X position in body frame [m]
float32[16] delta_y                 # Delta Y position in body frame [m]
float32[16] delta_heading           # Delta heading (yaw change) [rad]
float32[16] steering_angle          # Articulation angle [rad]

# End effector state (absolute values)
float32[16] bucket_height           # Bucket height from ground [m]
float32[16] tilt_angle              # Bucket tilt angle [rad]

# Traction control feedforward
float32[16] target_slip_rate        # Target tire slip rate [0.0-1.0]

# Confidence and validity
float32 confidence                  # VLA model confidence [0.0-1.0]
bool valid                          # Trajectory is valid for execution

# Sequence tracking
uint32 sequence_id                  # Sequence ID for trajectory tracking

# Source identification
uint8 source                        # Source of trajectory
uint8 SOURCE_VLA_MODEL = 0          # From VLA neural network
uint8 SOURCE_PLANNER = 1            # From motion planner
uint8 SOURCE_OPERATOR = 2           # From operator input
