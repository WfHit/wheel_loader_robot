# VLA End Effector Setpoint Triplet
# Position setpoint triplet for VLA end effector control
# Published by navigator AutoVLAEndEffector mode, consumed by FlightTaskAutoVLAEndEffector

uint64 timestamp                    # time since system start (microseconds)

# Previous setpoint (for trajectory smoothing)
bool previous_valid                 # Previous setpoint is valid
float32 previous_bucket_x           # Previous bucket position X (m)
float32 previous_bucket_y           # Previous bucket position Y (m)
float32 previous_bucket_z           # Previous bucket position Z (m)
float32 previous_bucket_yaw         # Previous bucket yaw (rad)

# Current setpoint (main target)
bool current_valid                  # Current setpoint is valid
float32 current_bucket_x            # Current bucket position X (m)
float32 current_bucket_y            # Current bucket position Y (m)
float32 current_bucket_z            # Current bucket position Z (m)
float32 current_bucket_roll         # Current bucket roll (rad)
float32 current_bucket_pitch        # Current bucket pitch (rad)
float32 current_bucket_yaw          # Current bucket yaw (rad)

# Next setpoint (for look-ahead planning)
bool next_valid                     # Next setpoint is valid
float32 next_bucket_x               # Next bucket position X (m)
float32 next_bucket_y               # Next bucket position Y (m)
float32 next_bucket_z               # Next bucket position Z (m)
float32 next_bucket_yaw             # Next bucket yaw (rad)

# Motion constraints
float32 max_velocity                # Maximum allowed velocity [m/s]
float32 max_acceleration            # Maximum allowed acceleration [m/s^2]
float32 acceptance_radius           # Acceptance radius [m]

# Wheel slip control
float32 front_wheel_slip_rate       # Target front wheel axis slip rate [0.0-1.0]
float32 rear_wheel_slip_rate        # Target rear wheel axis slip rate [0.0-1.0]

# Trajectory item information
uint8 item_type                     # Type of trajectory item
uint8 TYPE_WAYPOINT = 0             # Standard waypoint
uint8 TYPE_LOITER = 1               # Loiter at this position
uint8 TYPE_APPROACH = 2             # Approach maneuver (slower)
uint8 TYPE_DIG = 4                  # Digging operation
uint8 TYPE_DUMP = 5                 # Dumping operation
uint8 TYPE_LOAD = 6                 # Loading operation

# Control flags
bool autocontinue                   # Automatically continue to next item
float32 loiter_radius               # Loiter radius [m]
