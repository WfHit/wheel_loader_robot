# VLA End Effector Trajectory Item message
# Individual trajectory waypoint for VLA end effector control
# Can be uploaded via MAVLink and stored in dataman

uint64 timestamp                    # time since system start (microseconds)

uint16 sequence                     # Sequence number of this trajectory item

# Bucket target position (world frame, NED)
float32 bucket_position_x           # Target bucket position X (m)
float32 bucket_position_y           # Target bucket position Y (m)
float32 bucket_position_z           # Target bucket position Z (m)

# Bucket target orientation (Euler angles)
float32 bucket_orientation_roll     # Target bucket roll angle (rad)
float32 bucket_orientation_pitch    # Target bucket pitch angle (rad)
float32 bucket_orientation_yaw      # Target bucket yaw angle (rad)

# Motion constraints
float32 max_velocity                # Maximum allowed velocity [m/s]
float32 max_acceleration            # Maximum allowed acceleration [m/s^2]
float32 time_inside                 # Time to wait at waypoint before continuing [s]
float32 acceptance_radius           # Radius in which waypoint is accepted as reached [m]

# Wheel slip control
float32 front_wheel_slip_rate       # Target front wheel axis slip rate [0.0-1.0]
float32 rear_wheel_slip_rate        # Target rear wheel axis slip rate [0.0-1.0]

# Control mode
uint8 control_mode                  # Control mode for this item
uint8 MODE_TRAJECTORY = 0           # Trajectory following mode
uint8 MODE_POSITION = 1             # Position hold mode
uint8 MODE_STOP = 2                 # Stop at this waypoint
uint8 MODE_EMERGENCY_STOP = 3       # Emergency stop

# Trajectory item type
uint8 item_type                     # Type of trajectory item
uint8 TYPE_WAYPOINT = 0             # Standard waypoint
uint8 TYPE_LOITER = 1               # Loiter at this position
uint8 TYPE_APPROACH = 2             # Approach maneuver (slower)
uint8 TYPE_DEPART = 3               # Departure maneuver
uint8 TYPE_DIG = 4                  # Digging operation
uint8 TYPE_DUMP = 5                 # Dumping operation
uint8 TYPE_LOAD = 6                 # Loading operation

# Control flags
bool autocontinue                   # Automatically continue to next item
bool force_heading                  # Heading must be reached before continuing
bool altitude_is_relative           # If true, altitude is relative to start point

# VLA confidence and validation
float32 confidence_score            # VLA confidence in this waypoint [0.0-1.0]
bool valid_output                   # Whether this item is valid for execution

# Frame of reference
uint8 frame                         # Coordinate frame
uint8 FRAME_GLOBAL = 0              # Global frame (lat/lon/alt)
uint8 FRAME_LOCAL_NED = 1           # Local NED frame
uint8 FRAME_BODY = 2                # Body-fixed frame
