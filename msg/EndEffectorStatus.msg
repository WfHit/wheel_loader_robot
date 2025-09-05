# End effector status message
# Published by end effector trajectory follower and control modules
# Contains current state and health information for the wheel loader's end effector

uint64 timestamp                    # time since system start (microseconds)

# Current position in reference frame
float32 x                          # Current X position [m]
float32 y                          # Current Y position [m]
float32 z                          # Current Z position [m]

# Current velocity in reference frame
float32 vx                         # Current X velocity [m/s]
float32 vy                         # Current Y velocity [m/s]
float32 vz                         # Current Z velocity [m/s]

# Current orientation (quaternion)
float32[4] q                       # Quaternion [w, x, y, z] (unit quaternion)

# Current angular velocity
float32 wx                         # Angular velocity around X axis [rad/s]
float32 wy                         # Angular velocity around Y axis [rad/s]
float32 wz                         # Angular velocity around Z axis [rad/s]

# Reference frame specification
uint8 reference_frame              # Reference frame for positions/orientations

# Control status
uint8 control_mode                 # Current control mode
uint8 trajectory_status            # Trajectory execution status
bool tracking_setpoint             # Currently tracking a trajectory setpoint
bool position_valid                # Position estimate is valid
bool orientation_valid             # Orientation estimate is valid

# Error metrics
float32 position_error             # Current position tracking error [m]
float32 orientation_error          # Current orientation tracking error [rad]
float32 velocity_error             # Current velocity tracking error [m/s]

# System health
bool system_healthy                # Overall system health status
bool sensors_healthy               # Sensor subsystem health
bool actuators_healthy             # Actuator subsystem health
bool communication_healthy         # Communication subsystem health

# Safety status
bool emergency_stop_active         # Emergency stop is currently active
bool safety_limits_violated        # Safety limits have been violated
bool collision_detected            # Collision detection active

# Diagnostic information
uint32 control_loop_counter        # Control loop execution counter
float32 control_loop_frequency     # Current control loop frequency [Hz]
float32 last_setpoint_age          # Age of last received setpoint [s]

# Reference frame constants
uint8 FRAME_LOCAL = 0              # Local NED frame
uint8 FRAME_BODY = 1               # Vehicle body frame
uint8 FRAME_WORLD = 2              # World frame

# Control mode constants
uint8 MODE_POSITION = 0            # Position control
uint8 MODE_VELOCITY = 1            # Velocity control
uint8 MODE_FORCE = 2               # Force/torque control
uint8 MODE_TRAJECTORY = 3          # Trajectory following
uint8 MODE_STOP = 4                # Stop command
uint8 MODE_EMERGENCY_STOP = 5      # Emergency stop
uint8 MODE_CALIBRATE = 6           # Calibration mode
uint8 MODE_MANUAL = 7              # Manual control

# Trajectory status constants
uint8 TRAJ_IDLE = 0                # No active trajectory
uint8 TRAJ_EXECUTING = 1           # Actively following trajectory
uint8 TRAJ_COMPLETED = 2           # Trajectory completed successfully
uint8 TRAJ_ABORTED = 3             # Trajectory aborted due to error
uint8 TRAJ_PREEMPTED = 4           # Trajectory preempted by new command
