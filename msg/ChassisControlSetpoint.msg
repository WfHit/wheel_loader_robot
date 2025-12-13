# Chassis control setpoint in WORLD coordinates
# Published by ModeVLA at 50Hz
# Subscribed by chassis trajectory follower

uint64 timestamp                    # time since system start (microseconds)

# Position setpoint (world frame)
float32 x                           # X position [m]
float32 y                           # Y position [m]
float32 heading                     # Heading [rad]

# Velocity setpoint (world frame)
float32 vx                          # X velocity [m/s]
float32 vy                          # Y velocity [m/s]
float32 yaw_rate                    # Yaw rate [rad/s]

# Acceleration setpoint (world frame)
float32 ax                          # X acceleration [m/s²]
float32 ay                          # Y acceleration [m/s²]
float32 yaw_acceleration            # Yaw acceleration [rad/s²]

# Steering control
float32 steering_angle              # Articulation angle [rad]
float32 steering_rate               # Articulation rate [rad/s]

# Traction control feedforward
float32 target_slip_rate            # Target slip rate [0.0-1.0]

# Trajectory info
uint32 sequence                     # Trajectory sequence ID
bool valid                          # Setpoint is valid
