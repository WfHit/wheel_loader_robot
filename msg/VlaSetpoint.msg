# Single VLA setpoint containing full state for chassis, boom, and tilt
# Used as nested message in VlaSetpointTriplet
# Contains smoothed waypoint data from VlaTrajectoryTask

uint64 timestamp                    # time since system start (microseconds)

# === Chassis State (World Frame) ===
float32 chassis_x                   # X position [m]
float32 chassis_y                   # Y position [m]
float32 chassis_heading             # Heading [rad]
float32 chassis_vx                  # X velocity [m/s]
float32 chassis_vy                  # Y velocity [m/s]
float32 chassis_yaw_rate            # Yaw rate [rad/s]
float32 chassis_ax                  # X acceleration [m/s²]
float32 chassis_ay                  # Y acceleration [m/s²]
float32 chassis_yaw_acc             # Yaw acceleration [rad/s²]
float32 steering_angle              # Articulation angle [rad]
float32 steering_rate               # Articulation rate [rad/s]
float32 target_slip_rate            # Target slip rate [0.0-1.0]

# === Boom State (bucket_height, IK done in ModeVLA) ===
float32 bucket_height               # Bucket height from ground [m]
float32 bucket_height_velocity      # Bucket height velocity [m/s]

# === Tilt State ===
float32 tilt_angle                  # Tilt angle [rad]
float32 tilt_angular_velocity       # Angular velocity [rad/s]
float32 tilt_angular_acceleration   # Angular acceleration [rad/s²]

# === Validity ===
bool valid                          # Setpoint is valid
float32 progress                    # Progress along trajectory [0.0-1.0]
