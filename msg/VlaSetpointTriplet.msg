# VLA Setpoint Triplet for wheel loader trajectory control
# Published by VlaTrajectoryTask at ~10Hz
# Subscribed by ModeVLA in mode_manager for 50Hz interpolation

uint64 timestamp                    # time since system start (microseconds)

# Triplet of setpoints (previous → current → next)
VlaSetpoint previous
VlaSetpoint current
VlaSetpoint next

# Trajectory metadata
uint32 sequence                     # Trajectory sequence ID
float32 vla_confidence              # VLA model confidence [0.0-1.0]
bool valid                          # Triplet is valid for execution
