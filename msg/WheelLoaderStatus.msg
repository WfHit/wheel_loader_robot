# Wheel loader overall system status
uint64 timestamp    # time since system start (microseconds)

# System operation modes
bool zeroing_mode_active    # true if actuators are in zeroing mode
bool autonomous_mode        # true if in autonomous operation mode
bool manual_mode           # true if in manual control mode
bool emergency_stop        # true if emergency stop is active

# Overall system health
uint8 system_health        # 0=unknown, 1=ok, 2=warning, 3=error, 4=critical
float32 overall_health_score # 0.0-1.0, higher is better

# Load handling status
bool load_detected         # true if load is detected
float32 load_weight_kg       # estimated load weight in kg
bool boom_extended         # true if boom is extended
bool bucket_tilted         # true if bucket is tilted

# Safety status
bool all_safety_systems_ok # true if all safety systems are operational
uint32 active_faults       # bitmask of active fault conditions
uint32 warning_flags       # bitmask of warning conditions

# Position and motion status
float32 vehicle_speed_ms     # current vehicle speed in m/s
float32 boom_angle_deg       # current boom angle in degrees
float32 bucket_angle_deg     # current bucket angle in degrees
