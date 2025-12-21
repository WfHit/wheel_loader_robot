# Throw launch status message
# Reports the current state of throw launch functionality

uint64 timestamp		# time since system start (microseconds)

# State enum values
uint8 STATE_DISABLED = 0    # Feature disabled
uint8 STATE_IDLE = 1        # Feature enabled but not armed
uint8 STATE_ARMED = 2       # Ready for throw
uint8 STATE_UNSAFE = 3      # Throw detected, waiting for fall
uint8 STATE_FLYING = 4      # Flying - motors started

uint8 state                 # Current throw launch state (one of STATE_* values)

bool throw_launch_in_progress   # True if throw launch sequence is active (not DISABLED and not FLYING)
bool ready_to_throw             # True if in ARMED state and ready to be thrown
