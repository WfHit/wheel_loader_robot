# VLA End Effector Trajectory message
# Similar to Mission message, this allows VLA end effector trajectories to be uploaded via MAVLink
# and stored in dataman for execution by the AutoVLAEndEffector navigator mode

uint64 timestamp                    # time since system start (microseconds)

uint8 dataman_id                    # dataman storage location: 0 or 1
uint16 count                        # count of trajectory items stored in dataman
int32 current_seq                   # current sequence number, -1 if not set

uint32 trajectory_id                # indicates updates to the trajectory, reload from dataman if changed

# Trajectory execution state
uint8 execution_mode                # Current execution mode
uint8 EXECUTION_MODE_IDLE = 0       # Not executing
uint8 EXECUTION_MODE_ACTIVE = 1     # Actively following trajectory
uint8 EXECUTION_MODE_PAUSED = 2     # Paused
uint8 EXECUTION_MODE_COMPLETE = 3   # Trajectory completed

# Control flags
bool loop_trajectory                # If true, loop back to start after completing
bool reverse_on_complete            # If true, reverse trajectory direction on completion
