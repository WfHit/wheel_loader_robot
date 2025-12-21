# Vehicle identity and type information
# Contains MAVLink system identification and vehicle locomotion type

uint64 timestamp # time since system start (microseconds)

# Current vehicle locomotion method. A vehicle can have different methods (e.g. VTOL transitions from RW to FW method)
uint8 vehicle_type
uint8 VEHICLE_TYPE_UNKNOWN = 0
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3
uint8 VEHICLE_TYPE_WHEEL_LOADER = 4  # Articulated wheel loader with boom and bucket

# MAVLink identification
uint8 system_type  # system type, contains mavlink MAV_TYPE
uint8 system_id    # system id, contains MAVLink's system ID field
uint8 component_id # subsystem / component id, contains MAVLink's component ID field
