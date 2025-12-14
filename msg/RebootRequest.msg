# Reboot/Shutdown request message from CommandProcessor to SystemManager

uint64 timestamp		# time since system start (microseconds)

# Reboot types
uint8 REBOOT_TYPE_NONE = 0		# Do nothing
uint8 REBOOT_TYPE_REBOOT = 1		# Reboot autopilot
uint8 REBOOT_TYPE_SHUTDOWN = 2		# Shutdown autopilot
uint8 REBOOT_TYPE_BOOTLOADER = 3	# Reboot to bootloader

uint8 reboot_type		# Type of reboot/shutdown requested

uint8 source_system		# Source system ID
uint8 source_component		# Source component ID
