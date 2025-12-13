/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file SystemManagerBase.hpp
 *
 * Base class for vehicle-type-specific SystemManager implementations.
 * This provides the common infrastructure while allowing vehicle-specific
 * behavior through virtual method overrides.
 */

#pragma once

#include "system_manager.hpp"

/**
 * @class SystemManagerBase
 *
 * Abstract base class that extends SystemManager with virtual hooks for
 * vehicle-type-specific behavior. Each vehicle type (WheelLoader, RotaryWing,
 * FixedWing, Rover) should create a subclass that overrides the virtual methods.
 *
 * Architecture:
 * - SystemManager: Original PX4 class with all common functionality
 * - SystemManagerBase: Adds virtual hooks for vehicle-specific behavior
 * - SystemManagerWheelLoader, etc.: Vehicle-specific implementations
 *
 * Virtual methods to override:
 * - shouldRejectCommand(): Vehicle-specific command filtering
 * - getFailsafeAction(): Vehicle-specific failsafe behavior
 * - updateVehicleSpecificState(): Vehicle-specific state updates
 * - getVehicleTypeName(): Human-readable vehicle type name
 */
class SystemManagerBase : public SystemManager
{
public:
	SystemManagerBase();
	~SystemManagerBase() override = default;

	//========================================================================
	// Vehicle Type Information (pure virtual - must override)
	//========================================================================

	/**
	 * Get the vehicle type this subclass handles
	 * @return vehicle_status_s::VEHICLE_TYPE_* constant
	 */
	virtual uint8_t getVehicleType() const = 0;

	/**
	 * Get human-readable name of this vehicle type
	 * @return Vehicle type name string
	 */
	virtual const char* getVehicleTypeName() const = 0;

	//========================================================================
	// Command Handling (virtual with default behavior)
	//========================================================================

	/**
	 * Check if a command should be rejected for this vehicle type.
	 * Override in subclasses to implement vehicle-specific command filtering.
	 *
	 * @param cmd The vehicle command to check
	 * @return true if command should be rejected, false to allow processing
	 */
	virtual bool shouldRejectCommand(const vehicle_command_s &cmd) const;

	/**
	 * Get the list of commands that are NOT supported by this vehicle type.
	 * Used for efficient command rejection.
	 *
	 * @param[out] commands Array to fill with unsupported command IDs
	 * @param max_commands Maximum number of commands that can be stored
	 * @return Number of unsupported commands filled in the array
	 */
	virtual int getUnsupportedCommands(uint16_t* commands, int max_commands) const;

	//========================================================================
	// Failsafe Behavior (virtual with default behavior)
	//========================================================================

	/**
	 * Failsafe action types for vehicle-specific handling
	 */
	enum class FailsafeAction : uint8_t {
		None = 0,
		Warn,              // Warning only
		Hold,              // Stop/hover in place
		RTL,               // Return to launch
		Land,              // Land immediately
		Descend,           // Controlled descent
		Terminate,         // Kill motors
		EmergencyStop,     // Emergency stop (ground vehicles)
		ManualTakeover,    // Force manual mode
	};

	/**
	 * Get the appropriate failsafe action for an event.
	 * Override to implement vehicle-specific failsafe logic.
	 *
	 * @param event The failsafe event that occurred
	 * @param current_state Current vehicle state
	 * @return Recommended failsafe action
	 */
	virtual FailsafeAction getFailsafeAction(uint8_t event, const vehicle_status_s& current_state) const;

	/**
	 * Check if this vehicle type supports a specific failsafe action
	 *
	 * @param action The failsafe action to check
	 * @return true if the action is supported
	 */
	virtual bool supportsFailsafeAction(FailsafeAction action) const;

	//========================================================================
	// Arming Logic (virtual with default behavior)
	//========================================================================

	/**
	 * Perform vehicle-specific pre-arm checks.
	 * Called in addition to standard pre-arm checks.
	 *
	 * @param[out] reason Human-readable reason if check fails
	 * @return true if all vehicle-specific checks pass
	 */
	virtual bool vehicleSpecificPrearmCheck(const char** reason) const;

	/**
	 * Get vehicle-specific arming requirements mask.
	 * Bits indicate which checks are required for this vehicle type.
	 *
	 * @return Bitmask of required arming checks
	 */
	virtual uint32_t getRequiredArmingChecksMask() const;

	//========================================================================
	// State Management (virtual with default behavior)
	//========================================================================

	/**
	 * Update vehicle-specific state.
	 * Called during the main run loop.
	 */
	virtual void updateVehicleSpecificState();

	/**
	 * Initialize vehicle-specific components.
	 * Called during module initialization.
	 */
	virtual void initVehicleSpecific();

	//========================================================================
	// Mode Availability (virtual with default behavior)
	//========================================================================

	/**
	 * Get mask of operation modes available for this vehicle type.
	 * Bits correspond to vehicle_status_s::OPERATION_MODE_* values.
	 *
	 * @return Bitmask of available operation modes
	 */
	virtual uint64_t getAvailableModesMask() const;

	/**
	 * Check if a specific operation mode is available
	 *
	 * @param operation_mode The mode to check
	 * @return true if the mode is available for this vehicle type
	 */
	virtual bool isModeAvailable(uint8_t operation_mode) const;

	/**
	 * Get the default operation mode for this vehicle type
	 *
	 * @return Default operation mode
	 */
	virtual uint8_t getDefaultMode() const;

	/**
	 * Get the failsafe mode for this vehicle type
	 *
	 * @return Failsafe operation mode
	 */
	virtual uint8_t getFailsafeMode() const;

protected:
	//========================================================================
	// Protected helpers for subclasses
	//========================================================================

	/**
	 * Check if a command is in a list of unsupported commands
	 */
	bool isCommandInList(uint16_t cmd, const uint16_t* list, int list_size) const;

	/**
	 * Log a command rejection with reason
	 */
	void logCommandRejection(const vehicle_command_s &cmd, const char* reason) const;
};

/**
 * @class SystemManagerFactory
 *
 * Factory for creating vehicle-type-specific SystemManager instances.
 * Uses the SYS_VEHICLE_TYPE parameter to determine which subclass to instantiate.
 */
class SystemManagerFactory
{
public:
	/**
	 * Create a SystemManager instance based on vehicle type parameter.
	 *
	 * @param vehicle_type_param Value from SYS_VEHICLE_TYPE parameter
	 * @return Pointer to newly allocated SystemManager subclass, or nullptr on failure
	 */
	static SystemManagerBase* create(int vehicle_type_param);

	/**
	 * Create using current parameter value
	 */
	static SystemManagerBase* createFromParam();

private:
	SystemManagerFactory() = delete;
};
