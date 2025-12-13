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
 * @file ModeManagerBase.hpp
 *
 * Base class for vehicle-type-specific ModeManager implementations.
 * Provides common infrastructure with virtual hooks for vehicle-specific
 * mode selection behavior.
 */

#pragma once

#include "ModeManager.hpp"

/**
 * @class ModeManagerBase
 *
 * Abstract base class that extends ModeManager with virtual hooks for
 * vehicle-type-specific mode selection. Each vehicle type should create
 * a subclass that overrides the virtual methods.
 *
 * Virtual methods to override:
 * - selectVehicleSpecificMode(): Main mode selection logic
 * - isModeAvailable(): Vehicle-specific mode availability check
 * - getDefaultModeIndex(): Default mode when no specific mode requested
 * - getFailsafeModeIndex(): Mode to use on failsafe
 * - getVehicleTypeName(): Human-readable vehicle type name
 */
class ModeManagerBase : public ModeManager
{
public:
	ModeManagerBase();
	~ModeManagerBase() override = default;

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
	// Mode Selection (pure virtual - must override)
	//========================================================================

	/**
	 * Vehicle-specific mode selection implementation.
	 * Called when mode needs to be selected/changed.
	 */
	virtual void selectVehicleSpecificMode() = 0;

	//========================================================================
	// Mode Availability (virtual with default behavior)
	//========================================================================

	/**
	 * Check if a specific operation mode is available for this vehicle type.
	 *
	 * @param operation_mode The mode to check
	 * @return true if mode is available
	 */
	virtual bool isModeAvailable(uint8_t operation_mode) const;

	/**
	 * Get mask of available operation modes for this vehicle type.
	 * Bits correspond to vehicle_status_s::OPERATION_MODE_* values.
	 *
	 * @return Bitmask of available modes
	 */
	virtual uint64_t getAvailableModesMask() const;

	/**
	 * Get the default ModeIndex for this vehicle type.
	 * Used when no specific mode is requested.
	 *
	 * @return Default ModeIndex
	 */
	virtual ModeIndex getDefaultModeIndex() const;

	/**
	 * Get the failsafe ModeIndex for this vehicle type.
	 *
	 * @return Failsafe ModeIndex
	 */
	virtual ModeIndex getFailsafeModeIndex() const;

	//========================================================================
	// Mode Transition (virtual with default behavior)
	//========================================================================

	/**
	 * Check if transition from current mode to requested mode is allowed.
	 *
	 * @param from_mode Current operation mode
	 * @param to_mode Requested operation mode
	 * @return true if transition is allowed
	 */
	virtual bool isTransitionAllowed(uint8_t from_mode, uint8_t to_mode) const;

	/**
	 * Get mode index for a given operation mode.
	 * Vehicle types may map operation modes differently.
	 *
	 * @param operation_mode The operation mode
	 * @return Corresponding ModeIndex
	 */
	virtual ModeIndex getModeIndexForOperationMode(uint8_t operation_mode) const;

protected:
	//========================================================================
	// Protected helpers for subclasses
	//========================================================================

	/**
	 * Attempt to switch to a mode with fallback handling.
	 *
	 * @param mode_index The mode to switch to
	 * @param fallback_index Fallback mode if primary fails
	 * @return true if mode switched successfully
	 */
	bool switchModeWithFallback(ModeIndex mode_index, ModeIndex fallback_index);

	/**
	 * Log mode selection for debugging
	 */
	void logModeSelection(const char* mode_name, bool success) const;
};

/**
 * @class ModeManagerFactory
 *
 * Factory for creating vehicle-type-specific ModeManager instances.
 */
class ModeManagerFactory
{
public:
	/**
	 * Create a ModeManager instance based on vehicle type parameter.
	 *
	 * @param vehicle_type_param Value from SYS_VEHICLE_TYPE parameter
	 * @return Pointer to newly allocated ModeManager subclass, or nullptr on failure
	 */
	static ModeManagerBase* create(int vehicle_type_param);

	/**
	 * Create using current parameter value
	 */
	static ModeManagerBase* createFromParam();

private:
	ModeManagerFactory() = delete;
};
