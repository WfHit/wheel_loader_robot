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
 * @file VehicleTypeStrategy.hpp
 *
 * Vehicle Type Strategy Pattern - Base interface for vehicle-specific behavior
 *
 * This pattern allows different vehicle types to have their own:
 * - Operation modes and mode selection logic
 * - Automation tasks
 * - Control mode flags
 * - Mode requirements
 * - Failsafe behaviors
 *
 * Each vehicle type implements this interface to customize its behavior.
 */

#pragma once

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/offboard_control_mode.h>
#include <cstdint>

namespace vehicle_type
{

/**
 * @brief Describes a supported operation mode for a vehicle type
 */
struct OperationModeInfo {
	uint8_t mode_id;           ///< OPERATION_MODE_* constant
	const char *name;          ///< Human-readable name
	bool is_auto_mode;         ///< True if this is an autonomous mode
	bool requires_position;    ///< True if mode requires position estimate
	bool requires_manual_input; ///< True if mode requires RC/manual input
	uint8_t fallback_mode;     ///< Mode to fall back to if this mode fails
};

/**
 * @brief Configuration for mode change behavior
 */
struct ModeChangeConfig {
	bool allow_stick_override;     ///< Allow stick override in autonomous modes
	bool require_disarm_for_mode_change; ///< Require disarm before changing modes
	uint8_t default_mode;          ///< Default mode when no specific mode requested
	uint8_t failsafe_mode;         ///< Mode to enter on failsafe
	uint8_t manual_fallback_mode;  ///< Mode to fall back to when manual control needed
};

/**
 * @brief Base interface for vehicle type strategies
 *
 * Implement this interface for each vehicle type to customize behavior.
 */
class VehicleTypeStrategy
{
public:
	virtual ~VehicleTypeStrategy() = default;

	//========================================================================
	// Vehicle Identification
	//========================================================================

	/**
	 * @brief Get the vehicle type constant
	 * @return One of vehicle_status_s::VEHICLE_TYPE_* constants
	 */
	virtual uint8_t getVehicleType() const = 0;

	/**
	 * @brief Get human-readable name for this vehicle type
	 * @return Vehicle type name
	 */
	virtual const char *getVehicleTypeName() const = 0;

	//========================================================================
	// Operation Modes
	//========================================================================

	/**
	 * @brief Get the number of supported operation modes
	 * @return Number of operation modes
	 */
	virtual int getNumSupportedModes() const = 0;

	/**
	 * @brief Get information about a supported operation mode
	 * @param index Index into supported modes array (0 to getNumSupportedModes()-1)
	 * @return Pointer to mode info, or nullptr if index out of range
	 */
	virtual const OperationModeInfo *getSupportedMode(int index) const = 0;

	/**
	 * @brief Check if a specific operation mode is supported
	 * @param operation_mode OPERATION_MODE_* constant
	 * @return true if mode is supported
	 */
	virtual bool isModeSupported(uint8_t operation_mode) const = 0;

	/**
	 * @brief Get the bitmask of valid operation modes
	 * @return Bitmask where bit N is set if OPERATION_MODE N is valid
	 */
	virtual uint32_t getValidOperationModesMask() const = 0;

	/**
	 * @brief Get the bitmask of modes that users can select
	 * @return Bitmask where bit N is set if OPERATION_MODE N can be selected
	 */
	virtual uint32_t getSelectableOperationModesMask() const = 0;

	//========================================================================
	// Mode Selection Logic
	//========================================================================

	/**
	 * @brief Get the mode change configuration
	 * @return Mode change configuration
	 */
	virtual ModeChangeConfig getModeChangeConfig() const = 0;

	/**
	 * @brief Select the appropriate mode based on current state
	 *
	 * This is called when a mode change is requested to determine the actual
	 * mode to enter. May return a different mode than requested (e.g., fallback).
	 *
	 * @param requested_mode The requested operation mode
	 * @param current_mode The current operation mode
	 * @param is_armed True if vehicle is armed
	 * @param has_position True if position estimate is available
	 * @param has_manual_control True if manual control input is available
	 * @return The actual mode to enter
	 */
	virtual uint8_t selectMode(uint8_t requested_mode, uint8_t current_mode,
				   bool is_armed, bool has_position, bool has_manual_control) const = 0;

	/**
	 * @brief Get the fallback mode when current mode fails
	 * @param failed_mode The mode that failed
	 * @param is_armed True if vehicle is armed
	 * @return The fallback mode to enter
	 */
	virtual uint8_t getFallbackMode(uint8_t failed_mode, bool is_armed) const = 0;

	//========================================================================
	// Control Mode Flags
	//========================================================================

	/**
	 * @brief Get control mode flags for a specific operation mode
	 *
	 * Sets the appropriate control mode flags for the given operation mode.
	 * This determines which controllers are active.
	 *
	 * @param operation_mode The current operation mode
	 * @param offboard_control_mode Offboard control mode settings
	 * @param[out] control_mode Control mode flags to set
	 */
	virtual void getControlModeFlags(uint8_t operation_mode,
					 const offboard_control_mode_s &offboard_control_mode,
					 vehicle_control_mode_s &control_mode) const = 0;

	/**
	 * @brief Check if stabilization is required for this vehicle type
	 *
	 * Aircraft like multirotors require attitude/rate stabilization even in
	 * manual mode, while ground vehicles typically don't.
	 *
	 * @return true if stabilization is required
	 */
	virtual bool requiresStabilization() const = 0;

	//========================================================================
	// Mode Requirements
	//========================================================================

	/**
	 * @brief Set mode requirements flags
	 *
	 * Configures which sensor/system requirements apply to each mode for
	 * this vehicle type. Used for failsafe and arming checks.
	 *
	 * @param[out] flags Failsafe flags to configure
	 */
	virtual void setModeRequirements(failsafe_flags_s &flags) const = 0;

	//========================================================================
	// Failsafe Behavior
	//========================================================================

	/**
	 * @brief Get the failsafe action for a specific failure type
	 * @param failure_type Type of failure (from failsafe system)
	 * @param current_mode Current operation mode
	 * @param is_armed True if vehicle is armed
	 * @return Operation mode to enter as failsafe action
	 */
	virtual uint8_t getFailsafeAction(uint8_t failure_type, uint8_t current_mode, bool is_armed) const = 0;

	/**
	 * @brief Check if mode is considered safe for landing/stopping
	 * @param operation_mode Mode to check
	 * @return true if mode results in vehicle stopping/landing safely
	 */
	virtual bool isSafeStopMode(uint8_t operation_mode) const = 0;

	//========================================================================
	// Automation Task Support
	//========================================================================

	/**
	 * @brief Check if this vehicle type supports autonomous missions
	 * @return true if missions are supported
	 */
	virtual bool supportsMissions() const = 0;

	/**
	 * @brief Check if this vehicle type supports return-to-launch
	 * @return true if RTL is supported
	 */
	virtual bool supportsRTL() const = 0;

	/**
	 * @brief Get the automation task for a given operation mode
	 *
	 * Returns a task identifier that the automation module uses to select
	 * the appropriate task implementation.
	 *
	 * @param operation_mode Current operation mode
	 * @return Task identifier string, or nullptr if no task for this mode
	 */
	virtual const char *getAutomationTaskForMode(uint8_t operation_mode) const = 0;
};

} // namespace vehicle_type
