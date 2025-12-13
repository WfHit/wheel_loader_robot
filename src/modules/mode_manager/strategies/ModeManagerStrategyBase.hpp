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
 * @file ModeManagerStrategyBase.hpp
 *
 * Base class for ModeManager vehicle-type-specific strategies
 *
 * Defines the interface for vehicle-specific mode selection and setpoint generation.
 */

#pragma once

#include <lib/vehicle_strategy/VehicleStrategyTypes.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <stdint.h>

// Forward declaration
enum class ModeIndex : int;
enum class ModeError;

namespace mode_manager_strategy
{

using namespace vehicle_strategy;

/**
 * @brief Mode selection context passed to strategy
 */
struct ModeSelectionContext {
	uint8_t operation_mode;
	bool is_armed;
	bool in_transition_mode;
	bool flag_control_auto_enabled;
	bool flag_control_altitude_enabled;
	bool command_failed;
};

/**
 * @brief Mode selection result from strategy
 */
struct ModeSelectionResult {
	int requested_mode_index;    // ModeIndex to switch to (-1 for None)
	bool found_mode;             // A valid mode was found
	bool fallback_used;          // Fallback mode was used
	ModeResult result;           // Overall result
};

/**
 * @brief Base class for ModeManager vehicle-type-specific strategies
 *
 * Each vehicle type implements this interface to define how ModeManager
 * should select modes and generate setpoints for that vehicle type.
 */
class ModeManagerStrategyBase
{
public:
	virtual ~ModeManagerStrategyBase() = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	/**
	 * @brief Get the vehicle type this strategy handles
	 * @return VehicleType enum value
	 */
	virtual VehicleType getVehicleType() const = 0;

	/**
	 * @brief Get the human-readable name
	 * @return Strategy name string
	 */
	virtual const char* getName() const = 0;

	//========================================================================
	// Mode Selection
	//========================================================================

	/**
	 * @brief Select the appropriate mode based on current context
	 * @param context Current mode selection context
	 * @return ModeSelectionResult with requested mode
	 *
	 * This is the main entry point for vehicle-specific mode selection.
	 * The strategy should analyze the context and return the appropriate mode.
	 */
	virtual ModeSelectionResult selectMode(const ModeSelectionContext& context) const = 0;

	/**
	 * @brief Check if a mode is available for this vehicle type
	 * @param operation_mode OPERATION_MODE_* value
	 * @return true if mode is available
	 */
	virtual bool isModeAvailable(uint8_t operation_mode) const = 0;

	/**
	 * @brief Get the default mode for this vehicle type
	 * @return Default ModeIndex
	 */
	virtual int getDefaultModeIndex() const = 0;

	/**
	 * @brief Get the failsafe mode for this vehicle type
	 * @return Failsafe ModeIndex
	 */
	virtual int getFailsafeModeIndex() const = 0;

	/**
	 * @brief Get available modes mask
	 * @return Bitmask of available OPERATION_MODE_* values
	 */
	virtual uint32_t getAvailableModesMask() const = 0;

	//========================================================================
	// Mode Transition
	//========================================================================

	/**
	 * @brief Check if mode transition should preserve setpoints
	 * @param from_mode Current mode index
	 * @param to_mode Requested mode index
	 * @return true if setpoints should be preserved
	 */
	virtual bool shouldPreserveSetpoints(int from_mode, int to_mode) const
	{
		(void)from_mode;
		(void)to_mode;
		return false;
	}

	/**
	 * @brief Check if mode transition requires reset
	 * @param from_mode Current mode index
	 * @param to_mode Requested mode index
	 * @return true if triplets should be reset
	 */
	virtual bool shouldResetTriplets(int from_mode, int to_mode) const
	{
		(void)from_mode;
		(void)to_mode;
		return true;
	}

	//========================================================================
	// Setpoint Generation
	//========================================================================

	/**
	 * @brief Get the setpoint update rate for this vehicle type
	 * @return Update rate in Hz
	 */
	virtual float getSetpointUpdateRate() const
	{
		return 50.0f;  // Default 50Hz
	}

	/**
	 * @brief Check if setpoint generation is needed for current mode
	 * @param operation_mode Current operation mode
	 * @return true if setpoints should be generated
	 */
	virtual bool needsSetpointGeneration(uint8_t operation_mode) const
	{
		(void)operation_mode;
		return true;
	}
};

} // namespace mode_manager_strategy
