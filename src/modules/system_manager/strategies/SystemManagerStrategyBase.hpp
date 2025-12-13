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
 * @file SystemManagerStrategyBase.hpp
 *
 * Base class for SystemManager vehicle-type-specific strategies
 *
 * Defines the interface for vehicle-specific behavior in SystemManager:
 * - Command handling and validation
 * - Arming/disarming logic
 * - Failsafe behavior
 * - State transition validation
 */

#pragma once

#include <lib/vehicle_strategy/VehicleStrategyTypes.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <stdint.h>

namespace system_manager_strategy
{

using namespace vehicle_strategy;

/**
 * @brief Base class for SystemManager vehicle-type-specific strategies
 *
 * Each vehicle type implements this interface to define how SystemManager
 * should handle commands, arming, failsafe, and state transitions for that
 * vehicle type.
 */
class SystemManagerStrategyBase
{
public:
	virtual ~SystemManagerStrategyBase() = default;

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
	// Command Handling
	//========================================================================

	/**
	 * @brief Check if a command should be rejected for this vehicle type
	 * @param cmd The vehicle command to check
	 * @return true if the command should be rejected
	 */
	virtual bool shouldRejectCommand(const vehicle_command_s& cmd) const = 0;

	/**
	 * @brief Handle a vehicle command (vehicle-specific logic)
	 * @param cmd The vehicle command to handle
	 * @return CommandResult indicating how the command was handled
	 */
	virtual CommandResult handleCommand(const vehicle_command_s& cmd) const
	{
		(void)cmd;
		return CommandResult::Delegated;
	}

	/**
	 * @brief Get the list of unsupported commands for this vehicle type
	 * @param commands Output array of command IDs
	 * @param max_count Maximum number of commands to return
	 * @return Number of unsupported commands
	 */
	virtual size_t getUnsupportedCommands(uint16_t* commands, size_t max_count) const
	{
		(void)commands;
		(void)max_count;
		return 0;
	}

	//========================================================================
	// Arming Logic
	//========================================================================

	/**
	 * @brief Check if arming is allowed in current state
	 * @param vehicle_status Current vehicle status
	 * @return ArmingResult indicating if arming is allowed
	 */
	virtual ArmingResult checkArming(const vehicle_status_s& vehicle_status) const
	{
		(void)vehicle_status;
		return ArmingResult::Allowed;
	}

	/**
	 * @brief Check if disarming is allowed in current state
	 * @param vehicle_status Current vehicle status
	 * @param force Force disarm requested
	 * @return true if disarming is allowed
	 */
	virtual bool checkDisarming(const vehicle_status_s& vehicle_status, bool force) const
	{
		(void)vehicle_status;
		(void)force;
		return true;
	}

	/**
	 * @brief Get vehicle-specific arming checks to perform
	 * @return Bitmask of additional arming checks
	 */
	virtual uint32_t getAdditionalArmingChecks() const
	{
		return 0;
	}

	//========================================================================
	// Failsafe Handling
	//========================================================================

	/**
	 * @brief Get the failsafe action for a specific event
	 * @param event The failsafe event
	 * @param vehicle_status Current vehicle status
	 * @return FailsafeAction to take
	 */
	virtual FailsafeAction getFailsafeAction(FailsafeEvent event,
						 const vehicle_status_s& vehicle_status) const = 0;

	/**
	 * @brief Check if emergency stop should be triggered
	 * @param vehicle_status Current vehicle status
	 * @return true if emergency stop should be triggered
	 */
	virtual bool shouldEmergencyStop(const vehicle_status_s& vehicle_status) const
	{
		(void)vehicle_status;
		return false;
	}

	/**
	 * @brief Get the failsafe operation mode for this vehicle
	 * @return OPERATION_MODE_* constant for failsafe
	 */
	virtual uint8_t getFailsafeMode() const = 0;

	//========================================================================
	// State Transition Validation
	//========================================================================

	/**
	 * @brief Check if a mode transition is allowed
	 * @param current_mode Current operation mode
	 * @param requested_mode Requested operation mode
	 * @param vehicle_status Current vehicle status
	 * @return true if transition is allowed
	 */
	virtual bool isModeTransitionAllowed(uint8_t current_mode, uint8_t requested_mode,
					     const vehicle_status_s& vehicle_status) const
	{
		(void)current_mode;
		(void)requested_mode;
		(void)vehicle_status;
		return true;
	}

	/**
	 * @brief Get allowed modes for this vehicle type
	 * @return Bitmask of allowed OPERATION_MODE_* values
	 */
	virtual uint32_t getAllowedModesMask() const = 0;

	//========================================================================
	// RC Input Handling
	//========================================================================

	/**
	 * @brief Check if RC input is required for arming
	 * @return true if RC input is required
	 */
	virtual bool isRcRequiredForArming() const
	{
		return true;
	}

	/**
	 * @brief Get the RC input mode for this vehicle
	 * @return RC_INPUT_MODE_* constant
	 */
	virtual uint8_t getRcInputMode() const = 0;
};

} // namespace system_manager_strategy
