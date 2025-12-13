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
 * @file SystemManagerStrategyWheelLoader.hpp
 *
 * SystemManager strategy for wheel loader vehicles
 */

#pragma once

#include "SystemManagerStrategyBase.hpp"
#include <uORB/topics/vehicle_type_config.h>

namespace system_manager_strategy
{

class SystemManagerStrategyWheelLoader : public SystemManagerStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::WheelLoader;
	}

	const char* getName() const override
	{
		return "WheelLoader SystemManager Strategy";
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(const vehicle_command_s& cmd) const override
	{
		// Wheel loaders don't support aerial commands
		switch (cmd.command) {
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
			return true;

		default:
			return false;
		}
	}

	size_t getUnsupportedCommands(uint16_t* commands, size_t max_count) const override
	{
		const uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_NAV_LAND,
			vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND,
			vehicle_command_s::VEHICLE_CMD_DO_ORBIT,
			vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT,
			vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
		};

		size_t count = sizeof(unsupported) / sizeof(unsupported[0]);
		if (count > max_count) {
			count = max_count;
		}

		for (size_t i = 0; i < count; i++) {
			commands[i] = unsupported[i];
		}

		return count;
	}

	//========================================================================
	// Arming Logic
	//========================================================================

	ArmingResult checkArming(const vehicle_status_s& vehicle_status) const override
	{
		// Wheel loader specific arming checks
		// TODO: Add boom position check, bucket position check, etc.
		(void)vehicle_status;
		return ArmingResult::Allowed;
	}

	bool checkDisarming(const vehicle_status_s& vehicle_status, bool force) const override
	{
		// Allow disarm if not moving or force requested
		(void)vehicle_status;
		(void)force;
		return true;
	}

	//========================================================================
	// Failsafe Handling
	//========================================================================

	FailsafeAction getFailsafeAction(FailsafeEvent event,
					 const vehicle_status_s& vehicle_status) const override
	{
		(void)vehicle_status;

		switch (event) {
		case FailsafeEvent::RcLoss:
			// Ground vehicle: emergency stop on RC loss
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::DatalinkLoss:
			// Continue current task, operator may be nearby
			return FailsafeAction::Warn;

		case FailsafeEvent::LowBattery:
			return FailsafeAction::Warn;

		case FailsafeEvent::CriticalBattery:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::GeofenceBreach:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::ObstacleDetected:
			return FailsafeAction::Hold;

		case FailsafeEvent::PositionLoss:
			// Ground vehicle: stop if position lost
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::VlaTimeout:
			// Return to manual mode on VLA timeout
			return FailsafeAction::Hold;

		case FailsafeEvent::SensorFailure:
		case FailsafeEvent::MotorFailure:
			return FailsafeAction::EmergencyStop;

		default:
			return FailsafeAction::Warn;
		}
	}

	bool shouldEmergencyStop(const vehicle_status_s& vehicle_status) const override
	{
		// Emergency stop if critical failsafe active
		(void)vehicle_status;
		return false;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	//========================================================================
	// State Transition Validation
	//========================================================================

	bool isModeTransitionAllowed(uint8_t current_mode, uint8_t requested_mode,
				     const vehicle_status_s& vehicle_status) const override
	{
		(void)current_mode;
		(void)vehicle_status;

		// Only allow manual and VLA modes
		return (requested_mode == vehicle_status_s::OPERATION_MODE_MANUAL) ||
		       (requested_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA);
	}

	uint32_t getAllowedModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_VLA);
	}

	//========================================================================
	// RC Input Handling
	//========================================================================

	bool isRcRequiredForArming() const override
	{
		return true;
	}

	uint8_t getRcInputMode() const override
	{
		return vehicle_type_config_s::RC_INPUT_MODE_WHEEL_LOADER;
	}
};

} // namespace system_manager_strategy
