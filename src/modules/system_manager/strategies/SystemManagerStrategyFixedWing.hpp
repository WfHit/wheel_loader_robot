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
 * @file SystemManagerStrategyFixedWing.hpp
 *
 * SystemManager strategy for fixed wing aircraft
 */

#pragma once

#include "SystemManagerStrategyBase.hpp"
#include <uORB/topics/vehicle_type_config.h>

namespace system_manager_strategy
{

class SystemManagerStrategyFixedWing : public SystemManagerStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::FixedWing;
	}

	const char* getName() const override
	{
		return "FixedWing SystemManager Strategy";
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(const vehicle_command_s& cmd) const override
	{
		switch (cmd.command) {
		// Precision landing not typically supported for fixed wing
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
			return true;

		// Ground vehicle commands - not supported
		// (Add when boom/bucket commands are defined)

		default:
			return false;
		}
	}

	size_t getUnsupportedCommands(uint16_t* commands, size_t max_count) const override
	{
		const uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND,
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
		(void)vehicle_status;
		return ArmingResult::Allowed;
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
			return FailsafeAction::ReturnToLaunch;

		case FailsafeEvent::DatalinkLoss:
			return FailsafeAction::ReturnToLaunch;

		case FailsafeEvent::LowBattery:
			return FailsafeAction::ReturnToLaunch;

		case FailsafeEvent::CriticalBattery:
			return FailsafeAction::Land;

		case FailsafeEvent::GeofenceBreach:
			return FailsafeAction::ReturnToLaunch;

		case FailsafeEvent::ObstacleDetected:
			// Fixed wing can't really stop
			return FailsafeAction::Warn;

		case FailsafeEvent::PositionLoss:
			return FailsafeAction::Descend;

		case FailsafeEvent::SensorFailure:
		case FailsafeEvent::MotorFailure:
			return FailsafeAction::Land;

		default:
			return FailsafeAction::Warn;
		}
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_DESCEND;
	}

	//========================================================================
	// State Transition Validation
	//========================================================================

	uint32_t getAllowedModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1u << vehicle_status_s::OPERATION_MODE_STAB) |
		       (1u << vehicle_status_s::OPERATION_MODE_ALTCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
		       (1u << vehicle_status_s::OPERATION_MODE_OFFBOARD) |
		       (1u << vehicle_status_s::OPERATION_MODE_DESCEND);
	}

	//========================================================================
	// RC Input Handling
	//========================================================================

	uint8_t getRcInputMode() const override
	{
		return vehicle_type_config_s::RC_INPUT_MODE_STANDARD;
	}
};

} // namespace system_manager_strategy
