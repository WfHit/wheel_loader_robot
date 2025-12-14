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
 * @file RoverCapabilities.hpp
 *
 * Vehicle capabilities for ground rover vehicles.
 *
 * Key characteristics:
 * - Ground-based operation only
 * - Position and velocity control
 * - No aerial modes
 * - Mission navigation support
 */

#pragma once

#include "../VehicleCapabilities.hpp"

namespace vehicle
{

class RoverCapabilities : public VehicleCapabilities
{
public:
	//========================================================================
	// Identity
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROVER;
	}

	const char* getName() const override
	{
		return "Rover";
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		return ModeMask::MANUAL |
		       ModeMask::POSITION |
		       ModeMask::AUTO_MISSION |
		       ModeMask::AUTO_RTL |
		       ModeMask::OFFBOARD;
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_AUTO_RTL;
	}

	//========================================================================
	// Task Availability
	//========================================================================

	uint32_t getAvailableTasksMask() const override
	{
		return TaskMask::MISSION |
		       TaskMask::RTL |
		       TaskMask::LOITER;
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(uint16_t cmd_id) const override
	{
		switch (cmd_id) {
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
		case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
			return true;

		default:
			return false;
		}
	}

	size_t getUnsupportedCommands(uint16_t* cmds, size_t max) const override
	{
		static constexpr uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_NAV_LAND,
			vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND,
			vehicle_command_s::VEHICLE_CMD_DO_ORBIT,
			vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
			vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE,
		};

		size_t count = sizeof(unsupported) / sizeof(unsupported[0]);
		count = (count > max) ? max : count;

		for (size_t i = 0; i < count; i++) {
			cmds[i] = unsupported[i];
		}

		return count;
	}

	//========================================================================
	// Failsafe Behavior
	//========================================================================

	FailsafeAction getFailsafeAction(FailsafeEvent event) const override
	{
		switch (event) {
		case FailsafeEvent::RcLoss:
			return FailsafeAction::Hold;

		case FailsafeEvent::DatalinkLoss:
			return FailsafeAction::Hold;

		case FailsafeEvent::LowBattery:
			return FailsafeAction::RTL;

		case FailsafeEvent::CriticalBattery:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::GeofenceBreach:
			return FailsafeAction::RTL;

		case FailsafeEvent::PositionLoss:
			return FailsafeAction::EmergencyStop;

		default:
			return FailsafeAction::Hold;
		}
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::Hold:
		case FailsafeAction::RTL:
		case FailsafeAction::EmergencyStop:
		case FailsafeAction::Disarm:
			return true;

		case FailsafeAction::Land:
		case FailsafeAction::Descend:
		case FailsafeAction::Terminate:
		default:
			return false;
		}
	}

	//========================================================================
	// Control Capabilities
	//========================================================================

	ControlCapabilities getControlCapabilities() const override
	{
		return ControlCapabilities{
			.altitude_control = false,
			.position_control = true,
			.velocity_control = true,
			.attitude_control = false,
			.boom_control = false,
			.tilt_control = false,
			.articulated_steering = false,
			.vertical_takeoff = false,
			.landing_gear = false,
			.vtol_transition = false
		};
	}

	//========================================================================
	// Physical Limits
	//========================================================================

	SafetyLimits getSafetyLimits() const override
	{
		return SafetyLimits{
			.max_velocity = 10.0f,           // 10 m/s
			.emergency_stop_decel = 5.0f,    // 5 m/s²
			.max_turn_rate = 1.0f,           // 1 rad/s
			.geofence_margin = 3.0f          // 3 m margin
		};
	}

	//========================================================================
	// Arming Checks
	//========================================================================

	uint32_t getRequiredArmingChecks() const override
	{
		return 0xFFFFFFFF;  // All checks enabled
	}
};

}  // namespace vehicle
