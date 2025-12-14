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
 * @file RotaryWingCapabilities.hpp
 *
 * Vehicle capabilities for rotary wing (multicopter/helicopter) vehicles.
 *
 * Key characteristics:
 * - Full 3D flight capability
 * - Vertical takeoff and landing
 * - Position, altitude, and velocity control
 * - All flight modes available
 */

#pragma once

#include "../VehicleCapabilities.hpp"

namespace vehicle
{

class RotaryWingCapabilities : public VehicleCapabilities
{
public:
	//========================================================================
	// Identity
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	const char* getName() const override
	{
		return "Rotary Wing";
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		return ModeMask::MANUAL |
		       ModeMask::ALTITUDE |
		       ModeMask::POSITION |
		       ModeMask::STABILIZED |
		       ModeMask::ACRO |
		       ModeMask::AUTO_MISSION |
		       ModeMask::AUTO_LOITER |
		       ModeMask::AUTO_RTL |
		       ModeMask::AUTO_TAKEOFF |
		       ModeMask::AUTO_LAND |
		       ModeMask::OFFBOARD;
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_POSITION;
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
		       TaskMask::LOITER |
		       TaskMask::RTL |
		       TaskMask::TAKEOFF |
		       TaskMask::LAND |
		       TaskMask::PRECLAND |
		       TaskMask::ORBIT |
		       TaskMask::DESCEND;
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(uint16_t cmd_id) const override
	{
		// Rotary wing supports most commands
		switch (cmd_id) {
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
			return true;  // Not a VTOL

		default:
			return false;
		}
	}

	size_t getUnsupportedCommands(uint16_t* cmds, size_t max) const override
	{
		static constexpr uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
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
			return FailsafeAction::RTL;

		case FailsafeEvent::DatalinkLoss:
			return FailsafeAction::RTL;

		case FailsafeEvent::LowBattery:
			return FailsafeAction::RTL;

		case FailsafeEvent::CriticalBattery:
			return FailsafeAction::Land;

		case FailsafeEvent::GeofenceBreach:
			return FailsafeAction::RTL;

		case FailsafeEvent::PositionLoss:
			return FailsafeAction::Descend;

		case FailsafeEvent::ObstacleDetected:
			return FailsafeAction::Hold;

		case FailsafeEvent::SensorFailure:
			return FailsafeAction::Land;

		default:
			return FailsafeAction::Hold;
		}
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		// Rotary wing supports all failsafe actions except terminate
		switch (action) {
		case FailsafeAction::Terminate:
			return false;

		default:
			return true;
		}
	}

	//========================================================================
	// Control Capabilities
	//========================================================================

	ControlCapabilities getControlCapabilities() const override
	{
		return ControlCapabilities{
			.altitude_control = true,
			.position_control = true,
			.velocity_control = true,
			.attitude_control = true,
			.boom_control = false,
			.tilt_control = false,
			.articulated_steering = false,
			.vertical_takeoff = true,
			.landing_gear = true,
			.vtol_transition = false
		};
	}

	//========================================================================
	// Physical Limits
	//========================================================================

	SafetyLimits getSafetyLimits() const override
	{
		return SafetyLimits{
			.max_velocity = 20.0f,           // 20 m/s
			.emergency_stop_decel = 10.0f,   // 10 m/s²
			.max_turn_rate = 3.14f,          // π rad/s
			.geofence_margin = 10.0f         // 10 m margin
		};
	}

	//========================================================================
	// Arming Checks
	//========================================================================

	uint32_t getRequiredArmingChecks() const override
	{
		return 0xFFFFFFFF;  // All checks required
	}
};

}  // namespace vehicle
