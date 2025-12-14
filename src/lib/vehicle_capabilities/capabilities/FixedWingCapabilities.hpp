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
 * @file FixedWingCapabilities.hpp
 *
 * Vehicle capabilities for fixed wing aircraft.
 *
 * Key characteristics:
 * - Forward flight required for lift
 * - Runway takeoff/landing (non-VTOL)
 * - Altitude and position control
 * - Limited hovering capability (loiter only)
 */

#pragma once

#include "../VehicleCapabilities.hpp"

namespace vehicle
{

class FixedWingCapabilities : public VehicleCapabilities
{
public:
	//========================================================================
	// Identity
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	}

	const char* getName() const override
	{
		return "Fixed Wing";
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
		return vehicle_status_s::OPERATION_MODE_STABILIZED;
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
		       TaskMask::ORBIT;
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(uint16_t cmd_id) const override
	{
		switch (cmd_id) {
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
			return true;

		default:
			return false;
		}
	}

	size_t getUnsupportedCommands(uint16_t* cmds, size_t max) const override
	{
		static constexpr uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
			vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND,
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

		default:
			return FailsafeAction::RTL;
		}
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::EmergencyStop:
			return false;  // Fixed wing can't stop in place

		case FailsafeAction::Hold:
			return false;  // Fixed wing can't hover

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
			.vertical_takeoff = false,
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
			.max_velocity = 50.0f,           // 50 m/s (180 km/h)
			.emergency_stop_decel = 0.0f,    // Can't stop in place
			.max_turn_rate = 0.5f,           // 0.5 rad/s (bank limited)
			.geofence_margin = 50.0f         // 50 m margin (larger for turning radius)
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
