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
 * @file WheelLoaderCapabilities.hpp
 *
 * Vehicle capabilities for articulated wheel loader vehicles.
 *
 * Key characteristics:
 * - Ground-based operation only
 * - Articulated frame steering
 * - Boom and tilt actuators for bucket control
 * - VLA (Vision-Language-Action) autonomous mode
 * - No aerial modes (takeoff, land, RTL, etc.)
 */

#pragma once

#include "../VehicleCapabilities.hpp"

namespace vehicle
{

class WheelLoaderCapabilities : public VehicleCapabilities
{
public:
	//========================================================================
	// Identity
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char* getName() const override
	{
		return "Wheel Loader";
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		// Wheel loaders only support ground-based modes
		return ModeMask::MANUAL |
		       ModeMask::AUTO_VLA |
		       ModeMask::AUTO_MISSION |
		       ModeMask::OFFBOARD;
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		// Failsafe: stop in place (manual mode with zero inputs)
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	//========================================================================
	// Task Availability
	//========================================================================

	uint32_t getAvailableTasksMask() const override
	{
		// Wheel loaders support limited task set
		return TaskMask::VLA |
		       TaskMask::MISSION;
		// Note: No TAKEOFF, LAND, RTL, LOITER, PRECLAND, etc.
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(uint16_t cmd_id) const override
	{
		// Reject all aerial commands
		switch (cmd_id) {
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
		case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_TIME:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_TO_ALT:
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
			vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT,
			vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
			vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE,
			vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM,
			vehicle_command_s::VEHICLE_CMD_NAV_LOITER_TIME,
			vehicle_command_s::VEHICLE_CMD_NAV_LOITER_TO_ALT,
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
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::DatalinkLoss:
			return FailsafeAction::Hold;

		case FailsafeEvent::LowBattery:
			return FailsafeAction::Warn;

		case FailsafeEvent::CriticalBattery:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::GeofenceBreach:
			return FailsafeAction::Hold;

		case FailsafeEvent::PositionLoss:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::ObstacleDetected:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::SensorFailure:
			return FailsafeAction::EmergencyStop;

		case FailsafeEvent::VlaTimeout:
			return FailsafeAction::Hold;

		case FailsafeEvent::BoomLimit:
			return FailsafeAction::Hold;

		case FailsafeEvent::ManualOverride:
			return FailsafeAction::None;

		case FailsafeEvent::None:
		default:
			return FailsafeAction::None;
		}
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::Hold:
		case FailsafeAction::EmergencyStop:
		case FailsafeAction::Disarm:
			return true;

		// Aerial failsafe actions not supported
		case FailsafeAction::RTL:
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
			.boom_control = true,
			.tilt_control = true,
			.articulated_steering = true,
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
			.max_velocity = 5.0f,            // 5 m/s (18 km/h)
			.emergency_stop_decel = 3.0f,    // 3 m/s²
			.max_turn_rate = 0.5f,           // 0.5 rad/s
			.geofence_margin = 5.0f          // 5 m margin
		};
	}

	//========================================================================
	// Arming Checks
	//========================================================================

	uint32_t getRequiredArmingChecks() const override
	{
		// Wheel loaders don't need GPS or altitude-related checks
		// Return a mask that excludes airborne-specific checks
		uint32_t mask = 0xFFFFFFFF;

		// TODO: Define specific bits to disable
		// For now, require all checks but the implementation can
		// selectively disable checks in the health_and_arming module

		return mask;
	}

	bool vehicleSpecificPrearmCheck(const char** reason) const override
	{
		// TODO: Add wheel loader specific checks:
		// - Bucket position safe
		// - Boom position safe
		// - Articulation angle within limits
		// - Parking brake status

		(void)reason;
		return true;
	}
};

}  // namespace vehicle
