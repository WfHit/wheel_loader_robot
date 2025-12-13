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
 * @file SystemManagerRover.hpp
 *
 * SystemManager subclass for rover (ground) vehicles.
 * Handles rover specific command filtering, failsafe behavior,
 * and mode management.
 */

#pragma once

#include "SystemManagerBase.hpp"

/**
 * @class SystemManagerRover
 *
 * Rover (ground vehicle) specific SystemManager implementation.
 *
 * Key behaviors:
 * - Rejects all aerial commands
 * - Uses hold/stop for failsafe
 * - Supports ground vehicle modes (manual, mission, RTL to start)
 * - Simplified arming checks (no altitude requirements)
 */
class SystemManagerRover : public SystemManagerBase
{
public:
	SystemManagerRover() : SystemManagerBase() {}
	~SystemManagerRover() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROVER;
	}

	const char* getVehicleTypeName() const override
	{
		return "Rover";
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(const vehicle_command_s &cmd) const override
	{
		// Reject all aerial commands
		switch (cmd.command) {
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
			logCommandRejection(cmd, "aerial command not supported for rover");
			return true;

		default:
			return false;
		}
	}

	int getUnsupportedCommands(uint16_t* commands, int max_commands) const override
	{
		static const uint16_t unsupported[] = {
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
		};

		int count = sizeof(unsupported) / sizeof(unsupported[0]);
		count = (count > max_commands) ? max_commands : count;

		for (int i = 0; i < count; i++) {
			commands[i] = unsupported[i];
		}

		return count;
	}

	//========================================================================
	// Failsafe Behavior
	//========================================================================

	FailsafeAction getFailsafeAction(uint8_t event, const vehicle_status_s& current_state) const override
	{
		(void)event;
		(void)current_state;

		// Rovers stop in place on failsafe
		return FailsafeAction::Hold;
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::Hold:
		case FailsafeAction::ManualTakeover:
			return true;

		// Rovers can potentially RTL (drive back to start)
		case FailsafeAction::RTL:
			return true;

		// Aerial failsafe actions not supported
		case FailsafeAction::Land:
		case FailsafeAction::Descend:
		case FailsafeAction::Terminate:
		case FailsafeAction::EmergencyStop:
		default:
			return false;
		}
	}

	//========================================================================
	// Arming Logic
	//========================================================================

	bool vehicleSpecificPrearmCheck(const char** reason) const override
	{
		// Rover specific checks could include:
		// - Wheel encoder check
		// - Steering servo check
		(void)reason;
		return true;
	}

	uint32_t getRequiredArmingChecksMask() const override
	{
		// Rovers don't need altitude/airspeed checks
		uint32_t mask = 0xFFFFFFFF;
		// Could disable specific aerial-related checks here
		return mask;
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		// Rover modes - ground vehicle operation
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		// Rovers hold position on failsafe
		return vehicle_status_s::OPERATION_MODE_AUTO_LOITER;
	}

	//========================================================================
	// State Management
	//========================================================================

	void initVehicleSpecific() override
	{
		PX4_INFO("SystemManagerRover initialized");
	}

	void updateVehicleSpecificState() override
	{
		// Rover specific state updates
		// - Wheel speed monitoring
		// - Terrain detection
	}
};
