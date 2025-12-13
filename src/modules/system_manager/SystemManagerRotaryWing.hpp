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
 * @file SystemManagerRotaryWing.hpp
 *
 * SystemManager subclass for rotary wing (multicopter) vehicles.
 * Handles multicopter specific command filtering, failsafe behavior,
 * and mode management.
 */

#pragma once

#include "SystemManagerBase.hpp"

/**
 * @class SystemManagerRotaryWing
 *
 * Rotary wing (multicopter) specific SystemManager implementation.
 *
 * Key behaviors:
 * - Supports all aerial commands except fixed-wing specific ones
 * - Uses RTL or land for failsafe
 * - Supports full range of flight modes
 * - Full altitude/GPS arming checks
 */
class SystemManagerRotaryWing : public SystemManagerBase
{
public:
	SystemManagerRotaryWing() : SystemManagerBase() {}
	~SystemManagerRotaryWing() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	const char* getVehicleTypeName() const override
	{
		return "Rotary Wing";
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(const vehicle_command_s &cmd) const override
	{
		// Reject fixed-wing specific commands
		switch (cmd.command) {
		case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
			logCommandRejection(cmd, "fixed-wing only command");
			return true;

		default:
			return false;
		}
	}

	int getUnsupportedCommands(uint16_t* commands, int max_commands) const override
	{
		static const uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT,
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

		// If on ground, just hold
		if (current_state.vehicle_land_detected) {
			return FailsafeAction::Hold;
		}

		// Default failsafe for multicopters is RTL
		return FailsafeAction::RTL;
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::Hold:
		case FailsafeAction::RTL:
		case FailsafeAction::Land:
		case FailsafeAction::Descend:
		case FailsafeAction::Terminate:
		case FailsafeAction::ManualTakeover:
			return true;

		// Ground vehicle actions not supported
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
		// Standard multicopter checks are handled by health_and_arming_checks
		(void)reason;
		return true;
	}

	uint32_t getRequiredArmingChecksMask() const override
	{
		// Full arming checks for aerial vehicle
		return 0xFFFFFFFF;
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		// Multicopters support most modes except wheel loader specific
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ALTCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSITION_SLOW) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ORBIT) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_DESCEND);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_POSCTL;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_AUTO_RTL;
	}

	//========================================================================
	// State Management
	//========================================================================

	void initVehicleSpecific() override
	{
		PX4_INFO("SystemManagerRotaryWing initialized");
	}

	void updateVehicleSpecificState() override
	{
		// Standard multicopter state updates handled by base class
	}
};
