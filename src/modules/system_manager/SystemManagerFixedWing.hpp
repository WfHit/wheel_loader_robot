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
 * @file SystemManagerFixedWing.hpp
 *
 * SystemManager subclass for fixed wing vehicles.
 * Handles fixed wing specific command filtering, failsafe behavior,
 * and mode management.
 */

#pragma once

#include "SystemManagerBase.hpp"

/**
 * @class SystemManagerFixedWing
 *
 * Fixed wing specific SystemManager implementation.
 *
 * Key behaviors:
 * - Supports fixed-wing flight modes
 * - RTL with circle loiter for failsafe
 * - No precision landing support (typically)
 * - Altitude-based arming checks
 */
class SystemManagerFixedWing : public SystemManagerBase
{
public:
	SystemManagerFixedWing() : SystemManagerBase() {}
	~SystemManagerFixedWing() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	}

	const char* getVehicleTypeName() const override
	{
		return "Fixed Wing";
	}

	//========================================================================
	// Command Handling
	//========================================================================

	bool shouldRejectCommand(const vehicle_command_s &cmd) const override
	{
		switch (cmd.command) {
		// Precision landing typically not supported for fixed wing
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
			logCommandRejection(cmd, "precision landing not supported");
			return true;

		// Wheel loader specific commands
		// (These would be boom/bucket commands when defined)

		default:
			return false;
		}
	}

	int getUnsupportedCommands(uint16_t* commands, int max_commands) const override
	{
		static const uint16_t unsupported[] = {
			vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND,
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

		// Fixed wings typically RTL and loiter (circle) on failsafe
		return FailsafeAction::RTL;
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::RTL:
		case FailsafeAction::Land:
		case FailsafeAction::Terminate:
		case FailsafeAction::ManualTakeover:
			return true;

		// Hold/hover not possible for fixed wing
		case FailsafeAction::Hold:
		case FailsafeAction::Descend:
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
		// Fixed wing specific checks
		// - Airspeed sensor check
		// - Control surfaces check
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
		// Fixed wing modes
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ALTCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ORBIT) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
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
	// State Management
	//========================================================================

	void initVehicleSpecific() override
	{
		PX4_INFO("SystemManagerFixedWing initialized");
	}

	void updateVehicleSpecificState() override
	{
		// Fixed wing specific state updates
		// - Airspeed monitoring
		// - Stall detection
	}
};
