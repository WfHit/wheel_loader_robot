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
 * @file SystemManagerWheelLoader.hpp
 *
 * SystemManager subclass for wheel loader vehicles.
 * Handles articulated wheel loader specific command filtering,
 * failsafe behavior, and mode management.
 */

#pragma once

#include "SystemManagerBase.hpp"

/**
 * @class SystemManagerWheelLoader
 *
 * Wheel loader specific SystemManager implementation.
 *
 * Key behaviors:
 * - Rejects aerial commands (takeoff, land, orbit, etc.)
 * - Uses emergency stop for failsafe instead of RTL
 * - Supports only ground-based operation modes (Manual, VLA)
 * - No altitude-based arming checks
 */
class SystemManagerWheelLoader : public SystemManagerBase
{
public:
	SystemManagerWheelLoader() : SystemManagerBase() {}
	~SystemManagerWheelLoader() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char* getVehicleTypeName() const override
	{
		return "Wheel Loader";
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
			logCommandRejection(cmd, "aerial command not supported");
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

		// Wheel loaders use emergency stop (brake) for failsafe
		// No RTL or land options for ground vehicles
		return FailsafeAction::EmergencyStop;
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::Hold:
		case FailsafeAction::EmergencyStop:
		case FailsafeAction::ManualTakeover:
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
	// Arming Logic
	//========================================================================

	bool vehicleSpecificPrearmCheck(const char** reason) const override
	{
		// TODO: Add wheel loader specific checks
		// - Bucket position safe
		// - Boom position safe
		// - Articulation angle neutral
		// - Parking brake engaged (for some modes)

		(void)reason;
		return true;
	}

	uint32_t getRequiredArmingChecksMask() const override
	{
		// Wheel loaders don't need altitude/GPS checks for basic operation
		// Remove airborne-related checks
		uint32_t mask = 0xFFFFFFFF;

		// Disable checks that don't apply to ground vehicles
		// These would be specific bit flags from health_and_arming_checks
		// For now, return all checks enabled

		return mask;
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		// Wheel loaders only support manual and VLA autonomous modes
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_VLA) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		// Wheel loaders go to manual on failsafe (emergency stop handled separately)
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	//========================================================================
	// State Management
	//========================================================================

	void initVehicleSpecific() override
	{
		PX4_INFO("SystemManagerWheelLoader initialized");
	}

	void updateVehicleSpecificState() override
	{
		// TODO: Update wheel loader specific state
		// - Monitor boom/bucket positions
		// - Check articulation limits
		// - Monitor hydraulic pressure
	}
};
