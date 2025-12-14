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
 * @brief SystemManager subclass for wheel loader vehicles
 *
 * This class manages system-level operations for articulated wheel loaders,
 * including command filtering, failsafe behavior, arming logic, and mode
 * management.
 *
 * Command handling:
 * - Rejects all aerial commands (takeoff, land, orbit, etc.)
 * - Accepts ground navigation and wheel loader specific commands
 *
 * Failsafe behavior:
 * - Emergency stop (apply brakes, stop all motion)
 * - No RTL/Land options (ground vehicle)
 * - Manual takeover for operator intervention
 *
 * Arming:
 * - Simplified checks (no altitude/GPS requirements for basic operation)
 * - Future: bucket/boom position checks, hydraulic pressure verification
 */

#pragma once

#include "SystemManagerBase.hpp"

/**
 * @class SystemManagerWheelLoader
 * @brief Wheel loader specific SystemManager implementation
 *
 * Key characteristics:
 * - Ground-only operation modes
 * - Emergency stop as primary failsafe
 * - Aerial command rejection
 * - Simplified arming requirements
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
		switch (cmd.command) {
		// Aerial navigation commands - reject
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_TIME:
			logCommandRejection(cmd, "aerial navigation not supported");
			return true;

		// Aerial maneuver commands - reject
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
		case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
			logCommandRejection(cmd, "aerial maneuver not supported");
			return true;

		default:
			return false;
		}
	}

	int getUnsupportedCommands(uint16_t* commands, int max_commands) const override
	{
		static constexpr uint16_t kUnsupportedCommands[] = {
			vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF,
			vehicle_command_s::VEHICLE_CMD_NAV_LAND,
			vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND,
			vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM,
			vehicle_command_s::VEHICLE_CMD_NAV_LOITER_TIME,
			vehicle_command_s::VEHICLE_CMD_DO_ORBIT,
			vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT,
			vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
			vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE,
		};

		constexpr int kCount = sizeof(kUnsupportedCommands) / sizeof(kUnsupportedCommands[0]);
		const int copy_count = (kCount > max_commands) ? max_commands : kCount;

		for (int i = 0; i < copy_count; i++) {
			commands[i] = kUnsupportedCommands[i];
		}

		return copy_count;
	}

	//========================================================================
	// Failsafe Behavior
	//========================================================================

	FailsafeAction getFailsafeAction(uint8_t event, const vehicle_status_s& current_state) const override
	{
		(void)event;
		(void)current_state;

		// Ground vehicle failsafe: apply brakes and stop
		return FailsafeAction::EmergencyStop;
	}

	bool supportsFailsafeAction(FailsafeAction action) const override
	{
		switch (action) {
		// Supported ground vehicle actions
		case FailsafeAction::None:
		case FailsafeAction::Warn:
		case FailsafeAction::Hold:
		case FailsafeAction::EmergencyStop:
		case FailsafeAction::ManualTakeover:
			return true;

		// Aerial failsafe actions - not supported
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
		// TODO: Implement wheel loader specific pre-arm checks
		// - Bucket in safe position
		// - Boom in safe position
		// - Articulation angle within limits
		// - Hydraulic pressure OK
		// - Parking brake engaged (for some modes)
		(void)reason;
		return true;
	}

	uint32_t getRequiredArmingChecksMask() const override
	{
		// Ground vehicle: disable airborne-related checks
		// Return all checks enabled for now; specific bits can be
		// disabled when health_and_arming_checks bit flags are defined
		return 0xFFFFFFFF;
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
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
		// TODO: Implement wheel loader state monitoring
		// - Boom/bucket position monitoring
		// - Articulation angle monitoring
		// - Hydraulic system status
		// - Load cell readings (if equipped)
	}
};
