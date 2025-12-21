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
 * @file vehicle_command_handler.hpp
 *
 * Abstract interface for vehicle-specific command handling.
 *
 * This provides an extensible way to handle vehicle_command_s for different
 * vehicle types. Each vehicle type implements this interface to define how
 * commands should be processed, what modes they trigger, and what automation
 * tasks they initiate.
 */

#pragma once

#include <stdint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/mode_change_request.h>
#include <uORB/topics/automation_task.h>
#include <uORB/topics/calibration_request.h>
#include <uORB/topics/reboot_request.h>
#include <uORB/topics/actuator_test_request.h>
#include <uORB/topics/prearm_check_request.h>
#include <uORB/topics/storage_request.h>
#include <uORB/topics/flight_termination_request.h>
#include <uORB/topics/set_home_request.h>

namespace command_processor
{

/**
 * @brief Result of command processing
 */
enum class CommandResult : uint8_t {
	Accepted = 0,            // Command accepted, processing complete
	Denied = 1,              // Command denied (permanent - not allowed)
	Unsupported = 2,         // Command not supported by this vehicle
	TemporarilyRejected = 3, // Command temporarily rejected (try again)
	RequiresModeChange = 4,  // Command requires mode change first
	RequiresAutomation = 5,  // Command triggers automation task
	RequiresArming = 6,      // Command requires arming action
	RequiresCalibration = 7, // Command requires calibration
	RequiresReboot = 8,      // Command requires reboot/shutdown
	RequiresStorage = 9,     // Command requires storage operation
	RequiresActuatorTest = 10, // Command requires actuator test
	RequiresPrearmCheck = 11,  // Command requires prearm check
	RequiresFlightTermination = 12, // Command requires flight termination
	RequiresSetHome = 13,    // Command requires set home
	Forwarded = 14,          // Command forwarded to another module
	InProgress = 15          // Command processing in progress
};

/**
 * @brief Command processing context
 *
 * Contains information needed during command processing and
 * outputs for mode changes and automation tasks.
 */
struct CommandContext {
	// Input: Current vehicle state
	uint8_t current_mode{0};
	bool is_armed{false};
	uint8_t vehicle_type{0};

	// Output: Mode change request (if CommandResult::RequiresModeChange)
	bool mode_change_requested{false};
	uint8_t requested_mode{0};
	uint8_t mode_change_source{mode_change_request_s::SOURCE_COMMAND};
	bool mode_allow_fallback{false};
	bool mode_force{false};

	// Output: Automation task (if CommandResult::RequiresAutomation)
	bool automation_task_requested{false};
	automation_task_s automation_task{};

	// Output: Arming action (if CommandResult::RequiresArming)
	bool arm_requested{false};
	bool disarm_requested{false};
	bool arm_force{false};

	// Output: Calibration request (if CommandResult::RequiresCalibration)
	bool calibration_requested{false};
	calibration_request_s calibration_request{};

	// Output: Reboot request (if CommandResult::RequiresReboot)
	bool reboot_requested{false};
	reboot_request_s reboot_request{};

	// Output: Storage request (if CommandResult::RequiresStorage)
	bool storage_requested{false};
	storage_request_s storage_request{};

	// Output: Actuator test request (if CommandResult::RequiresActuatorTest)
	bool actuator_test_requested{false};
	actuator_test_request_s actuator_test_request{};

	// Output: Prearm check request (if CommandResult::RequiresPrearmCheck)
	bool prearm_check_requested{false};
	prearm_check_request_s prearm_check_request{};

	// Output: Flight termination request (if CommandResult::RequiresFlightTermination)
	bool flight_termination_requested{false};
	flight_termination_request_s flight_termination_request{};

	// Output: Set home request (if CommandResult::RequiresSetHome)
	bool set_home_requested{false};
	set_home_request_s set_home_request{};

	// Command ACK result to return
	uint8_t ack_result{0};  // VEHICLE_CMD_RESULT_*
};

/**
 * @brief Abstract interface for vehicle-specific command handling
 *
 * Each vehicle type implements this interface to define how commands
 * are processed. The CommandProcessor uses this interface to dispatch
 * commands to the appropriate handler.
 */
class VehicleCommandHandler
{
public:
	virtual ~VehicleCommandHandler() = default;

	/**
	 * @brief Get the vehicle type this handler supports
	 * @return Vehicle type constant from vehicle_status_s
	 */
	virtual uint8_t getVehicleType() const = 0;

	/**
	 * @brief Get the handler name for logging
	 * @return Handler name string
	 */
	virtual const char *getName() const = 0;

	/**
	 * @brief Check if this handler supports a specific command
	 *
	 * @param command Command ID (VEHICLE_CMD_*)
	 * @return true if command is supported by this handler
	 */
	virtual bool supportsCommand(uint16_t command) const = 0;

	/**
	 * @brief Check if a command should be rejected for this vehicle type
	 *
	 * Some commands are not applicable to certain vehicle types
	 * (e.g., TAKEOFF for ground vehicles without lift capability).
	 *
	 * @param cmd The vehicle command
	 * @return true if command should be rejected
	 */
	virtual bool shouldRejectCommand(const vehicle_command_s &cmd) const = 0;

	/**
	 * @brief Process a vehicle command
	 *
	 * Main entry point for command processing. The handler examines the
	 * command and determines:
	 * - Whether to accept/reject it
	 * - What mode change is needed (if any)
	 * - What automation task to trigger (if any)
	 * - What arming action is needed (if any)
	 *
	 * @param cmd The vehicle command to process
	 * @param ctx Command context (input state and output requests)
	 * @return CommandResult indicating how command was processed
	 */
	virtual CommandResult processCommand(const vehicle_command_s &cmd,
					     CommandContext &ctx) const = 0;

	/**
	 * @brief Get the target operation mode for a navigation command
	 *
	 * Maps navigation commands to their target modes.
	 * E.g., VEHICLE_CMD_NAV_TAKEOFF -> OPERATION_MODE_AUTO_TAKEOFF
	 *
	 * @param command Command ID
	 * @return Target OPERATION_MODE_*, or OPERATION_MODE_MAX if no mode change
	 */
	virtual uint8_t getTargetModeForCommand(uint16_t command) const = 0;

	/**
	 * @brief Get the automation task type for a command
	 *
	 * Maps commands to automation task types.
	 * E.g., VEHICLE_CMD_NAV_TAKEOFF -> TASK_TAKEOFF
	 *
	 * @param command Command ID
	 * @return TASK_* constant, or TASK_NONE if no task
	 */
	virtual uint8_t getAutomationTaskForCommand(uint16_t command) const = 0;

protected:
	/**
	 * @brief Helper to fill automation task from command
	 */
	static void fillAutomationTask(automation_task_s &task,
				       const vehicle_command_s &cmd,
				       uint8_t task_type)
	{
		task.timestamp = cmd.timestamp;
		task.task_type = task_type;
		task.param1 = cmd.param1;
		task.param2 = cmd.param2;
		task.param3 = cmd.param3;
		task.param4 = cmd.param4;
		task.latitude = cmd.param5;
		task.longitude = cmd.param6;
		task.altitude = cmd.param7;
		task.mission_item_index = -1;
		task.cmd_source_system = cmd.source_system;
		task.cmd_source_component = cmd.source_component;
		task.cmd_command = cmd.command;
		task.from_external = cmd.from_external;
		task.force = false;
	}

	/**
	 * @brief Helper to set mode change in context
	 */
	static void requestModeChange(CommandContext &ctx,
				      uint8_t mode,
				      bool allow_fallback = false,
				      bool force = false)
	{
		ctx.mode_change_requested = true;
		ctx.requested_mode = mode;
		ctx.mode_allow_fallback = allow_fallback;
		ctx.mode_force = force;
	}
};

} // namespace command_processor
