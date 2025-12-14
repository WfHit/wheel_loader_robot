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

#include "WheelLoaderCommandHandler.hpp"

namespace command_processor
{

bool WheelLoaderCommandHandler::supportsCommand(uint16_t command) const
{
	switch (command) {
	// Mode change commands (same as standard)
	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
	case vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_STANDARD_MODE:

	// Ground vehicle navigation
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
	case vehicle_command_s::VEHICLE_CMD_MISSION_START:

	// Safety commands
	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:

	// Home position
	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME:
		return true;

	// Aerial commands - not supported but handled to reject
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		return true;  // We handle these to reject them

	default:
		return false;
	}
}

bool WheelLoaderCommandHandler::shouldRejectCommand(const vehicle_command_s &cmd) const
{
	switch (cmd.command) {
	// Aerial commands - reject for ground vehicle
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
	case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		return true;

	default:
		return false;
	}
}

CommandResult WheelLoaderCommandHandler::processCommand(const vehicle_command_s &cmd,
							CommandContext &ctx) const
{
	// Check for wheel loader specific commands first
	switch (cmd.command) {
	// Aerial commands - reject
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		return handleNavTakeoff(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		return handleNavLand(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		return handleNavRtl(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		return handleDoOrbit(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		return handleNavPrecland(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		return handleNavVtolTakeoff(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;

	// Override reposition for wheel loader (uses VLA mode)
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		return handleDoReposition(cmd, ctx);

	default:
		// Delegate to standard handler for other commands
		return StandardCommandHandler::processCommand(cmd, ctx);
	}
}

uint8_t WheelLoaderCommandHandler::getTargetModeForCommand(uint16_t command) const
{
	switch (command) {
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		// Wheel loader uses VLA mode for positioning
		return vehicle_status_s::OPERATION_MODE_AUTO_VLA;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		// Wheel loader uses VLA for mission execution too
		return vehicle_status_s::OPERATION_MODE_AUTO_VLA;

	// Aerial commands - no valid target mode
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		return vehicle_status_s::OPERATION_MODE_MAX;  // Indicates rejection

	default:
		return StandardCommandHandler::getTargetModeForCommand(command);
	}
}

uint8_t WheelLoaderCommandHandler::getAutomationTaskForCommand(uint16_t command) const
{
	switch (command) {
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		// Wheel loader uses VLA trajectory for reposition
		return automation_task_s::TASK_VLA_TRAJECTORY;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		return automation_task_s::TASK_MISSION;

	// Aerial commands - no task
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		return automation_task_s::TASK_NONE;

	default:
		return StandardCommandHandler::getAutomationTaskForCommand(command);
	}
}

// Reject aerial commands

CommandResult WheelLoaderCommandHandler::handleNavTakeoff(const vehicle_command_s &cmd,
							  CommandContext &ctx) const
{
	(void)cmd;
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	return CommandResult::Unsupported;
}

CommandResult WheelLoaderCommandHandler::handleNavLand(const vehicle_command_s &cmd,
						       CommandContext &ctx) const
{
	(void)cmd;
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	return CommandResult::Unsupported;
}

CommandResult WheelLoaderCommandHandler::handleNavRtl(const vehicle_command_s &cmd,
						      CommandContext &ctx) const
{
	(void)cmd;
	// For wheel loader, RTL means go back to manual mode (safe state)
	requestModeChange(ctx, vehicle_status_s::OPERATION_MODE_MANUAL);
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresModeChange;
}

CommandResult WheelLoaderCommandHandler::handleDoOrbit(const vehicle_command_s &cmd,
						       CommandContext &ctx) const
{
	(void)cmd;
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	return CommandResult::Unsupported;
}

CommandResult WheelLoaderCommandHandler::handleNavPrecland(const vehicle_command_s &cmd,
							   CommandContext &ctx) const
{
	(void)cmd;
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	return CommandResult::Unsupported;
}

CommandResult WheelLoaderCommandHandler::handleNavVtolTakeoff(const vehicle_command_s &cmd,
							      CommandContext &ctx) const
{
	(void)cmd;
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
	return CommandResult::Unsupported;
}

// Wheel loader specific implementations

CommandResult WheelLoaderCommandHandler::handleDoReposition(const vehicle_command_s &cmd,
							    CommandContext &ctx) const
{
	const uint32_t change_mode_flags = (uint32_t)cmd.param2;
	const bool mode_switch_not_requested = (change_mode_flags & 1) == 0;
	const bool unsupported_bits_set = (change_mode_flags & ~1) != 0;

	if (mode_switch_not_requested || unsupported_bits_set) {
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}

	// Wheel loader uses VLA mode for reposition
	requestModeChange(ctx, vehicle_status_s::OPERATION_MODE_AUTO_VLA);

	// Create VLA trajectory task
	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_VLA_TRAJECTORY);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult WheelLoaderCommandHandler::handleVlaTrajectory(const vehicle_command_s &cmd,
							     CommandContext &ctx) const
{
	// Switch to VLA mode
	requestModeChange(ctx, vehicle_status_s::OPERATION_MODE_AUTO_VLA);

	// Create VLA trajectory task
	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_VLA_TRAJECTORY);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

} // namespace command_processor
