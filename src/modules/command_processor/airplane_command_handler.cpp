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

#include "airplane_command_handler.hpp"
#include <lib/modes/standard_modes.hpp>

// MAVLink mode flags
#define VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define VEHICLE_MODE_FLAG_AUTO_ENABLED 4
#define VEHICLE_MODE_FLAG_GUIDED_ENABLED 8
#define VEHICLE_MODE_FLAG_STABILIZE_ENABLED 16
#define VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED 64

// PX4 custom modes
#define PX4_CUSTOM_MAIN_MODE_MANUAL 1
#define PX4_CUSTOM_MAIN_MODE_ALTCTL 2
#define PX4_CUSTOM_MAIN_MODE_POSCTL 3
#define PX4_CUSTOM_MAIN_MODE_AUTO 4
#define PX4_CUSTOM_MAIN_MODE_ACRO 5
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD 6
#define PX4_CUSTOM_MAIN_MODE_STABILIZED 7
#define PX4_CUSTOM_MAIN_MODE_TERMINATION 8

// PX4 custom sub modes
#define PX4_CUSTOM_SUB_MODE_AUTO_LOITER 3
#define PX4_CUSTOM_SUB_MODE_AUTO_MISSION 4
#define PX4_CUSTOM_SUB_MODE_AUTO_RTL 5
#define PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF 2
#define PX4_CUSTOM_SUB_MODE_AUTO_LAND 6
#define PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET 8
#define PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND 9
#define PX4_CUSTOM_SUB_MODE_EXTERNAL1 10
#define PX4_CUSTOM_SUB_MODE_EXTERNAL8 17
#define PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL 0
#define PX4_CUSTOM_SUB_MODE_POSCTL_SLOW 3

namespace command_processor
{

bool AirPlaneCommandHandler::supportsCommand(uint16_t command) const
{
	switch (command) {
	// Mode change commands
	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
	case vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE:
	case vehicle_command_s::VEHICLE_CMD_DO_SET_STANDARD_MODE:

	// Navigation commands
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
	case vehicle_command_s::VEHICLE_CMD_MISSION_START:
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
	case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:

	// Safety commands
	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:

	// Home position
	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME:

	// System commands - arming, calibration, reboot, etc.
	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE:
	case vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW:
	case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
	case vehicle_command_s::VEHICLE_CMD_RUN_PREARM_CHECKS:
		return true;

	default:
		return false;
	}
}

CommandResult AirPlaneCommandHandler::processCommand(const vehicle_command_s &cmd,
						     CommandContext &ctx) const
{
	switch (cmd.command) {
	// Mode change commands
	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
		return handleDoSetMode(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE:
		return handleSetNavState(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_SET_STANDARD_MODE:
		return handleDoSetStandardMode(cmd, ctx);

	// Navigation commands
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		return handleNavTakeoff(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		return handleNavLand(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		return handleNavRtl(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		return handleMissionStart(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		return handleDoReposition(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		return handleDoChangeAltitude(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		return handleDoOrbit(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		return handleNavPrecland(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		return handleNavVtolTakeoff(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		return handleDoFigureEight(cmd, ctx);

	// Safety commands
	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:
		return handleDoFlighttermination(cmd, ctx);

	// Home position
	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME:
		return handleDoSetHome(cmd, ctx);

	// System commands
	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
		return handleComponentArmDisarm(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION:
		return handlePreflightCalibration(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
		return handlePreflightRebootShutdown(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE:
		return handlePreflightStorage(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW:
		return handleFixedMagCalYaw(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
		return handleActuatorTest(cmd, ctx);

	case vehicle_command_s::VEHICLE_CMD_RUN_PREARM_CHECKS:
		return handleRunPrearmChecks(cmd, ctx);

	default:
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}
}

uint8_t AirPlaneCommandHandler::getTargetModeForCommand(uint16_t command) const
{
	switch (command) {
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		return mode_status_s::OPERATION_MODE_AUTO_TAKEOFF;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		return mode_status_s::OPERATION_MODE_AUTO_LAND;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		return mode_status_s::OPERATION_MODE_AUTO_RTL;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		return mode_status_s::OPERATION_MODE_AUTO_MISSION;

	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		return mode_status_s::OPERATION_MODE_AUTO_LOITER;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		return mode_status_s::OPERATION_MODE_ORBIT;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		return mode_status_s::OPERATION_MODE_AUTO_PRECLAND;

	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		return mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF;

	case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:
		return mode_status_s::OPERATION_MODE_TERMINATION;

	default:
		return mode_status_s::OPERATION_MODE_MAX;
	}
}

uint8_t AirPlaneCommandHandler::getAutomationTaskForCommand(uint16_t command) const
{
	switch (command) {
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		return automation_task_s::TASK_TAKEOFF;

	case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		return automation_task_s::TASK_LAND;

	case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		return automation_task_s::TASK_RTL;

	case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		return automation_task_s::TASK_MISSION;

	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		return automation_task_s::TASK_REPOSITION;

	case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		return automation_task_s::TASK_CHANGE_ALTITUDE;

	case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		return automation_task_s::TASK_ORBIT;

	case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		return automation_task_s::TASK_PRECLAND;

	case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		return automation_task_s::TASK_VTOL_TAKEOFF;

	case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		return automation_task_s::TASK_FIGURE_EIGHT;

	case vehicle_command_s::VEHICLE_CMD_DO_SET_HOME:
		return automation_task_s::TASK_HOME_SET;

	default:
		return automation_task_s::TASK_NONE;
	}
}

uint8_t AirPlaneCommandHandler::parseDoSetModeParams(const vehicle_command_s &cmd) const
{
	uint8_t base_mode = (uint8_t)cmd.param1;
	uint8_t custom_main_mode = (uint8_t)cmd.param2;
	uint8_t custom_sub_mode = (uint8_t)cmd.param3;

	uint8_t desired_mode = mode_status_s::OPERATION_MODE_MAX;

	if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED) {
		switch (custom_main_mode) {
		case PX4_CUSTOM_MAIN_MODE_MANUAL:
			desired_mode = mode_status_s::OPERATION_MODE_MANUAL;
			break;

		case PX4_CUSTOM_MAIN_MODE_ALTCTL:
			desired_mode = mode_status_s::OPERATION_MODE_ALTCTL;
			break;

		case PX4_CUSTOM_MAIN_MODE_POSCTL:
			if (custom_sub_mode == PX4_CUSTOM_SUB_MODE_POSCTL_SLOW) {
				desired_mode = mode_status_s::OPERATION_MODE_POSITION_SLOW;
			} else {
				desired_mode = mode_status_s::OPERATION_MODE_POSCTL;
			}
			break;

		case PX4_CUSTOM_MAIN_MODE_AUTO:
			switch (custom_sub_mode) {
			case PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_LOITER;
				break;
			case PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_MISSION;
				break;
			case PX4_CUSTOM_SUB_MODE_AUTO_RTL:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_RTL;
				break;
			case PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_TAKEOFF;
				break;
			case PX4_CUSTOM_SUB_MODE_AUTO_LAND:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_LAND;
				break;
			case PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET;
				break;
			case PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND:
				desired_mode = mode_status_s::OPERATION_MODE_AUTO_PRECLAND;
				break;
			default:
				if (custom_sub_mode >= PX4_CUSTOM_SUB_MODE_EXTERNAL1 &&
				    custom_sub_mode <= PX4_CUSTOM_SUB_MODE_EXTERNAL8) {
					desired_mode = mode_status_s::OPERATION_MODE_EXTERNAL1 +
						       (custom_sub_mode - PX4_CUSTOM_SUB_MODE_EXTERNAL1);
				} else if (custom_sub_mode == 0) {
					desired_mode = mode_status_s::OPERATION_MODE_AUTO_MISSION;
				}
				break;
			}
			break;

		case PX4_CUSTOM_MAIN_MODE_ACRO:
			desired_mode = mode_status_s::OPERATION_MODE_ACRO;
			break;

		case PX4_CUSTOM_MAIN_MODE_STABILIZED:
			desired_mode = mode_status_s::OPERATION_MODE_STAB;
			break;

		case PX4_CUSTOM_MAIN_MODE_OFFBOARD:
			desired_mode = mode_status_s::OPERATION_MODE_OFFBOARD;
			break;

		case PX4_CUSTOM_MAIN_MODE_TERMINATION:
			desired_mode = mode_status_s::OPERATION_MODE_TERMINATION;
			break;
		}

	} else {
		// Use base mode
		if (base_mode & VEHICLE_MODE_FLAG_AUTO_ENABLED) {
			desired_mode = mode_status_s::OPERATION_MODE_AUTO_MISSION;

		} else if (base_mode & VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED) {
			if (base_mode & VEHICLE_MODE_FLAG_GUIDED_ENABLED) {
				desired_mode = mode_status_s::OPERATION_MODE_POSCTL;

			} else if (base_mode & VEHICLE_MODE_FLAG_STABILIZE_ENABLED) {
				desired_mode = mode_status_s::OPERATION_MODE_STAB;

			} else {
				desired_mode = mode_status_s::OPERATION_MODE_MANUAL;
			}
		}
	}

	return desired_mode;
}

CommandResult AirPlaneCommandHandler::handleDoSetMode(const vehicle_command_s &cmd,
						      CommandContext &ctx) const
{
	uint8_t desired_mode = parseDoSetModeParams(cmd);

	if (desired_mode == mode_status_s::OPERATION_MODE_MAX) {
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
		return CommandResult::Denied;
	}

	// Special handling for LAND mode: always allow with force
	const bool force = (desired_mode == mode_status_s::OPERATION_MODE_AUTO_LAND);

	requestModeChange(ctx, desired_mode, false, force);
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresModeChange;
}

CommandResult AirPlaneCommandHandler::handleSetNavState(const vehicle_command_s &cmd,
							CommandContext &ctx) const
{
	uint8_t desired_mode = (uint8_t)(cmd.param1 + 0.5f);

	requestModeChange(ctx, desired_mode);
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresModeChange;
}

CommandResult AirPlaneCommandHandler::handleDoSetStandardMode(const vehicle_command_s &cmd,
							      CommandContext &ctx) const
{
	uint8_t standard_mode = (uint8_t)cmd.param1;
	uint8_t desired_mode = mode_util::getOperationModeFromStandardMode(standard_mode);

	if (desired_mode == mode_status_s::OPERATION_MODE_MAX) {
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
		return CommandResult::Denied;
	}

	requestModeChange(ctx, desired_mode);
	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresModeChange;
}

CommandResult AirPlaneCommandHandler::handleNavTakeoff(const vehicle_command_s &cmd,
						       CommandContext &ctx) const
{
	// Request mode change to AUTO_TAKEOFF
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_TAKEOFF);

	// Create automation task
	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_TAKEOFF);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleNavLand(const vehicle_command_s &cmd,
						    CommandContext &ctx) const
{
	// Land mode is always forced (emergency use)
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_LAND, false, true);

	// Create automation task
	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_LAND);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleNavRtl(const vehicle_command_s &cmd,
						   CommandContext &ctx) const
{
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_RTL);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_RTL);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleMissionStart(const vehicle_command_s &cmd,
							 CommandContext &ctx) const
{
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_MISSION);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_MISSION);
	ctx.automation_task.mission_item_index = (int32_t)(cmd.param1 + 0.5f);

	// Mission start also requires arming
	ctx.arm_requested = true;

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleDoReposition(const vehicle_command_s &cmd,
							 CommandContext &ctx) const
{
	const uint32_t change_mode_flags = (uint32_t)cmd.param2;
	const bool mode_switch_not_requested = (change_mode_flags & 1) == 0;
	const bool unsupported_bits_set = (change_mode_flags & ~1) != 0;

	if (mode_switch_not_requested || unsupported_bits_set) {
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}

	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_LOITER);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_REPOSITION);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleDoChangeAltitude(const vehicle_command_s &cmd,
							     CommandContext &ctx) const
{
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_LOITER);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_CHANGE_ALTITUDE);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleDoOrbit(const vehicle_command_s &cmd,
						    CommandContext &ctx) const
{
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_ORBIT);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_ORBIT);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleNavPrecland(const vehicle_command_s &cmd,
							CommandContext &ctx) const
{
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_PRECLAND);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_PRECLAND);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleNavVtolTakeoff(const vehicle_command_s &cmd,
							   CommandContext &ctx) const
{
	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_VTOL_TAKEOFF);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleDoFigureEight(const vehicle_command_s &cmd,
							  CommandContext &ctx) const
{
	// Figure-8 only for fixed wing and VTOL
	if (ctx.vehicle_type != vehicle_identity_s::VEHICLE_TYPE_FIXED_WING) {
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}

	requestModeChange(ctx, mode_status_s::OPERATION_MODE_AUTO_LOITER);

	ctx.automation_task_requested = true;
	fillAutomationTask(ctx.automation_task, cmd, automation_task_s::TASK_FIGURE_EIGHT);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresAutomation;
}

CommandResult AirPlaneCommandHandler::handleDoFlighttermination(const vehicle_command_s &cmd,
								CommandContext &ctx) const
{
	// Publish flight termination request to SystemManager
	ctx.flight_termination_requested = true;
	ctx.flight_termination_request.terminate = (cmd.param1 > 0.5f);

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresFlightTermination;
}

CommandResult AirPlaneCommandHandler::handleDoSetHome(const vehicle_command_s &cmd,
						      CommandContext &ctx) const
{
	// Publish set home request to SystemManager
	ctx.set_home_requested = true;

	const int param_use_current = static_cast<int>(cmd.param1 + 0.5f);

	if (param_use_current == 1) {
		ctx.set_home_request.use_current_position = true;

	} else {
		ctx.set_home_request.use_current_position = false;
		ctx.set_home_request.latitude = cmd.param5;
		ctx.set_home_request.longitude = cmd.param6;
		ctx.set_home_request.altitude = cmd.param7;
	}

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresSetHome;
}

CommandResult AirPlaneCommandHandler::handleComponentArmDisarm(const vehicle_command_s &cmd,
							       CommandContext &ctx) const
{
	const int param_arm = static_cast<int>(cmd.param1 + 0.5f);
	const int param_force = static_cast<int>(cmd.param2 + 0.5f);

	if (param_arm == 1) {
		// Arm request
		ctx.arm_requested = true;
		ctx.arm_force = (param_force == 21196);  // Magic number for force arm

	} else if (param_arm == 0) {
		// Disarm request
		ctx.disarm_requested = true;
		ctx.arm_force = (param_force == 21196);  // Magic number for force disarm
	}

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresArming;
}

CommandResult AirPlaneCommandHandler::handlePreflightCalibration(const vehicle_command_s &cmd,
								 CommandContext &ctx) const
{
	ctx.calibration_requested = true;

	// Determine calibration type from parameters
	if (static_cast<int>(cmd.param1) == 1) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_GYRO;

	} else if (static_cast<int>(cmd.param1) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
		   static_cast<int>(cmd.param5) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION ||
		   static_cast<int>(cmd.param7) == vehicle_command_s::PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_TEMPERATURE;

	} else if (static_cast<int>(cmd.param2) == 1) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_MAG;

	} else if (static_cast<int>(cmd.param3) == 1) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_BARO;

	} else if (static_cast<int>(cmd.param4) == 1) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_RC_START;

	} else if (static_cast<int>(cmd.param4) == 2) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_RC_TRIM;

	} else if (static_cast<int>(cmd.param4) == 0) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_RC_END;

	} else if (static_cast<int>(cmd.param5) == 1) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_ACCEL;

	} else if (static_cast<int>(cmd.param5) == 2) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_LEVEL;

	} else if (static_cast<int>(cmd.param5) == 4) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_ACCEL_QUICK;

	} else if (static_cast<int>(cmd.param6) == 1 || static_cast<int>(cmd.param6) == 2) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_AIRSPEED;

	} else if (static_cast<int>(cmd.param7) == 1) {
		ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_ESC;

	} else {
		ctx.calibration_requested = false;
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresCalibration;
}

CommandResult AirPlaneCommandHandler::handlePreflightRebootShutdown(const vehicle_command_s &cmd,
								    CommandContext &ctx) const
{
	ctx.reboot_requested = true;

	const int param1 = static_cast<int>(cmd.param1);

	if (param1 == 0) {
		ctx.reboot_request.reboot_type = reboot_request_s::REBOOT_TYPE_NONE;

	} else if (param1 == 1) {
		ctx.reboot_request.reboot_type = reboot_request_s::REBOOT_TYPE_REBOOT;

	} else if (param1 == 2) {
		ctx.reboot_request.reboot_type = reboot_request_s::REBOOT_TYPE_SHUTDOWN;

	} else if (param1 == 3) {
		ctx.reboot_request.reboot_type = reboot_request_s::REBOOT_TYPE_BOOTLOADER;

	} else {
		ctx.reboot_requested = false;
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresReboot;
}

CommandResult AirPlaneCommandHandler::handlePreflightStorage(const vehicle_command_s &cmd,
							     CommandContext &ctx) const
{
	ctx.storage_requested = true;

	const int param1 = static_cast<int>(cmd.param1);

	if (param1 == 0) {
		ctx.storage_request.operation = storage_request_s::STORAGE_OP_LOAD_DEFAULT;

	} else if (param1 == 1) {
		ctx.storage_request.operation = storage_request_s::STORAGE_OP_SAVE_DEFAULT;

	} else if (param1 == 2) {
		ctx.storage_request.operation = storage_request_s::STORAGE_OP_RESET_ALL_CONFIG;

	} else if (param1 == 3) {
		ctx.storage_request.operation = storage_request_s::STORAGE_OP_RESET_SENSOR_FACTORY;

	} else if (param1 == 4) {
		ctx.storage_request.operation = storage_request_s::STORAGE_OP_RESET_ALL;

	} else {
		ctx.storage_requested = false;
		ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return CommandResult::Unsupported;
	}

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresStorage;
}

CommandResult AirPlaneCommandHandler::handleFixedMagCalYaw(const vehicle_command_s &cmd,
							   CommandContext &ctx) const
{
	ctx.calibration_requested = true;
	ctx.calibration_request.calibration_type = calibration_request_s::CALIBRATION_TYPE_MAG_QUICK;

	// Store the calibration parameters
	ctx.calibration_request.mag_heading_rad = PX4_ISFINITE(cmd.param1) ?
						  static_cast<float>(cmd.param1 * M_PI / 180.0) : 0.f;
	ctx.calibration_request.latitude = cmd.param3;
	ctx.calibration_request.longitude = cmd.param4;

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresCalibration;
}

CommandResult AirPlaneCommandHandler::handleActuatorTest(const vehicle_command_s &cmd,
							 CommandContext &ctx) const
{
	ctx.actuator_test_requested = true;
	ctx.actuator_test_request.function = static_cast<uint16_t>(cmd.param5 + 0.5f);
	ctx.actuator_test_request.value = cmd.param1;
	ctx.actuator_test_request.timeout_ms = cmd.param2;

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresActuatorTest;
}

CommandResult AirPlaneCommandHandler::handleRunPrearmChecks(const vehicle_command_s &cmd,
							    CommandContext &ctx) const
{
	(void)cmd;
	ctx.prearm_check_requested = true;
	ctx.prearm_check_request.run_checks = true;

	ctx.ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	return CommandResult::RequiresPrearmCheck;
}

} // namespace command_processor
