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

#include "command_processor.hpp"

#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <commander/px4_custom_mode.h>
#include <uORB/topics/vehicle_identity.h>
#include <uORB/topics/vtol_vehicle_status.h>

namespace command_processor
{

#ifndef CONSTRAINED_FLASH
static bool send_vehicle_command(
	const uint32_t cmd,
	const float param1 = NAN,
	const float param2 = NAN,
	const float param3 = NAN,
	const float param4 = NAN,
	const double param5 = static_cast<double>(NAN),
	const double param6 = static_cast<double>(NAN),
	const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}

static bool wait_for_vehicle_command_reply(const uint32_t cmd,
		uORB::SubscriptionData<vehicle_command_ack_s> &vehicle_command_ack_sub)
{
	hrt_abstime start = hrt_absolute_time();

	while (hrt_absolute_time() - start < 100_ms) {
		if (vehicle_command_ack_sub.update()) {
			if (vehicle_command_ack_sub.get().command == cmd) {
				return vehicle_command_ack_sub.get().result ==
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}

		px4_usleep(10000);
	}

	return false;
}

static bool broadcast_vehicle_command(
	const uint32_t cmd,
	const float param1 = NAN,
	const float param2 = NAN,
	const float param3 = NAN,
	const float param4 = NAN,
	const double param5 = static_cast<double>(NAN),
	const double param6 = static_cast<double>(NAN),
	const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = 0;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = 0;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}
#endif // CONSTRAINED_FLASH

CommandProcessor::CommandProcessor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
	selectHandler();
}

CommandProcessor::~CommandProcessor()
{
	perf_free(_loop_perf);
}

bool CommandProcessor::init()
{
	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void CommandProcessor::Run()
{
	if (should_exit()) {
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		selectHandler();
	}

	// Update vehicle status
	_vehicle_status_sub.update();

	// Check if vehicle type changed
	if (_vehicle_status_sub.get().vehicle_type != _current_vehicle_type) {
		_current_vehicle_type = _vehicle_status_sub.get().vehicle_type;
		selectHandler();
	}

	// Process incoming vehicle commands
	vehicle_command_s cmd;

	while (_vehicle_command_sub.update(&cmd)) {
		processCommand(cmd);
	}

	// Handle async results (mode change, automation task)
	handleResults();

	perf_end(_loop_perf);
}

void CommandProcessor::selectHandler()
{
	switch (_current_vehicle_type) {
	case vehicle_identity_s::VEHICLE_TYPE_WHEEL_LOADER:
		_active_handler = &_wheel_loader_handler;
		break;

	case vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING:
	case vehicle_identity_s::VEHICLE_TYPE_FIXED_WING:
	case vehicle_identity_s::VEHICLE_TYPE_ROVER:
	default:
		_active_handler = &_airplane_handler;
		break;
	}

	PX4_DEBUG("Selected command handler: %s", _active_handler->getName());
}

bool CommandProcessor::processCommand(const vehicle_command_s &cmd)
{
	// Check target system/component
	const vehicle_status_s &status = _vehicle_status_sub.get();

	if (((cmd.target_system != status.system_id) && (cmd.target_system != 0)) ||
	    ((cmd.target_component != status.component_id) && (cmd.target_component != 0))) {
		return false;
	}

	// Check if command is supported by current handler
	if (!_active_handler->supportsCommand(cmd.command)) {
		// Forward unsupported commands (they might be handled by other modules)
		return false;
	}

	// Check if command should be rejected for this vehicle type
	if (_active_handler->shouldRejectCommand(cmd)) {
		PX4_DEBUG("Command %d rejected for vehicle type %d", cmd.command, _current_vehicle_type);
		answerCommand(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
		return true;
	}

	// Build command context
	CommandContext ctx;
	ctx.current_mode = _mode_status_sub.get().current_mode;
	ctx.is_armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	ctx.vehicle_type = status.vehicle_type;

	// Process command through handler
	CommandResult result = _active_handler->processCommand(cmd, ctx);

	// Handle result
	switch (result) {
	case CommandResult::Accepted:
		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::Denied:
	case CommandResult::Unsupported:
	case CommandResult::TemporarilyRejected:
		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresModeChange:
		if (ctx.mode_change_requested) {
			publishModeChangeRequest(cmd, ctx);

			// Store pending command for async ACK
			_pending_command.command = cmd.command;
			_pending_command.source_system = cmd.source_system;
			_pending_command.source_component = cmd.source_component;
			_pending_command.timestamp = cmd.timestamp;
			_pending_command.waiting_for_mode = true;
		}

		// If also automation task, publish it
		if (ctx.automation_task_requested) {
			publishAutomationTask(ctx);
		}
		break;

	case CommandResult::RequiresAutomation:
		// Publish mode change if needed
		if (ctx.mode_change_requested) {
			publishModeChangeRequest(cmd, ctx);
			_pending_command.waiting_for_mode = true;
		}

		// Publish automation task
		if (ctx.automation_task_requested) {
			publishAutomationTask(ctx);
			_pending_command.waiting_for_task = true;
		}

		// Store pending command
		_pending_command.command = cmd.command;
		_pending_command.source_system = cmd.source_system;
		_pending_command.source_component = cmd.source_component;
		_pending_command.timestamp = cmd.timestamp;
		break;

	case CommandResult::RequiresArming:
		if (ctx.arm_requested || ctx.disarm_requested) {
			publishArmingRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresCalibration:
		if (ctx.calibration_requested) {
			publishCalibrationRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresReboot:
		if (ctx.reboot_requested) {
			publishRebootRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresStorage:
		if (ctx.storage_requested) {
			publishStorageRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresActuatorTest:
		if (ctx.actuator_test_requested) {
			publishActuatorTestRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresPrearmCheck:
		if (ctx.prearm_check_requested) {
			publishPrearmCheckRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresFlightTermination:
		if (ctx.flight_termination_requested) {
			publishFlightTerminationRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::RequiresSetHome:
		if (ctx.set_home_requested) {
			publishSetHomeRequest(cmd, ctx);
		}

		answerCommand(cmd, ctx.ack_result);
		break;

	case CommandResult::Forwarded:
	case CommandResult::InProgress:
		// Don't ACK - let other module handle it
		break;
	}

	return true;
}

void CommandProcessor::publishModeChangeRequest(const vehicle_command_s &cmd,
						const CommandContext &ctx)
{
	mode_change_request_s req{};
	req.timestamp = hrt_absolute_time();
	req.requested_mode = ctx.requested_mode;
	req.source = ctx.mode_change_source;
	req.allow_fallback = ctx.mode_allow_fallback;
	req.force = ctx.mode_force;
	req.cmd_source_system = cmd.source_system;
	req.cmd_source_component = cmd.source_component;
	req.cmd_command = cmd.command;

	_mode_change_request_pub.publish(req);

	PX4_DEBUG("Published mode change request: mode=%d, source=%d",
		  ctx.requested_mode, ctx.mode_change_source);
}

void CommandProcessor::publishAutomationTask(const CommandContext &ctx)
{
	automation_task_s task = ctx.automation_task;
	task.timestamp = hrt_absolute_time();

	_automation_task_pub.publish(task);

	PX4_DEBUG("Published automation task: type=%d", task.task_type);
}

void CommandProcessor::publishArmingRequest(const vehicle_command_s &cmd,
					    const CommandContext &ctx)
{
	arming_request_s req{};
	req.timestamp = hrt_absolute_time();
	req.request_arm = ctx.arm_requested;
	req.request_disarm = ctx.disarm_requested;
	req.force = ctx.arm_force;
	req.cmd_source_system = cmd.source_system;
	req.cmd_source_component = cmd.source_component;
	req.cmd_command = cmd.command;

	_arming_request_pub.publish(req);

	PX4_DEBUG("Published arming request: arm=%d, disarm=%d",
		  ctx.arm_requested, ctx.disarm_requested);
}

void CommandProcessor::publishCalibrationRequest(const vehicle_command_s &cmd,
						 const CommandContext &ctx)
{
	calibration_request_s req = ctx.calibration_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_calibration_request_pub.publish(req);

	PX4_DEBUG("Published calibration request: type=%d", req.calibration_type);
}

void CommandProcessor::publishRebootRequest(const vehicle_command_s &cmd,
					    const CommandContext &ctx)
{
	reboot_request_s req = ctx.reboot_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_reboot_request_pub.publish(req);

	PX4_DEBUG("Published reboot request: type=%d", req.reboot_type);
}

void CommandProcessor::publishStorageRequest(const vehicle_command_s &cmd,
					     const CommandContext &ctx)
{
	storage_request_s req = ctx.storage_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_storage_request_pub.publish(req);

	PX4_DEBUG("Published storage request: operation=%d", req.operation);
}

void CommandProcessor::publishActuatorTestRequest(const vehicle_command_s &cmd,
						  const CommandContext &ctx)
{
	actuator_test_request_s req = ctx.actuator_test_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_actuator_test_request_pub.publish(req);

	PX4_DEBUG("Published actuator test request: function=%d", req.function);
}

void CommandProcessor::publishPrearmCheckRequest(const vehicle_command_s &cmd,
						 const CommandContext &ctx)
{
	prearm_check_request_s req = ctx.prearm_check_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_prearm_check_request_pub.publish(req);

	PX4_DEBUG("Published prearm check request");
}

void CommandProcessor::publishFlightTerminationRequest(const vehicle_command_s &cmd,
						       const CommandContext &ctx)
{
	flight_termination_request_s req = ctx.flight_termination_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_flight_termination_request_pub.publish(req);

	PX4_DEBUG("Published flight termination request: terminate=%d", req.terminate);
}

void CommandProcessor::publishSetHomeRequest(const vehicle_command_s &cmd,
					     const CommandContext &ctx)
{
	set_home_request_s req = ctx.set_home_request;
	req.timestamp = hrt_absolute_time();
	req.source_system = cmd.source_system;
	req.source_component = cmd.source_component;

	_set_home_request_pub.publish(req);

	PX4_DEBUG("Published set home request: use_current=%d", req.use_current_position);
}

void CommandProcessor::answerCommand(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = cmd.command;
	ack.result = result;
	ack.result_param1 = 0;
	ack.result_param2 = 0;
	ack.target_system = cmd.source_system;
	ack.target_component = cmd.source_component;
	ack.from_external = false;

	_command_ack_pub.publish(ack);
}

void CommandProcessor::handleResults()
{
	// Check for mode change result
	mode_change_result_s mode_result;

	if (_mode_change_result_sub.update(&mode_result)) {
		if (_pending_command.waiting_for_mode &&
		    mode_result.cmd_command == _pending_command.command) {

			_pending_command.waiting_for_mode = false;

			// If no longer waiting for anything, send final ACK
			if (!_pending_command.waiting_for_task) {
				vehicle_command_ack_s ack{};
				ack.timestamp = hrt_absolute_time();
				ack.command = _pending_command.command;
				ack.result = mode_result.result;
				ack.target_system = _pending_command.source_system;
				ack.target_component = _pending_command.source_component;
				ack.from_external = false;

				_command_ack_pub.publish(ack);
				_pending_command.command = 0;
			}
		}
	}

	// Check for automation task result
	automation_task_result_s task_result;

	if (_automation_task_result_sub.update(&task_result)) {
		if (_pending_command.waiting_for_task &&
		    task_result.cmd_command == _pending_command.command) {

			_pending_command.waiting_for_task = false;

			// If no longer waiting for anything, send final ACK
			if (!_pending_command.waiting_for_mode) {
				vehicle_command_ack_s ack{};
				ack.timestamp = hrt_absolute_time();
				ack.command = _pending_command.command;
				ack.result = task_result.result;
				ack.target_system = _pending_command.source_system;
				ack.target_component = _pending_command.source_component;
				ack.from_external = false;

				_command_ack_pub.publish(ack);
				_pending_command.command = 0;
			}
		}
	}

	// Timeout for pending commands (5 seconds)
	if (_pending_command.command != 0 &&
	    hrt_elapsed_time(&_pending_command.timestamp) > 5_s) {
		PX4_WARN("Pending command %d timed out", _pending_command.command);

		vehicle_command_ack_s ack{};
		ack.timestamp = hrt_absolute_time();
		ack.command = _pending_command.command;
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
		ack.target_system = _pending_command.source_system;
		ack.target_component = _pending_command.source_component;
		ack.from_external = false;

		_command_ack_pub.publish(ack);
		_pending_command.command = 0;
	}
}

int CommandProcessor::task_spawn(int argc, char *argv[])
{
	CommandProcessor *instance = new CommandProcessor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CommandProcessor::custom_command(int argc, char *argv[])
{
#ifndef CONSTRAINED_FLASH

	if (!strcmp(argv[0], "calibrate")) {
		if (argc > 1) {
			if (!strcmp(argv[1], "gyro")) {
				// gyro calibration: param1 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 1.f, 0.f, 0.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "mag")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// magnetometer quick calibration: VEHICLE_CMD_FIXED_MAG_CAL_YAW
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_FIXED_MAG_CAL_YAW);

				} else {
					// magnetometer calibration: param2 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 1.f, 0.f, 0.f, 0.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "baro")) {
				// baro calibration: param3 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 1.f, 0.f, 0.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "accel")) {
				if (argc > 2 && (strcmp(argv[2], "quick") == 0)) {
					// accelerometer quick calibration: param5 = 3
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 4.0, 0.0, 0.f);

				} else {
					// accelerometer calibration: param5 = 1
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 1.0, 0.0, 0.f);
				}

			} else if (!strcmp(argv[1], "level")) {
				// board level calibration: param5 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 2.0, 0.0, 0.f);

			} else if (!strcmp(argv[1], "airspeed")) {
				// airspeed calibration: param6 = 2
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 2.0, 0.f);

			} else if (!strcmp(argv[1], "esc")) {
				// ESC calibration: param7 = 1
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION, 0.f, 0.f, 0.f, 0.f, 0.0, 0.0, 1.f);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
				return 1;
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[0], "check")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_RUN_PREARM_CHECKS);

		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
		PX4_INFO("Prearm check: %s", vehicle_status_sub.get().pre_flight_checks_pass ? "OK" : "FAILED");

		return 0;
	}

	if (!strcmp(argv[0], "arm")) {
		float param2 = 0.f;

		// 21196: force arming/disarming (e.g. allow arming to override prearm checks and disarming in flight)
		if (argc > 1 && !strcmp(argv[1], "-f")) {
			param2 = 21196.f;
		}

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
				     param2);

		return 0;
	}

	if (!strcmp(argv[0], "disarm")) {
		float param2 = 0.f;

		// 21196: force arming/disarming (e.g. allow arming to override prearm checks and disarming in flight)
		if (argc > 1 && !strcmp(argv[1], "-f")) {
			param2 = 21196.f;
		}

		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				     static_cast<float>(vehicle_command_s::ARMING_ACTION_DISARM),
				     param2);

		return 0;
	}

	if (!strcmp(argv[0], "takeoff")) {
		// switch to takeoff mode and arm
		uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);

		if (wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF, vehicle_command_ack_sub)) {
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
					     static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
					     0.f);
		}

		return 0;
	}

	if (!strcmp(argv[0], "land")) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);

		return 0;
	}

	if (!strcmp(argv[0], "transition")) {
		uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
		vehicle_status_s vehicle_status{};
		vehicle_status_sub.copy(&vehicle_status);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION,
				     (float)(vehicle_status.vehicle_type == vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING ?
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW :
					     vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC), 0.0f);

		return 0;
	}

	if (!strcmp(argv[0], "mode")) {
		if (argc > 1) {

			if (!strcmp(argv[1], "manual")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_MANUAL);

			} else if (!strcmp(argv[1], "altctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);

			} else if (!strcmp(argv[1], "posctl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

			} else if (!strcmp(argv[1], "position:slow")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL,
						     PX4_CUSTOM_SUB_MODE_POSCTL_SLOW);

			} else if (!strcmp(argv[1], "auto:mission")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);

			} else if (!strcmp(argv[1], "auto:loiter")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

			} else if (!strcmp(argv[1], "auto:rtl")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_RTL);

			} else if (!strcmp(argv[1], "acro")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ACRO);

			} else if (!strcmp(argv[1], "offboard")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_OFFBOARD);

			} else if (!strcmp(argv[1], "stabilized")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_STABILIZED);

			} else if (!strcmp(argv[1], "auto:takeoff")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);

			} else if (!strcmp(argv[1], "auto:land")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

			} else if (!strcmp(argv[1], "auto:precland")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND);

			} else if (!strcmp(argv[1], "ext1")) {
				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_EXTERNAL1);

			} else {
				PX4_ERR("argument %s unsupported.", argv[1]);
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
		}
	}

	if (!strcmp(argv[0], "lockdown")) {

		if (argc < 2) {
			CommandProcessor::print_usage("not enough arguments, missing [on, off]");
			return 1;
		}

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION,
						strcmp(argv[1], "off") ? 2.0f : 0.0f /* lockdown */, 0.0f);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[0], "pair")) {

		// GCS pairing request handled by a companion
		bool ret = broadcast_vehicle_command(vehicle_command_s::VEHICLE_CMD_START_RX_PAIR, 10.f);

		return (ret ? 0 : 1);
	}

	if (!strcmp(argv[0], "set_ekf_origin")) {
		if (argc > 3) {

			double latitude  = atof(argv[1]);
			double longitude = atof(argv[2]);
			float  altitude  = atof(argv[3]);

			// Set the ekf NED origin global coordinates.
			bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
							0.f, 0.f, 0.0, 0.0, latitude, longitude, altitude);
			return (ret ? 0 : 1);

		} else {
			PX4_ERR("missing argument");
			return 0;
		}
	}

	if (!strcmp(argv[0], "poweroff")) {

		bool ret = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
						2.0f);

		return (ret ? 0 : 1);
	}

#endif // CONSTRAINED_FLASH

	return print_usage("unknown command");
}

int CommandProcessor::print_status()
{
	PX4_INFO("Running, active handler: %s", _active_handler->getName());
	PX4_INFO("Vehicle type: %d", _current_vehicle_type);

	if (_pending_command.command != 0) {
		PX4_INFO("Pending command: %d (mode=%d, task=%d)",
			 _pending_command.command,
			 _pending_command.waiting_for_mode,
			 _pending_command.waiting_for_task);
	}

	perf_print_counter(_loop_perf);
	return 0;
}

int CommandProcessor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Command Processor module handles vehicle_command_s messages and dispatches them
to vehicle-specific command handlers. It publishes mode_change_request and
automation_task messages based on the commands received.

This module provides:
- Vehicle-type specific command handling
- Mode change request publishing
- Automation task publishing
- Command acknowledgment handling
- CLI commands for arm/disarm, mode changes, calibration, etc.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("command_processor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
#ifndef CONSTRAINED_FLASH
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Run sensor calibration");
	PRINT_MODULE_USAGE_ARG("mag|baro|accel|gyro|level|esc|airspeed", "Calibration type", false);
	PRINT_MODULE_USAGE_ARG("quick", "Quick calibration [mag, accel (not recommended)]", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("check", "Run prearm checks");
	PRINT_MODULE_USAGE_COMMAND("arm");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force arming (do not run prearm checks)", true);
	PRINT_MODULE_USAGE_COMMAND("disarm");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Force disarming (disarm in air)", true);
	PRINT_MODULE_USAGE_COMMAND("takeoff");
	PRINT_MODULE_USAGE_COMMAND("land");
	PRINT_MODULE_USAGE_COMMAND_DESCR("transition", "VTOL transition");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Change flight mode");
	PRINT_MODULE_USAGE_ARG("manual|acro|offboard|stabilized|altctl|posctl|position:slow|auto:mission|auto:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland|ext1",
			       "Flight mode", false);
	PRINT_MODULE_USAGE_COMMAND("pair");
	PRINT_MODULE_USAGE_COMMAND("lockdown");
	PRINT_MODULE_USAGE_ARG("on|off", "Turn lockdown on or off", false);
	PRINT_MODULE_USAGE_COMMAND("set_ekf_origin");
	PRINT_MODULE_USAGE_ARG("lat, lon, alt", "Origin Latitude, Longitude, Altitude", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("poweroff", "Power off board (if supported)");
#endif
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace command_processor

extern "C" __EXPORT int command_processor_main(int argc, char *argv[])
{
	return command_processor::CommandProcessor::main(argc, argv);
}
