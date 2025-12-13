/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "ModeManager.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;

ModeManager::ModeManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();

	// initialize all modes
	// currently this is required to get all parameters read
	for (int i = 0; i < static_cast<int>(ModeIndex::Count); i++) {
		_initMode(static_cast<ModeIndex>(i));
	}

	// disable all modes
	_initMode(ModeIndex::None);
}

ModeManager::~ModeManager()
{
	if (_current_mode.mode) {
		_current_mode.mode->~Mode();
	}

	perf_free(_loop_perf);
}

bool ModeManager::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// limit to every other vehicle_local_position update (50 Hz)
	_vehicle_local_position_sub.set_interval_us(20_ms);
	_time_stamp_last_loop = hrt_absolute_time();
	return true;
}

void ModeManager::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// generate setpoints on local position changes
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		const hrt_abstime time_stamp_now = vehicle_local_position.timestamp_sample;
		// Guard against too small (< 0.2ms) and too large (> 100ms) dt's.
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) / 1e6f), 0.0002f, 0.1f);
		_time_stamp_last_loop = time_stamp_now;

		_vehicle_control_mode_sub.update();
		_vehicle_land_detected_sub.update();
		_vehicle_status_sub.update();

		selectAndActivateMode();

		if (_vehicle_command_sub.updated()) {
			handleCommand();
		}

		tryApplyCommandIfAny();

		if (isAnyModeActive()) {
			generateControlSetpoint(dt, vehicle_local_position);
		}

	}

	perf_end(_loop_perf);
}

void ModeManager::updateParams()
{
	ModuleParams::updateParams();

	if (isAnyModeActive()) {
		_current_mode.mode->handleParameterUpdate();
	}
}

void ModeManager::selectAndActivateMode()
{
	// Do not run any mode for VTOLs in fixed-wing mode
	if ((_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
	    || ((_vehicle_status_sub.get().operation_mode >= vehicle_status_s::OPERATION_MODE_EXTERNAL1)
		&& (_vehicle_status_sub.get().operation_mode <= vehicle_status_s::OPERATION_MODE_EXTERNAL8))) {
		switchMode(ModeIndex::None);
		return;
	}

	// Only run transition mode if altitude control is enabled (e.g. in Altitdue, Position, Auto flight mode)
	if (_vehicle_status_sub.get().in_transition_mode &&
		_vehicle_control_mode_sub.get().flag_control_altitude_enabled) {
		switchMode(ModeIndex::Transition);
		return;
	}

	bool found_some_mode = false;
	bool matching_mode_running = true;
	bool mode_failure = false;
	const bool operation_mode_descend = (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_DESCEND);

	// Follow me
	if (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) {
		found_some_mode = true;
		ModeError error = ModeError::InvalidMode;

#if !defined(CONSTRAINED_FLASH)
		error = switchMode(ModeIndex::AutoFollowTarget);
#endif // !CONSTRAINED_FLASH

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Manual mode for wheel loader (ground vehicle manual control)
	if (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL) {
		found_some_mode = true;
		ModeError error = switchMode(ModeIndex::ManualWheelLoader);

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// VLA 7-DOF Trajectory Following (chassis + boom + tilt)
	if (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA) {
		found_some_mode = true;
		ModeError error = switchMode(ModeIndex::VLA);

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Orbit
	if ((_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_ORBIT)
	    && !_command_failed) {
		found_some_mode = true;
		ModeError error = ModeError::InvalidMode;

#if !defined(CONSTRAINED_FLASH)
		error = switchMode(ModeIndex::Orbit);
#endif // !CONSTRAINED_FLASH

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Navigator interface for autonomous modes
	if (_vehicle_control_mode_sub.get().flag_control_auto_enabled
	    && !operation_mode_descend) {
		found_some_mode = true;

		if (switchMode(ModeIndex::Auto) != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// position slow mode
	if (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_POSITION_SLOW) {
		found_some_mode = true;
		ModeError error = switchMode(ModeIndex::ManualAccelerationSlow);
		mode_failure = error != ModeError::NoError;
	}

	// Manual position control
	if ((_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_POSCTL) || mode_failure) {
		found_some_mode = true;
		ModeError error = ModeError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchMode(ModeIndex::ManualPosition);
			break;

		case 4:
		default:
			if (_param_mpc_pos_mode.get() != 4) {
				PX4_ERR("MPC_POS_MODE %" PRId32 " invalid, resetting", _param_mpc_pos_mode.get());
				_param_mpc_pos_mode.set(4);
				_param_mpc_pos_mode.commit();
			}

			error = switchMode(ModeIndex::ManualAcceleration);
			break;
		}

		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Manual altitude control
	if ((_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) || mode_failure) {
		found_some_mode = true;
		ModeError error = ModeError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchMode(ModeIndex::ManualAltitude);
			break;

		case 3:
		default:
			error = switchMode(ModeIndex::ManualAltitudeSmoothVel);
			break;
		}

		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Emergency descend
	if (operation_mode_descend || mode_failure) {
		found_some_mode = true;

		ModeError error = switchMode(ModeIndex::Descend);

		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	if (mode_failure) {
		// For some reason no mode was able to start, go into failsafe mode
		found_some_mode = (switchMode(ModeIndex::Failsafe) == ModeError::NoError);
	}

	if (!found_some_mode) {
		switchMode(ModeIndex::None);
	}

	if (!matching_mode_running && _vehicle_control_mode_sub.get().flag_armed && !_no_matching_mode_error_printed) {
		PX4_ERR("Matching mode was not able to run, Nav state: %" PRIu32,
			_vehicle_status_sub.get().operation_mode, static_cast<uint32_t>(_current_mode.index));
	}

	_no_matching_mode_error_printed = !matching_mode_running;
}

void ModeManager::tryApplyCommandIfAny()
{
	if (isAnyModeActive() && _current_command.command != 0 && hrt_absolute_time() < _current_command.timestamp + 200_ms) {
		bool success = false;

		if (_current_mode.mode->applyCommandParameters(_current_command, success)) {
			_current_command.command = 0;

			if (!success) {
				switchMode(ModeIndex::Failsafe);
				_command_failed = true;
			}
		}
	}
}

void ModeManager::handleCommand()
{
	// get command
	vehicle_command_s command;

	while (_vehicle_command_sub.update(&command)) {

		switch (command.command) {
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
			// The command might trigger a mode switch, and the mode switch can happen before or
			// after we receive the command here, so we store it for later.
			memcpy(&_current_command, &command, sizeof(vehicle_command_s));
			_command_failed = false;
			break;
		}

		if (_current_mode.mode) {
			// check for other commands not related to mode switching
			if ((command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED)
			    && (static_cast<uint8_t>(command.param1 + .5f) == vehicle_command_s::SPEED_TYPE_GROUNDSPEED)
			    && (command.param2 > 0.f)) {
				_current_mode.mode->overrideCruiseSpeed(command.param2);
			}
		}
	}
}

void ModeManager::generateControlSetpoint(const float dt,
		const vehicle_local_position_s &vehicle_local_position)
{
	// If the mode fails sned out empty NAN setpoints and the controller will emergency failsafe
	trajectory_setpoint_s setpoint = Mode::empty_control_setpoint;
	vehicle_constraints_s constraints = Mode::empty_constraints;

	if (_current_mode.mode->updateInitialize() && _current_mode.mode->update()) {
		// setpoints and constraints for the position controller from mode
		setpoint = _current_mode.mode->getControlSetpoint();
		constraints = _current_mode.mode->getConstraints();
	}

	if (_takeoff_status_sub.updated()) {
		takeoff_status_s takeoff_status;

		if (_takeoff_status_sub.copy(&takeoff_status)) {
			_takeoff_state = takeoff_status.takeoff_state;
		}
	}

	if (_takeoff_state < takeoff_status_s::TAKEOFF_STATE_RAMPUP) {
		// reactivate the mode which will reset the setpoint to current state
		_current_mode.mode->reActivate();
	}


	setpoint.timestamp = hrt_absolute_time();
	_control_setpoint_pub.publish(setpoint);

	constraints.timestamp = hrt_absolute_time();
	_vehicle_constraints_pub.publish(constraints);

	// if there's any change in landing gear setpoint publish it
	landing_gear_s landing_gear = _current_mode.mode->getGear();

	if (landing_gear.landing_gear != _old_landing_gear_position
	    && landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {

		landing_gear.timestamp = hrt_absolute_time();
		_landing_gear_pub.publish(landing_gear);
	}

	_old_landing_gear_position = landing_gear.landing_gear;
}

ModeError ModeManager::switchMode(ModeIndex new_mode_index)
{
	// switch to the running mode, nothing to do
	if (new_mode_index == _current_mode.index) {
		return ModeError::NoError;
	}

	// Save current setpoints for the next Mode
	trajectory_setpoint_s last_setpoint = Mode::empty_control_setpoint;

	if (isAnyModeActive()) {
		last_setpoint = _current_mode.mode->getControlSetpoint();
	}

	if (_initMode(new_mode_index)) {
		// invalid mode
		return ModeError::InvalidMode;
	}

	if (!isAnyModeActive()) {
		// no mode running
		return ModeError::NoError;
	}

	// activation failed
	if (!_current_mode.mode->updateInitialize() || !_current_mode.mode->activate(last_setpoint)) {
		_current_mode.mode->~Mode();
		_current_mode.mode = nullptr;
		_current_mode.index = ModeIndex::None;
		return ModeError::ActivationFailed;
	}

	_command_failed = false;

	return ModeError::NoError;
}

ModeError ModeManager::switchMode(int new_mode_index)
{
	// make sure we are in range of the enumeration before casting
	if (static_cast<int>(ModeIndex::None) <= new_mode_index &&
	    static_cast<int>(ModeIndex::Count) > new_mode_index) {
		return switchMode(ModeIndex(new_mode_index));
	}

	switchMode(ModeIndex::None);
	return ModeError::InvalidMode;
}

const char *ModeManager::errorToString(const ModeError error)
{
	switch (error) {
	case ModeError::NoError: return "No Error";

	case ModeError::InvalidMode: return "Invalid Mode";

	case ModeError::ActivationFailed: return "Activation Failed";
	}

	return "This error is not mapped to a string or is unknown.";
}

int ModeManager::task_spawn(int argc, char *argv[])
{
	ModeManager *instance = new ModeManager();

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

int ModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ModeManager::print_status()
{
	if (isAnyModeActive()) {
		PX4_INFO("Running, active mode: %" PRIu32, static_cast<uint32_t>(_current_mode.index));

	} else {
		PX4_INFO("Running, no mode active");
	}

	perf_print_counter(_loop_perf);
	return 0;
}

int ModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the setpoint generation for all modes. It takes the current mode state of the vehicle as input
and outputs setpoints for controllers.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mode_manager_main(int argc, char *argv[])
{
	return ModeManager::main(argc, argv);
}
