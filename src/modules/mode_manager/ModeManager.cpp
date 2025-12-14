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
#include "UserModeIntention.hpp"

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

		// Handle mode change requests from system_manager
		handleModeChangeRequest();

		// Handle failsafe mode requests from system_manager
		handleFailsafeModeRequest();

		selectAndActivateMode();

		// Publish mode status for system_manager and other modules
		publishModeStatus();

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
	// Update vehicle type configuration subscription
	_vehicle_type_config_sub.update();

	// Get the user's intended mode from mode_change_request (not from vehicle_status)
	const uint8_t intended_mode = _user_mode_intention.get();

	// Handle external modes - no mode manager control
	if ((intended_mode >= vehicle_status_s::OPERATION_MODE_EXTERNAL3)
	    && (intended_mode <= vehicle_status_s::OPERATION_MODE_EXTERNAL8)) {
		switchMode(ModeIndex::None);
		return;
	}

	// Use vehicle-type-specific mode selection based on mode_change_logic from vehicle_type_config
	const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();

	if (vtc.config_valid) {
		switch (vtc.mode_change_logic) {
		case vehicle_type_config_s::MODE_CHANGE_LOGIC_WHEEL_LOADER:
			selectWheelLoaderMode();
			return;

		case vehicle_type_config_s::MODE_CHANGE_LOGIC_ROVER:
			selectRoverMode();
			return;

		case vehicle_type_config_s::MODE_CHANGE_LOGIC_ROTARY_WING:
			selectRotaryWingMode();
			return;

		case vehicle_type_config_s::MODE_CHANGE_LOGIC_FIXED_WING:
			selectFixedWingMode();
			return;

		case vehicle_type_config_s::MODE_CHANGE_LOGIC_STANDARD:
		default:
			// Fall through to standard mode selection
			break;
		}
	} else {
		// Fallback: Use vehicle_type when config is not valid
		switch (_vehicle_status_sub.get().vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
			selectWheelLoaderMode();
			return;

		case vehicle_status_s::VEHICLE_TYPE_ROVER:
			selectRoverMode();
			return;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			selectRotaryWingMode();
			return;

		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			selectFixedWingMode();
			return;

		default:
			// Fall through to standard mode selection
			break;
		}
	}

	// Standard mode selection using user_mode_intention
	selectStandardMode();
}

void ModeManager::handleModeChangeRequest()
{
	// Process mode change requests from system_manager
	// This updates the user mode intention based on incoming requests
	if (_user_mode_intention.update()) {
		// Mode was changed - the new mode will be selected in selectAndActivateMode()
		PX4_DEBUG("Mode intention updated to %d", _user_mode_intention.get());
	}

	// Handle disarm - restore last safe mode
	if (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_DISARMED &&
	    _vehicle_land_detected_sub.get().landed) {
		// Note: This is simplified - full implementation would track arming state transitions
	}
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

void ModeManager::selectWheelLoaderMode()
{
	// Wheel loader specific mode selection
	// Uses _user_mode_intention from mode_change_request (not vehicle_status)
	// Priority: VLA Auto -> Manual -> Failsafe

	const uint8_t intended_mode = _user_mode_intention.get();
	ModeError error = ModeError::NoError;
	bool mode_activated = false;

	// Check if requested mode is available using vehicle_type_config
	if (!isModeAvailableForVehicleType(intended_mode)) {
		PX4_WARN("Requested mode %d not available for wheel loader", intended_mode);
	}

	// VLA 7-DOF Trajectory Following (chassis + boom + tilt) - autonomous mode
	if (intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA) {
		error = switchMode(ModeIndex::VLA);

		if (error == ModeError::NoError) {
			mode_activated = true;

		} else {
			PX4_WARN("VLA mode activation failed, falling back to manual");
		}
	}

	// Manual mode for wheel loader (default mode or fallback from VLA failure)
	if (!mode_activated) {
		error = switchMode(ModeIndex::ManualWheelLoader);

		if (error == ModeError::NoError) {
			mode_activated = true;

		} else {
			PX4_WARN("Manual wheel loader mode activation failed");
		}
	}

	// Failsafe mode if no other mode was successfully activated
	if (!mode_activated) {
		PX4_WARN("Entering failsafe mode for wheel loader");
		error = switchMode(ModeIndex::Failsafe);

		if (error != ModeError::NoError) {
			PX4_ERR("No valid mode available for wheel loader");
			switchMode(ModeIndex::None);
		}
	}
}

void ModeManager::selectRoverMode()
{
	// Rover specific mode selection
	// Uses _user_mode_intention from mode_change_request (not vehicle_status)

	const uint8_t intended_mode = _user_mode_intention.get();
	ModeError error = ModeError::NoError;
	bool mode_activated = false;

	// Check if requested mode is available
	if (!isModeAvailableForVehicleType(intended_mode)) {
		PX4_WARN("Requested mode %d not available for rover", intended_mode);
	}

	// Manual mode
	if (intended_mode == vehicle_status_s::OPERATION_MODE_MANUAL) {
		error = switchMode(ModeIndex::ManualPosition);

		if (error == ModeError::NoError) {
			mode_activated = true;
		}
	}

	// Position control mode
	if (!mode_activated && intended_mode == vehicle_status_s::OPERATION_MODE_POSCTL) {
		error = switchMode(ModeIndex::ManualAcceleration);

		if (error == ModeError::NoError) {
			mode_activated = true;
		}
	}

	// Auto modes (mission, loiter, RTL)
	if (!mode_activated && (intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_MISSION ||
				intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LOITER ||
				intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_RTL)) {
		error = switchMode(ModeIndex::Auto);

		if (error == ModeError::NoError) {
			mode_activated = true;
		}
	}

	// Failsafe mode if no other mode was successfully activated
	if (!mode_activated) {
		// For rover, default to manual position control as failsafe
		error = switchMode(ModeIndex::ManualPosition);

		if (error != ModeError::NoError) {
			PX4_ERR("No valid mode available for rover");
			switchMode(ModeIndex::None);
		}
	}
}

void ModeManager::selectRotaryWingMode()
{
	// Rotary wing (multicopter) specific mode selection
	// Uses _user_mode_intention from mode_change_request (not vehicle_status)
	// Priority: Transition -> Follow Target -> Orbit -> Auto -> Position Slow -> Position -> Altitude -> Descend -> Failsafe

	const uint8_t intended_mode = _user_mode_intention.get();
	const bool operation_mode_descend = (intended_mode == vehicle_status_s::OPERATION_MODE_DESCEND);
	ModeError error = ModeError::NoError;
	bool found_some_mode = false;
	bool matching_mode_running = true;
	bool mode_failure = false;

	// Check if requested mode is available
	if (!isModeAvailableForVehicleType(intended_mode)) {
		PX4_WARN("Requested mode %d not available for rotary wing", intended_mode);
	}

	// Only run transition flight task if altitude control is enabled
	if (_vehicle_status_sub.get().in_transition_mode &&
	    _vehicle_control_mode_sub.get().flag_control_altitude_enabled) {
		switchMode(ModeIndex::Transition);
		return;
	}

	// Follow me
	if (intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) {
		found_some_mode = true;

#if !defined(CONSTRAINED_FLASH)
		error = switchMode(ModeIndex::AutoFollowTarget);
#else
		error = ModeError::InvalidMode;
#endif

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Orbit
	if ((intended_mode == vehicle_status_s::OPERATION_MODE_ORBIT) && !_command_failed) {
		found_some_mode = true;

#if !defined(CONSTRAINED_FLASH)
		error = switchMode(ModeIndex::Orbit);
#else
		error = ModeError::InvalidMode;
#endif

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Auto modes (mission, loiter, RTL, etc.)
	if (!found_some_mode && (intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_MISSION ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LOITER ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_RTL ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LAND ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND) &&
	    !operation_mode_descend) {
		found_some_mode = true;

		if (switchMode(ModeIndex::Auto) != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Position slow mode
	if (!found_some_mode && intended_mode == vehicle_status_s::OPERATION_MODE_POSITION_SLOW) {
		found_some_mode = true;
		error = switchMode(ModeIndex::ManualAccelerationSlow);
		mode_failure = error != ModeError::NoError;
	}

	// Manual position control
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_POSCTL) || mode_failure)) {
		found_some_mode = true;

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
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) || mode_failure)) {
		found_some_mode = true;

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
	if (!found_some_mode && (operation_mode_descend || mode_failure)) {
		found_some_mode = true;
		error = switchMode(ModeIndex::Descend);
		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Failsafe mode
	if (mode_failure) {
		found_some_mode = (switchMode(ModeIndex::Failsafe) == ModeError::NoError);
	}

	if (!found_some_mode) {
		switchMode(ModeIndex::None);
	}

	if (!matching_mode_running && _vehicle_control_mode_sub.get().flag_armed && !_no_matching_mode_error_printed) {
		PX4_ERR("Matching mode was not able to run for rotary wing, Nav state: %" PRIu32,
			intended_mode);
	}

	_no_matching_mode_error_printed = !matching_mode_running;
}

void ModeManager::selectFixedWingMode()
{
	// Fixed wing specific mode selection
	// Uses _user_mode_intention from mode_change_request (not vehicle_status)
	// Fixed wing uses different control modes compared to multirotors
	// Priority: Auto -> Position -> Altitude -> Failsafe

	const uint8_t intended_mode = _user_mode_intention.get();
	const bool operation_mode_descend = (intended_mode == vehicle_status_s::OPERATION_MODE_DESCEND);
	ModeError error = ModeError::NoError;
	bool found_some_mode = false;
	bool matching_mode_running = true;
	bool mode_failure = false;

	// Check if requested mode is available
	if (!isModeAvailableForVehicleType(intended_mode)) {
		PX4_WARN("Requested mode %d not available for fixed wing", intended_mode);
	}

	// Only run transition flight task if in transition mode
	if (_vehicle_status_sub.get().in_transition_mode &&
	    _vehicle_control_mode_sub.get().flag_control_altitude_enabled) {
		switchMode(ModeIndex::Transition);
		return;
	}

	// Auto modes (mission, loiter, RTL)
	if ((intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_MISSION ||
	     intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LOITER ||
	     intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_RTL ||
	     intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF ||
	     intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LAND) &&
	    !operation_mode_descend) {
		found_some_mode = true;

		if (switchMode(ModeIndex::Auto) != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Manual position control (for fixed wing this typically means altitude + heading hold)
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_POSCTL) || mode_failure)) {
		found_some_mode = true;
		error = switchMode(ModeIndex::ManualPosition);
		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Manual altitude control
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) || mode_failure)) {
		found_some_mode = true;
		error = switchMode(ModeIndex::ManualAltitude);
		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Stabilized mode (attitude control only)
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_STAB) || mode_failure)) {
		found_some_mode = true;
		// Fixed wing stabilized uses attitude control
		// Fall through to no specific mode - FW attitude controller handles this
		switchMode(ModeIndex::None);
		return;
	}

	// Manual mode (direct control)
	if (!found_some_mode && intended_mode == vehicle_status_s::OPERATION_MODE_MANUAL) {
		found_some_mode = true;
		// Fixed wing manual uses direct control - no mode manager involvement
		switchMode(ModeIndex::None);
		return;
	}

	// Emergency descend
	if (!found_some_mode && (operation_mode_descend || mode_failure)) {
		found_some_mode = true;
		error = switchMode(ModeIndex::Descend);
		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Failsafe mode
	if (mode_failure) {
		found_some_mode = (switchMode(ModeIndex::Failsafe) == ModeError::NoError);
	}

	if (!found_some_mode) {
		switchMode(ModeIndex::None);
	}

	if (!matching_mode_running && _vehicle_control_mode_sub.get().flag_armed && !_no_matching_mode_error_printed) {
		PX4_ERR("Matching mode was not able to run for fixed wing, Nav state: %" PRIu32,
			intended_mode);
	}

	_no_matching_mode_error_printed = !matching_mode_running;
}

void ModeManager::selectStandardMode()
{
	// Standard mode selection using _user_mode_intention (not vehicle_status)
	// This is a fallback for vehicle types without specific selection logic

	const uint8_t intended_mode = _user_mode_intention.get();
	const bool operation_mode_descend = (intended_mode == vehicle_status_s::OPERATION_MODE_DESCEND);
	ModeError error = ModeError::NoError;
	bool found_some_mode = false;
	bool matching_mode_running = true;
	bool mode_failure = false;

	// Only run transition flight task if altitude control is enabled
	if (_vehicle_status_sub.get().in_transition_mode &&
	    _vehicle_control_mode_sub.get().flag_control_altitude_enabled) {
		switchMode(ModeIndex::Transition);
		return;
	}

	// Follow me
	if (intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) {
		found_some_mode = true;
		ModeError error = ModeError::InvalidMode;

#if !defined(CONSTRAINED_FLASH)
		error = switchMode(ModeIndex::AutoFollowTarget);
#endif

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Manual mode for wheel loader
	if (!found_some_mode && intended_mode == vehicle_status_s::OPERATION_MODE_MANUAL) {
		found_some_mode = true;
		error = switchMode(ModeIndex::ManualWheelLoader);

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// VLA 7-DOF Trajectory Following
	if (!found_some_mode && intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA) {
		found_some_mode = true;
		error = switchMode(ModeIndex::VLA);

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Orbit
	if (!found_some_mode && (intended_mode == vehicle_status_s::OPERATION_MODE_ORBIT) && !_command_failed) {
		found_some_mode = true;

#if !defined(CONSTRAINED_FLASH)
		error = switchMode(ModeIndex::Orbit);
#else
		error = ModeError::InvalidMode;
#endif

		if (error != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Auto modes
	if (!found_some_mode && (intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_MISSION ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LOITER ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_RTL ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF ||
				 intended_mode == vehicle_status_s::OPERATION_MODE_AUTO_LAND) &&
	    !operation_mode_descend) {
		found_some_mode = true;

		if (switchMode(ModeIndex::Auto) != ModeError::NoError) {
			matching_mode_running = false;
			mode_failure = true;
		}
	}

	// Position slow mode
	if (!found_some_mode && intended_mode == vehicle_status_s::OPERATION_MODE_POSITION_SLOW) {
		found_some_mode = true;
		error = switchMode(ModeIndex::ManualAccelerationSlow);
		mode_failure = error != ModeError::NoError;
	}

	// Manual position control
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_POSCTL) || mode_failure)) {
		found_some_mode = true;

		switch (_param_mpc_pos_mode.get()) {
		case 0:
			error = switchMode(ModeIndex::ManualPosition);
			break;

		case 4:
		default:
			error = switchMode(ModeIndex::ManualAcceleration);
			break;
		}

		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	// Manual altitude control
	if (!found_some_mode && ((intended_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) || mode_failure)) {
		found_some_mode = true;

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
	if (!found_some_mode && (operation_mode_descend || mode_failure)) {
		found_some_mode = true;
		error = switchMode(ModeIndex::Descend);
		mode_failure = (error != ModeError::NoError);
		matching_mode_running = matching_mode_running && !mode_failure;
	}

	if (mode_failure) {
		found_some_mode = (switchMode(ModeIndex::Failsafe) == ModeError::NoError);
	}

	if (!found_some_mode) {
		switchMode(ModeIndex::None);
	}

	if (!matching_mode_running && _vehicle_control_mode_sub.get().flag_armed && !_no_matching_mode_error_printed) {
		PX4_ERR("Matching mode was not able to run, Nav state: %" PRIu32, intended_mode);
	}

	_no_matching_mode_error_printed = !matching_mode_running;
}

void ModeManager::handleFailsafeModeRequest()
{
	failsafe_mode_request_s failsafe_request;

	if (_failsafe_mode_request_sub.update(&failsafe_request)) {
		// Validate timestamp freshness (100ms timeout)
		if (hrt_elapsed_time(&failsafe_request.timestamp) > 100_ms) {
			PX4_WARN("Stale failsafe mode request, ignoring");
			return;
		}

		// Check severity and force flag
		bool should_change = failsafe_request.force;

		if (!should_change) {
			switch (failsafe_request.severity) {
			case failsafe_mode_request_s::SEVERITY_CRITICAL:
			case failsafe_mode_request_s::SEVERITY_HIGH:
				should_change = true;
				break;

			case failsafe_mode_request_s::SEVERITY_MEDIUM:
				// Medium severity: change unless user is actively controlling
				should_change = !_vehicle_control_mode_sub.get().flag_control_manual_enabled;
				break;

			case failsafe_mode_request_s::SEVERITY_LOW:
				// Low severity: only warn
				PX4_INFO("Failsafe condition (low severity): source=%d, suggested mode=%d",
					 failsafe_request.source, failsafe_request.requested_mode);
				break;
			}
		}

		if (should_change) {
			// Store previous mode before failsafe
			_previous_mode = _vehicle_status_sub.get().operation_mode;
			_failsafe_mode_active = true;

			// Request the failsafe mode through user intention system
			_user_mode_intention.change(failsafe_request.requested_mode,
						    mode_manager::ModeChangeSource::Failsafe,
						    failsafe_request.force);

			PX4_INFO("Failsafe mode request: %d -> %d (source=%d, severity=%d)",
				 _previous_mode, failsafe_request.requested_mode,
				 failsafe_request.source, failsafe_request.severity);
		}
	}
}

void ModeManager::publishModeStatus()
{
	mode_status_s mode_status{};
	mode_status.timestamp = hrt_absolute_time();

	// Current mode from vehicle_status (which reflects actual active mode)
	mode_status.current_mode = _vehicle_status_sub.get().operation_mode;
	mode_status.user_intended_mode = _user_mode_intention.get();
	mode_status.previous_mode = _previous_mode;

	// Mode executor in charge
	mode_status.mode_executor_in_charge = _vehicle_status_sub.get().mode_executor_in_charge;

	// Mode availability masks
	mode_status.valid_modes_mask = getValidModesMask();
	mode_status.can_set_modes_mask = getCanSetModesMask();

	// Status flags
	mode_status.mode_change_in_progress = (mode_status.current_mode != mode_status.user_intended_mode);
	mode_status.failsafe_mode_active = _failsafe_mode_active;

	// External mode detection
	mode_status.external_mode_active =
		(mode_status.current_mode >= vehicle_status_s::OPERATION_MODE_EXTERNAL1) &&
		(mode_status.current_mode <= vehicle_status_s::OPERATION_MODE_EXTERNAL8);

	// Last change result
	mode_status.last_change_result = _last_mode_change_result;

	_mode_status_pub.publish(mode_status);
}

uint32_t ModeManager::getValidModesMask() const
{
	const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();

	if (vtc.config_valid) {
		return vtc.available_modes_mask;
	}

	// Default: basic modes available
	uint32_t mask = 0;
	mask |= (1u << vehicle_status_s::OPERATION_MODE_MANUAL);
	mask |= (1u << vehicle_status_s::OPERATION_MODE_ALTCTL);
	mask |= (1u << vehicle_status_s::OPERATION_MODE_POSCTL);

	return mask;
}

uint32_t ModeManager::getCanSetModesMask() const
{
	uint32_t valid_mask = getValidModesMask();
	uint32_t can_set_mask = valid_mask;

	// Remove modes that cannot be set in current state
	const vehicle_status_s &vs = _vehicle_status_sub.get();

	// If not armed, can only set certain modes
	if (vs.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		// Allow pre-arm mode selection
		return can_set_mask;
	}

	// If in failsafe, limit mode changes
	if (_failsafe_mode_active) {
		// Only allow safer modes during failsafe
		can_set_mask &= ((1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
				 (1u << vehicle_status_s::OPERATION_MODE_POSCTL) |
				 (1u << vehicle_status_s::OPERATION_MODE_DESCEND));
	}

	return can_set_mask;
}

bool ModeManager::isModeAvailableForVehicleType(uint8_t operation_mode) const
{
	const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();

	if (!vtc.config_valid) {
		// If config is not valid, allow all modes (fallback behavior)
		return true;
	}

	// Check if the mode bit is set in the available_modes_mask
	if (operation_mode < 32) {
		return (vtc.available_modes_mask & (1u << operation_mode)) != 0;
	}

	return false;
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
