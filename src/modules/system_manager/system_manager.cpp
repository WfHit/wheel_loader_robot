/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file system_manager.cpp
 *
 * Main state machine / business logic
 *
 */

#include "system_manager.hpp"

/* system_manager module headers */
#include "arming/arm_authorization/arm_authorization.h"
#include "system_manager_helper.h"
#include "calibration/esc_calibration.h"
#define DEFINE_GET_PX4_CUSTOM_MODE
#include "px4_custom_mode.h"
#include "event_mode.hpp"
#include <lib/modes/ui.hpp>
#include <lib/modes/standard_modes.hpp>

/* PX4 headers */
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/external_reset_lockout.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>

#include <math.h>
#include <float.h>
#include <cstring>
#include <matrix/math.hpp>

#include <uORB/topics/vehicle_identity.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/tune_control.h>

typedef enum VEHICLE_MODE_FLAG {
	VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1,   /* 0b00000001 Reserved for future use. | */
	VEHICLE_MODE_FLAG_TEST_ENABLED         = 2,   /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	VEHICLE_MODE_FLAG_AUTO_ENABLED         = 4,   /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	VEHICLE_MODE_FLAG_GUIDED_ENABLED       = 8,   /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	VEHICLE_MODE_FLAG_STABILIZE_ENABLED    = 16,  /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	VEHICLE_MODE_FLAG_HIL_ENABLED          = 32,  /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,  /* 0b01000000 remote control input is enabled. | */
	VEHICLE_MODE_FLAG_SAFETY_ARMED         = 128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
	VEHICLE_MODE_FLAG_ENUM_END             = 129, /*  | */
} VEHICLE_MODE_FLAG;

// TODO: generate
static constexpr bool operator ==(const actuator_armed_s &a, const actuator_armed_s &b)
{
	return (a.armed == b.armed &&
		a.prearmed == b.prearmed &&
		a.ready_to_arm == b.ready_to_arm &&
		a.lockdown == b.lockdown &&
		a.manual_lockdown == b.manual_lockdown &&
		a.force_failsafe == b.force_failsafe &&
		a.in_esc_calibration_mode == b.in_esc_calibration_mode);
}
static_assert(sizeof(actuator_armed_s) == 16, "actuator_armed equality operator review");

#if defined(BOARD_HAS_POWER_CONTROL)
static orb_advert_t tune_control_pub = nullptr;

static void play_power_button_down_tune()
{
	// Override any other tunes because power-off sound should have the priority
	set_tune_override(tune_control_s::TUNE_ID_POWER_OFF);
}

static void stop_tune()
{
	tune_control_s tune_control{};
	tune_control.tune_override = true;
	tune_control.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
}

static orb_advert_t power_button_state_pub = nullptr;
static int power_button_state_notification_cb(
	board_power_button_state_notification_e request)
{
	// Note: this can be called from IRQ handlers, so we publish a message that will be handled
	// on the main thread of system_manager.
	power_button_state_s button_state{};
	button_state.timestamp = hrt_absolute_time();
	const int ret = PWR_BUTTON_RESPONSE_SHUT_DOWN_PENDING;

	switch (request) {
	case PWR_BUTTON_IDEL:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_IDEL;
		break;

	case PWR_BUTTON_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_DOWN;
		play_power_button_down_tune();
		break;

	case PWR_BUTTON_UP:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_UP;
		stop_tune();
		break;

	case PWR_BUTTON_REQUEST_SHUT_DOWN:
		button_state.event = power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN;
		break;

	default:
		PX4_ERR("unhandled power button state: %i", (int)request);
		return ret;
	}

	if (power_button_state_pub != nullptr) {
		orb_publish(ORB_ID(power_button_state), power_button_state_pub, &button_state);

	} else {
		PX4_ERR("power_button_state_pub not properly initialized");
	}

	return ret;
}
#endif // BOARD_HAS_POWER_CONTROL

int SystemManager::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}

int SystemManager::print_status()
{
	PX4_INFO("%s", is_armed() ? "Armed" : "Disarmed");
	PX4_INFO("navigation mode: %s",
		mode_util::operation_mode_names[_current_operation_mode]);
	PX4_INFO("user intended navigation mode: %s",
		mode_util::operation_mode_names[_cached_user_intended_mode]);
	PX4_INFO("in failsafe: %s", _failsafe_handler.inFailsafe() ? "yes" : "no");
	// Mode executor status is in mode_manager (use: mode_manager status)
	perf_print_counter(_loop_perf);
	perf_print_counter(_prearm_check_perf);
	return 0;
}

extern "C" __EXPORT int system_manager_main(int argc, char *argv[])
{
	return SystemManager::main(argc, argv);
}

transition_result_t SystemManager::arm(arm_disarm_reason_t calling_reason, bool run_prearm_checks)
{
	// Delegate to ArmingHandler which has the full arm logic
	arming_transition_result_t result = _arming_handler.arm(calling_reason, run_prearm_checks);

	if (result == TRANSITION_CHANGED) {
		_status_changed = true;
	}

	return static_cast<transition_result_t>(result);
}

transition_result_t SystemManager::disarm(arm_disarm_reason_t calling_reason, bool forced)
{
	// Delegate to ArmingHandler which has the full disarm logic
	arming_transition_result_t result = _arming_handler.disarm(calling_reason, forced);

	if (result == TRANSITION_CHANGED) {
		// update flight uuid
		const int32_t flight_uuid = _param_com_flight_uuid.get() + 1;
		_param_com_flight_uuid.set(flight_uuid);
		_param_com_flight_uuid.commit_no_notification();

		_status_changed = true;
	}

	return static_cast<transition_result_t>(result);
}

SystemManager::SystemManager() :
	ModuleParams(nullptr)
{
	_vehicle_land_detected.landed = true;

	_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_current_operation_mode = _cached_user_intended_mode;
	_current_operation_mode_user_intention = _cached_user_intended_mode;
	_current_operation_mode_timestamp = hrt_absolute_time();
	_vehicle_status.gcs_connection_lost = true;
	_vehicle_status.power_input_valid = true;

	// Initialize vehicle identity
	_vehicle_identity.system_id = 1;
	_vehicle_identity.component_id = 1;
	_vehicle_identity.system_type = 0;
	_vehicle_identity.vehicle_type = vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING;

	// default for vtol is rotary wing
	_vtol_vehicle_status.vehicle_vtol_state = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

	param_t param_mav_comp_id = param_find("MAV_COMP_ID");
	param_t param_mav_sys_id = param_find("MAV_SYS_ID");
	_param_mav_type = param_find("MAV_TYPE");
	_param_rc_map_fltmode = param_find("RC_MAP_FLTMODE");

	int32_t value_int32 = 0;

	// MAV_SYS_ID => vehicle_identity.system_id
	if ((param_mav_sys_id != PARAM_INVALID) && (param_get(param_mav_sys_id, &value_int32) == PX4_OK)) {
		_vehicle_identity.system_id = value_int32;
	}

	// MAV_COMP_ID => vehicle_identity.component_id
	if ((param_mav_comp_id != PARAM_INVALID) && (param_get(param_mav_comp_id, &value_int32) == PX4_OK)) {
		_vehicle_identity.component_id = value_int32;
	}

	update_parameters();

	// Set up failsafe handler with health and arming checks for notifications
	_failsafe_handler.setHealthAndArmingChecks(&_health_and_arming_checks);
}

SystemManager::~SystemManager()
{
	perf_free(_loop_perf);
	perf_free(_prearm_check_perf);
}

/*
 * ============================================================================
 * Request handlers from CommandProcessor
 *
 * These methods handle requests published by command_processor module.
 * This replaces the previous handle_command() function which processed
 * vehicle_command_s directly.
 * ============================================================================
 */

// handle_mode_change_requests removed - mode_manager handles ModeChangeRequest from command_processor
// handle_arming_requests removed - ArmingHandler handles arming requests
// handle_prearm_check_requests removed - ArmingHandler handles prearm check requests
// handle_flight_termination_requests removed - FailsafeHandler handles flight termination requests

void SystemManager::handle_set_home_requests()
{
	set_home_request_s req;

	if (_set_home_request_sub.update(&req)) {
		if (!_param_com_home_en.get()) {
			return;  // Home position disabled
		}

		if (req.use_current_position) {
			if (_home_position.setHomePosition(false)) {
				_status_changed = true;
			}

		} else {
			home_position_s home{};
			home.timestamp = hrt_absolute_time();
			home.lat = req.latitude;
			home.lon = req.longitude;
			home.alt = req.altitude;
			home.manual_home = true;
			home.valid_alt = true;
			home.valid_hpos = true;
			home.valid_lpos = false;

			_home_position.setHomePosValid();
			// The home position would need to be published through the HomePosition class
		}
	}
}

// handle_commands_from_mode_executors removed - mode executor management now in mode_manager

// execute_action_request removed - ArmingHandler::handle_action_requests() handles this


void SystemManager::update_parameters()
{
	// update parameters from storage
	updateParams();

	int32_t value_int32 = 0;

	// MAV_TYPE -> vehicle_identity.system_type
	if ((_param_mav_type != PARAM_INVALID) && (param_get(_param_mav_type, &value_int32) == PX4_OK)) {
		_vehicle_identity.system_type = value_int32;
	}

	_auto_disarm_killed.set_hysteresis_time_from(false, _param_com_kill_disarm.get() * 1_s);

	const bool is_rotary = is_rotary_wing(_vehicle_identity) || (is_vtol(_vehicle_identity)
			       && _vtol_vehicle_status.vehicle_vtol_state != vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const bool is_fixed = is_fixed_wing(_vehicle_identity) || (is_vtol(_vehicle_identity)
			      && _vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
	const bool is_ground = is_ground_vehicle(_vehicle_identity);

	/* disable manual override for all systems that rely on electronic stabilization */
	if (is_rotary) {
		_vehicle_identity.vehicle_type = vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING;

	} else if (is_fixed) {
		_vehicle_identity.vehicle_type = vehicle_identity_s::VEHICLE_TYPE_FIXED_WING;

	} else if (is_ground) {
		_vehicle_identity.vehicle_type = vehicle_identity_s::VEHICLE_TYPE_ROVER;
	}

	_vehicle_status.is_vtol = is_vtol(_vehicle_identity);
	_vehicle_status.is_vtol_tailsitter = is_vtol_tailsitter(_vehicle_identity);

	// _mode_switch_mapped = (RC_MAP_FLTMODE > 0)
	if (_param_rc_map_fltmode != PARAM_INVALID && (param_get(_param_rc_map_fltmode, &value_int32) == PX4_OK)) {
		_mode_switch_mapped = (value_int32 > 0);
	}

}

void SystemManager::run()
{
	/* initialize */
	led_init();
	buzzer_init();

#if defined(BOARD_HAS_POWER_CONTROL)
	{
		// we need to do an initial publication to make sure uORB allocates the buffer, which cannot happen
		// in IRQ context.
		power_button_state_s button_state{};
		button_state.timestamp = hrt_absolute_time();
		button_state.event = 0xff;
		power_button_state_pub = orb_advertise(ORB_ID(power_button_state), &button_state);

		_power_button_state_sub.copy(&button_state);

		tune_control_s tune_control{};
		button_state.timestamp = hrt_absolute_time();
		tune_control_pub = orb_advertise(ORB_ID(tune_control), &tune_control);
	}

	if (board_register_power_state_notification_cb(power_button_state_notification_cb) != 0) {
		PX4_ERR("Failed to register power notification callback");
	}

#endif // BOARD_HAS_POWER_CONTROL

	_boot_timestamp = hrt_absolute_time();

	arm_auth_init(&_mavlink_log_pub, &_vehicle_status.system_id);

	while (!should_exit()) {

		perf_begin(_loop_perf);

		const actuator_armed_s actuator_armed_prev{_arming_handler.actuator_armed()};

		/* update parameters */
		const bool params_updated = _parameter_update_sub.updated();

		if (params_updated) {
			// clear update
			parameter_update_s update;
			_parameter_update_sub.copy(&update);

			update_parameters();

			_status_changed = true;
		}

		handle_power_button_state();

		system_power_update();

		land_detector_update();

		safety_button_update();

		vtol_status_update();

		_home_position.update(_param_com_home_en.get(), !is_armed() && _vehicle_land_detected.landed);

		// Use ArmingHandler for auto-disarm
		_arming_handler.set_vehicle_status(&_vehicle_status);
		_arming_handler.set_vehicle_control_mode(&_vehicle_control_mode_sub.get());
		_arming_handler.set_vehicle_land_detected(&_vehicle_land_detected);
		_arming_handler.set_failsafe_flags(&_failsafe_flags);
		_arming_handler.set_current_mode(_current_operation_mode);
		_arming_handler.set_throttle_state(_is_throttle_above_center, _is_throttle_low);
		_arming_handler.set_have_taken_off(_have_taken_off_since_arming);
		_arming_handler.set_disarm_params(_param_com_disarm_land.get(), _param_com_disarm_prflt.get(),
						  _param_com_kill_disarm.get(), _param_com_force_safety.get(),
						  _param_com_disarm_man.get(), _param_com_home_en.get());
		_arming_handler.set_rc_calibration_in_progress(_vehicle_status.rc_calibration_in_progress);
		_arming_handler.handle_auto_disarm();

		battery_status_check();

		check_mission_result();

		manual_control_check();

		// data link checks which update the status
		data_link_check();

		// Check for failure detector status
		_vehicle_control_mode_sub.update();
		if (_failure_detector.update(_vehicle_status, _vehicle_control_mode_sub.get())) {
			_vehicle_status.failure_detector_status = _failure_detector.getStatus().value;
			_status_changed = true;
		}

		const hrt_abstime now = hrt_absolute_time();

		// Use FailsafeHandler for failsafe logic
		bool mode_change_requested = _had_mode_change;
		_had_mode_change = false;
		const bool operation_mode_or_failsafe_changed = _failsafe_handler.update(
			_vehicle_status,
			_failsafe_flags,
			_config_overrides,
			_cached_user_intended_mode,
			mode_change_requested,
			_failsafe_user_override_request);
		_failsafe_user_override_request = false;
		_failsafe_handler.handle_flight_termination_requests();

		// Update vehicle_status.operation_mode from mode_manager's ModeStatus
		update_mode_status();

		// Run arming checks @ 10Hz
		if ((now >= _last_health_and_arming_check + 100_ms) || _status_changed || operation_mode_or_failsafe_changed) {
			_last_health_and_arming_check = now;

			perf_begin(_prearm_check_perf);
			_health_and_arming_checks.update(_current_operation_mode);
			bool pre_flight_checks_pass = _health_and_arming_checks.canArm(_current_operation_mode);

			if (_vehicle_status.pre_flight_checks_pass != pre_flight_checks_pass) {
				_vehicle_status.pre_flight_checks_pass = pre_flight_checks_pass;
				_status_changed = true;
			}

			perf_end(_prearm_check_perf);
			check_and_inform_ready_for_takeoff();
		}

		// Handle requests from CommandProcessor
		// Note: ModeChangeRequests are handled by mode_manager directly
		if (_arming_handler.handle_arming_requests()) {
			_status_changed = true;
		}

		// Use CalibrationHandler for calibration, storage, reboot, and actuator test
		_calibration_handler.set_armed(is_armed());
		_calibration_handler.set_motor_test_enabled(_param_com_mot_test_en.get() == 1);
		_calibration_handler.handle_calibration_requests();
		_calibration_handler.handle_reboot_requests();
		_calibration_handler.handle_storage_requests();
		_calibration_handler.handle_actuator_test_requests();
		_calibration_handler.check_worker_thread();

		// Update vehicle_status with calibration state
		_vehicle_status.calibration_enabled = _calibration_handler.get_calibration_enabled();
		_vehicle_status.rc_calibration_in_progress = _calibration_handler.get_rc_calibration_in_progress();

		// Handle prearm check and action requests through ArmingHandler
		_arming_handler.handle_prearm_check_requests();
		if (_arming_handler.handle_action_requests()) {
			_status_changed = true;
		}

		handle_set_home_requests();

		// Sync actuator_armed state from ArmingHandler
		const actuator_armed_s &handler_armed = _arming_handler.actuator_armed();
		actuator_armed_s &arming_state = _arming_handler.actuator_armed_mut();

		// update actuator_armed from handler and local state
		arming_state.armed = is_armed();
		arming_state.prearmed = _arming_handler.get_prearm_state();
		arming_state.ready_to_arm = _vehicle_status.pre_flight_checks_pass || is_armed();
		arming_state.lockdown = ((_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON)
					    || _throw_launch_status_sub.get().throw_launch_in_progress);
		// arming_state.manual_lockdown set by ArmingHandler::handle_kill_switch()
		arming_state.force_failsafe = (_current_operation_mode == _vehicle_status.OPERATION_MODE_TERMINATION);
		arming_state.in_esc_calibration_mode = _calibration_handler.is_esc_calibration_in_progress();

		// if force_failsafe or manual_lockdown activated send parachute command
		if ((!actuator_armed_prev.force_failsafe && handler_armed.force_failsafe)
		    || (!actuator_armed_prev.manual_lockdown && handler_armed.manual_lockdown)
		   ) {
			if (is_armed()) {
				send_parachute_command();
			}
		}

		// publish states (armed, control_mode, vehicle_status, failure_detector_status) at 2 Hz or immediately when changed
		if ((now >= _vehicle_status.timestamp + 500_ms) || _status_changed || operation_mode_or_failsafe_changed
		    || !(handler_armed == actuator_armed_prev)) {

			// publish actuator_armed first (used by output modules)
			_arming_handler.publish_actuator_armed();

			// Note: vehicle_control_mode is now published by mode_manager

			// vehicle_status publish (after prearm updates above)
			_vehicle_status.timestamp = hrt_absolute_time();
			_vehicle_status_pub.publish(_vehicle_status);

			// vehicle_identity publish
			_vehicle_identity.timestamp = hrt_absolute_time();
			_vehicle_identity_pub.publish(_vehicle_identity);

			// failure_detector_status publish
			failure_detector_status_s fd_status{};
			fd_status.fd_roll = _failure_detector.getStatusFlags().roll;
			fd_status.fd_pitch = _failure_detector.getStatusFlags().pitch;
			fd_status.fd_alt = _failure_detector.getStatusFlags().alt;
			fd_status.fd_ext = _failure_detector.getStatusFlags().ext;
			fd_status.fd_arm_escs = _failure_detector.getStatusFlags().arm_escs;
			fd_status.fd_battery = _failure_detector.getStatusFlags().battery;
			fd_status.fd_imbalanced_prop = _failure_detector.getStatusFlags().imbalanced_prop;
			fd_status.fd_motor = _failure_detector.getStatusFlags().motor;
			fd_status.imbalanced_prop_metric = _failure_detector.getImbalancedPropMetric();
			fd_status.motor_failure_mask = _failure_detector.getMotorFailures();
			fd_status.timestamp = hrt_absolute_time();
			_failure_detector_status_pub.publish(fd_status);
		}

		check_worker_thread();

		update_tunes();
		control_status_leds(_status_changed, _battery_warning);

		_status_changed = false;

		arm_auth_update(hrt_absolute_time(), params_updated);

		px4_indicate_external_reset_lockout(LockoutComponent::Commander, is_armed());

		perf_end(_loop_perf);

		// sleep if there are no vehicle_commands or action_requests to process
		if (!_vehicle_command_sub.updated() && !_action_request_sub.updated()) {
			px4_usleep(COMMANDER_MONITORING_INTERVAL);
		}
	}

	rgbled_set_color_and_mode(led_control_s::COLOR_WHITE, led_control_s::MODE_OFF);

	/* close fds */
	led_deinit();
	buzzer_deinit();
}

void SystemManager::check_mission_result()
{
	if (_mission_result_sub.updated()) {
		const mission_result_s &mission_result = _mission_result_sub.get();

		const uint32_t prev_mission_mission_id = mission_result.mission_id;
		_mission_result_sub.update();

		// if mission_result is valid for the current mission
		const bool mission_result_ok = (mission_result.timestamp > _boot_timestamp)
					       && (mission_result.mission_id > 0);

		bool auto_mission_available = mission_result_ok && mission_result.valid;

		if (mission_result_ok) {
			/* Only evaluate mission state if home is set */
			if (!_failsafe_flags.home_position_invalid &&
			    (prev_mission_mission_id != mission_result.mission_id)) {

				if (!auto_mission_available) {
					/* the mission is invalid */
					tune_mission_fail(true);

				} else if (mission_result.warning) {
					/* the mission has a warning */
					tune_mission_warn(true);

				} else {
					/* the mission is valid */
					tune_mission_ok(true);
				}
			}
		}

		// Note: Mode transitions after takeoff/mission completion are now handled by
		// the automation module in mode_completed()
	}
}

bool SystemManager::get_prearm_state() const
{
	if (_vehicle_status.calibration_enabled) {
		return false;
	}

	switch ((PrearmedMode)_param_com_prearm_mode.get()) {
	case PrearmedMode::DISABLED:
		/* skip prearmed state  */
		return false;

	case PrearmedMode::ALWAYS:
		/* safety is not present, go into prearmed
		* (all output drivers should be started / unlocked last in the boot process
		* when the rest of the system is fully initialized)
		*/
		return hrt_elapsed_time(&_boot_timestamp) > 5_s;

	case PrearmedMode::SAFETY_BUTTON:
		if (_safety.isButtonAvailable()) {
			/* safety button is present, go into prearmed if safety is off */
			return _safety.isSafetyOff();
		}

		/* safety button is not present, do not go into prearmed */
		return false;
	}

	return false;
}

void SystemManager::handle_power_button_state()
{
#if defined(BOARD_HAS_POWER_CONTROL)

	/* handle power button state */
	if (_power_button_state_sub.updated()) {
		power_button_state_s button_state;

		if (_power_button_state_sub.copy(&button_state)) {
			if (button_state.event == power_button_state_s::PWR_BUTTON_STATE_REQUEST_SHUTDOWN) {
				if (!is_armed() && (px4_shutdown_request() == 0)) {
					while (1) { px4_usleep(1); }
				}
			}
		}
	}

#endif // BOARD_HAS_POWER_CONTROL
}

void SystemManager::system_power_update()
{
	system_power_s system_power;

	if (_system_power_sub.update(&system_power)) {

		if (hrt_elapsed_time(&system_power.timestamp) < 1_s) {
			if (system_power.servo_valid &&
			    !system_power.brick_valid &&
			    !system_power.usb_connected) {
				/* flying only on servo rail, this is unsafe */
				_vehicle_status.power_input_valid = false;

			} else {
				_vehicle_status.power_input_valid = true;
			}
		}
	}
}

void SystemManager::land_detector_update()
{
	if (_vehicle_land_detected_sub.updated()) {
		const bool was_landed = _vehicle_land_detected.landed;
		_vehicle_land_detected_sub.copy(&_vehicle_land_detected);

		// Only take actions if armed
		if (is_armed()) {
			if (!was_landed && _vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, "Landing detected\t");
				events::send(events::ID("commander_landing_detected"), events::Log::Info, "Landing detected");

			} else if (was_landed && !_vehicle_land_detected.landed) {
				mavlink_log_info(&_mavlink_log_pub, "Takeoff detected\t");
				events::send(events::ID("commander_takeoff_detected"), events::Log::Info, "Takeoff detected");
				_vehicle_status.takeoff_time = hrt_absolute_time();
				_have_taken_off_since_arming = true;
			}

			// automatically set or update home position
			if (_param_com_home_en.get()) {
				// set the home position when taking off
				if (!_vehicle_land_detected.landed) {
					if (was_landed) {
						_home_position.setHomePosition();

					} else if (_param_com_home_in_air.get()) {
						_home_position.setInAirHomePosition();
					}
				}
			}
		}
	}
}

void SystemManager::safety_button_update()
{
	const bool safety_changed = _safety.safetyButtonHandler();
	_vehicle_status.safety_button_available = _safety.isButtonAvailable();
	_vehicle_status.safety_off = _safety.isSafetyOff();

	if (safety_changed) {
		// Notify the user if the status of the safety button changes
		if (!_safety.isSafetyDisabled()) {
			if (_safety.isSafetyOff()) {
				set_tune(tune_control_s::TUNE_ID_NOTIFY_POSITIVE);

			} else {
				tune_neutral(true);
			}
		}

		_status_changed = true;
	}
}

void SystemManager::vtol_status_update()
{
	// Make sure that this is only adjusted if vehicle really is of type vtol
	if (_vtol_vehicle_status_sub.update(&_vtol_vehicle_status) && is_vtol(_vehicle_status)) {

		// Check if there has been any change while updating the flags (transition = rotary wing status)
		const uint8_t new_vehicle_type =
			_vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW ?
			vehicle_identity_s::VEHICLE_TYPE_FIXED_WING :
			vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING;

		if (new_vehicle_type != _vehicle_status.vehicle_type) {
			_vehicle_status.vehicle_type = new_vehicle_type;
			_status_changed = true;
		}

		const bool new_in_transition = _vtol_vehicle_status.vehicle_vtol_state ==
					       vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW
					       || _vtol_vehicle_status.vehicle_vtol_state == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_MC;

		if (_vehicle_status.in_transition_mode != new_in_transition) {
			_vehicle_status.in_transition_mode = new_in_transition;
			_status_changed = true;
		}

		if (_vehicle_status.in_transition_to_fw != (_vtol_vehicle_status.vehicle_vtol_state ==
				vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW)) {
			_vehicle_status.in_transition_to_fw = (_vtol_vehicle_status.vehicle_vtol_state ==
							       vtol_vehicle_status_s::VEHICLE_VTOL_STATE_TRANSITION_TO_FW);
			_status_changed = true;
		}

	}
}

void SystemManager::update_tunes()
{
	// play arming and battery warning tunes
	if (!_arm_tune_played && is_armed()) {

		/* play tune when armed */
		set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);
		_arm_tune_played = true;

	} else if (!_vehicle_status.usb_connected &&
		   (_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
		   (_battery_warning == battery_status_s::WARNING_CRITICAL)) {

		/* play tune on battery critical */
		set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_FAST);

	} else if ((_vehicle_status.hil_state != vehicle_status_s::HIL_STATE_ON) &&
		   (_battery_warning == battery_status_s::WARNING_LOW)) {
		/* play tune on battery warning */
		set_tune(tune_control_s::TUNE_ID_BATTERY_WARNING_SLOW);

	} else if (_vehicle_status.failsafe && is_armed()) {
		tune_failsafe(true);

	} else if (_throw_launch_status_sub.get().ready_to_throw) {
		set_tune(tune_control_s::TUNE_ID_ARMING_WARNING);

	} else {
		set_tune(tune_control_s::TUNE_ID_STOP);
	}

	/* reset arm_tune_played when disarmed */
	if (!is_armed()) {

		// Notify the user that it is safe to approach the vehicle
		if (_arm_tune_played) {
			tune_neutral(true);
		}

		_arm_tune_played = false;
	}
}

void SystemManager::check_worker_thread()
{
	// check if the worker has finished
	if (_worker_thread.hasResult()) {
		int ret = _worker_thread.getResultAndReset();

		// Sync ESC calibration state with handler
		_calibration_handler.set_esc_calibration_mode(false);

		if (_calibration_handler.get_calibration_enabled()) { // did we do a calibration?
			// Reset calibration state through handler (already done by check_worker_thread)
			// but we need to play tunes
			if (ret == 0) {
				tune_positive(true);

			} else {
				tune_negative(true);
			}
		}

		// Sync calibration enabled state
		_vehicle_status.calibration_enabled = _calibration_handler.get_calibration_enabled();
	}
}

// handle_auto_disarm removed - ArmingHandler::handle_auto_disarm() handles this
// failsafe_mode_change removed - FailsafeHandler::update() handles this
// publish_failsafe_request_to_mode_manager removed - FailsafeHandler handles this
// handle_critical_failsafe_actions removed - FailsafeHandler::handle_critical_actions() handles this
// update_failsafe_defer_state removed - FailsafeHandler::update_defer_state() handles this

void SystemManager::publish_mode_change_request(uint8_t requested_mode, uint8_t source)
{
	mode_change_request_s request{};
	request.timestamp = hrt_absolute_time();
	request.requested_mode = requested_mode;
	request.source = source;
	request.allow_fallback = false;
	request.force = false;
	request.cmd_source_system = 0;
	request.cmd_source_component = 0;
	request.cmd_command = 0;

	_mode_change_request_pub.publish(request);
}

// publish_failsafe_mode_request removed - FailsafeHandler handles this

void SystemManager::update_mode_status()
{
	_mode_status_sub.update();
	const mode_status_s &mode_status = _mode_status_sub.get();

	// Only update if we have valid mode status from mode_manager
	if (mode_status.timestamp == 0) {
		return;
	}

	// Track user intention changes (for failsafe user-takeover detection)
	if (_cached_user_intended_mode != mode_status.user_intended_mode) {
		_cached_user_intended_mode = mode_status.user_intended_mode;
		_had_mode_change = true;
		_ever_had_mode_change = true;
	}

	// mode_manager is the authoritative source for current operation mode
	if (_current_operation_mode != mode_status.current_mode) {
		_current_operation_mode = mode_status.current_mode;
		_current_operation_mode_timestamp = hrt_absolute_time();
		_status_changed = true;
	}

	// Sync user intention tracking
	_current_operation_mode_user_intention = mode_status.user_intended_mode;

	// Update vehicle_status from mode_manager
	_vehicle_status.operation_mode = mode_status.current_mode;
	_vehicle_status.mode_executor_in_charge = mode_status.mode_executor_in_charge;
	_vehicle_status.valid_operation_modes_mask = mode_status.valid_modes_mask;
	_vehicle_status.can_set_operation_modes_mask = mode_status.can_set_modes_mask;

	// Note: failsafe flag is set by failsafe_mode_change()
	// based on local failsafe evaluation, not from mode_status
}

void SystemManager::check_and_inform_ready_for_takeoff()
{
#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	static bool ready_for_takeoff_printed = false;

	if (_vehicle_status.vehicle_type == vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING ||
	    _vehicle_status.vehicle_type == vehicle_identity_s::VEHICLE_TYPE_FIXED_WING) {
		if (!ready_for_takeoff_printed &&
		    _health_and_arming_checks.canArm(mode_status_s::OPERATION_MODE_AUTO_TAKEOFF)) {
			PX4_INFO("%sReady for takeoff!%s", PX4_ANSI_COLOR_GREEN, PX4_ANSI_COLOR_RESET);
			ready_for_takeoff_printed = true;
		}
	}

#endif // CONFIG_ARCH_BOARD_PX4_SITL
}

void SystemManager::control_status_leds(bool changed, const uint8_t battery_warning)
{
	switch (blink_msg_state()) {
	case 1:
		// blinking LED message, don't touch LEDs
		return;

	case 2:
		// blinking LED message completed, restore normal state
		changed = true;
		break;

	default:
		break;
	}

	const hrt_abstime time_now_us = hrt_absolute_time();

	if (_cpuload_sub.updated()) {
		cpuload_s cpuload;

		if (_cpuload_sub.copy(&cpuload)) {
			const float cpuload_percent = cpuload.load * 100.f;

			bool overload = false;

			// Only check CPU load if it hasn't been disabled
			if (!(_param_com_cpu_max.get() < FLT_EPSILON)) {
				overload = (cpuload_percent > _param_com_cpu_max.get());
			}

			overload = overload || (cpuload.ram_usage > 0.99f);

			if (_overload_start == 0 && overload) {
				_overload_start = time_now_us;

			} else if (!overload) {
				_overload_start = 0;
			}
		}
	}

	const bool overload = (_overload_start != 0);

	// driving the RGB led
	if (changed || _last_overload != overload) {
		uint8_t led_mode = led_control_s::MODE_OFF;
		uint8_t led_color = led_control_s::COLOR_WHITE;
		bool set_normal_color = false;

		uint64_t overload_warn_delay = is_armed() ? 1_ms : 250_ms;

		// set mode
		if (overload && (time_now_us >= _overload_start + overload_warn_delay)) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_PURPLE;

		} else if (_throw_launch_status_sub.get().ready_to_throw) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_YELLOW;

		} else if (is_armed()) {
			led_mode = led_control_s::MODE_ON;
			set_normal_color = true;

		} else if (!_vehicle_status.pre_flight_checks_pass) {
			led_mode = led_control_s::MODE_BLINK_FAST;
			led_color = led_control_s::COLOR_RED;

		} else {
			led_mode = led_control_s::MODE_BREATHE;
			set_normal_color = true;
		}

		if (set_normal_color) {
			// set color
			if (_vehicle_status.failsafe) {
				led_color = led_control_s::COLOR_PURPLE;

			} else if (battery_warning == battery_status_s::WARNING_LOW) {
				led_color = led_control_s::COLOR_AMBER;

			} else if (battery_warning == battery_status_s::WARNING_CRITICAL) {
				led_color = led_control_s::COLOR_RED;

			} else {
				if (!_failsafe_flags.home_position_invalid && !_failsafe_flags.global_position_invalid) {
					led_color = led_control_s::COLOR_GREEN;

				} else {
					led_color = led_control_s::COLOR_BLUE;
				}
			}
		}

		if (led_mode != led_control_s::MODE_OFF) {
			rgbled_set_color_and_mode(led_color, led_mode);
		}
	}

	_last_overload = overload;

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)

	if (is_armed()) {
		if (_vehicle_status.failsafe) {
			BOARD_ARMED_LED_OFF();

			if (time_now_us >= _led_armed_state_toggle + 250_ms) {
				_led_armed_state_toggle = time_now_us;
				BOARD_ARMED_STATE_LED_TOGGLE();
			}

		} else {
			BOARD_ARMED_STATE_LED_OFF();

			// armed, solid
			BOARD_ARMED_LED_ON();
		}

	} else if (_vehicle_status.pre_flight_checks_pass) {
		BOARD_ARMED_LED_OFF();

		// ready to arm, blink at 1Hz
		if (time_now_us >= _led_armed_state_toggle + 1_s) {
			_led_armed_state_toggle = time_now_us;
			BOARD_ARMED_STATE_LED_TOGGLE();
		}

	} else {
		BOARD_ARMED_LED_OFF();

		// not ready to arm, blink at 10Hz
		if (time_now_us >= _led_armed_state_toggle + 100_ms) {
			_led_armed_state_toggle = time_now_us;
			BOARD_ARMED_STATE_LED_TOGGLE();
		}
	}

#endif

	// give system warnings on error LED
	if (overload) {
		if (time_now_us >= _led_overload_toggle + 50_ms) {
			_led_overload_toggle = time_now_us;
			BOARD_OVERLOAD_LED_TOGGLE();
		}

	} else {
		BOARD_OVERLOAD_LED_OFF();
	}
}

// Note: update_control_mode logic moved to mode_manager

void SystemManager::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
	switch (result) {
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED:
		PX4_DEBUG("command %" PRIu32 " denied", cmd.command);
		tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED:
		PX4_DEBUG("command %" PRIu32 " failed", cmd.command);
		tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		PX4_DEBUG("command %" PRIu32 " temporarily rejected", cmd.command);
		tune_negative(true);
		break;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		PX4_WARN("command %" PRIu32 " unsupported", cmd.command);
		tune_negative(true);
		break;

	default:
		PX4_ERR("command %" PRIu32 " invalid result %d", cmd.command, result);
		return;
	}

	/* publish ACK */
	vehicle_command_ack_s command_ack{};
	command_ack.command = cmd.command;
	command_ack.result = result;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.timestamp = hrt_absolute_time();
	_vehicle_command_ack_pub.publish(command_ack);
}

int SystemManager::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("system_manager",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 40,
				      PX4_STACK_ADJUSTED(3250),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return 0;
}

SystemManager *SystemManager::instantiate(int argc, char *argv[])
{
	SystemManager *instance = new SystemManager();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-h")) {
			instance->enable_hil();
		}
	}

	return instance;
}

void SystemManager::enable_hil()
{
	_vehicle_status.hil_state = vehicle_status_s::HIL_STATE_ON;
}

void SystemManager::data_link_check()
{
	// high latency data link
	iridiumsbd_status_s iridium_status;

	if (_iridiumsbd_status_sub.update(&iridium_status)) {
		_high_latency_datalink_timestamp = iridium_status.last_at_ok_timestamp;

		if (_vehicle_status.high_latency_data_link_lost &&
		    (_high_latency_datalink_timestamp > _high_latency_datalink_lost) &&
		    (_high_latency_datalink_regained == 0)
		   ) {
			_high_latency_datalink_regained = _high_latency_datalink_timestamp;
		}

		if (_vehicle_status.high_latency_data_link_lost &&
		    (_high_latency_datalink_regained != 0) &&
		    (hrt_elapsed_time(&_high_latency_datalink_regained) > (_param_com_hldl_reg_t.get() * 1_s))
		   ) {
			_vehicle_status.high_latency_data_link_lost = false;
			_status_changed = true;
		}
	}

	for (auto &telemetry_status :  _telemetry_status_subs) {
		telemetry_status_s telemetry;

		if (telemetry_status.update(&telemetry)) {

			// handle different radio types
			switch (telemetry.type) {
			case telemetry_status_s::LINK_TYPE_USB:
				// set (but don't unset) telemetry via USB as active once a MAVLink connection is up
				_vehicle_status.usb_connected = true;
				break;

			case telemetry_status_s::LINK_TYPE_IRIDIUM: {

					if ((_high_latency_datalink_timestamp > 0) &&
					    (hrt_elapsed_time(&_high_latency_datalink_timestamp) > (_param_com_hldl_loss_t.get() * 1_s))) {

						_high_latency_datalink_lost = _high_latency_datalink_timestamp;
						_high_latency_datalink_regained = 0;

						if (!_vehicle_status.high_latency_data_link_lost) {
							_vehicle_status.high_latency_data_link_lost = true;
							mavlink_log_critical(&_mavlink_log_pub, "High latency data link lost\t");
							events::send(events::ID("commander_high_latency_lost"), events::Log::Critical, "High latency data link lost");
							_status_changed = true;
						}
					}

					break;
				}
			}

			if (telemetry.heartbeat_type_gcs) {
				// Initial connection or recovery from data link lost
				if (_vehicle_status.gcs_connection_lost) {
					_vehicle_status.gcs_connection_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_gcs != 0) {
						mavlink_log_info(&_mavlink_log_pub, "GCS connection regained\t");
						events::send(events::ID("commander_dl_regained"), events::Log::Info, "GCS connection regained");
					}
				}

				_datalink_last_heartbeat_gcs = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_onboard_controller) {
				if (_onboard_controller_lost) {
					_onboard_controller_lost = false;
					_status_changed = true;

					if (_datalink_last_heartbeat_onboard_controller != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Onboard controller regained\t");
						events::send(events::ID("commander_onboard_ctrl_regained"), events::Log::Info, "Onboard controller regained");
					}
				}

				_datalink_last_heartbeat_onboard_controller = telemetry.timestamp;
			}

			if (telemetry.heartbeat_type_parachute) {
				if (_parachute_system_lost) {
					_parachute_system_lost = false;

					if (_datalink_last_heartbeat_parachute_system != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Parachute system regained\t");
						events::send(events::ID("commander_parachute_regained"), events::Log::Info, "Parachute system regained");
					}
				}

				bool healthy = telemetry.parachute_system_healthy;

				_datalink_last_heartbeat_parachute_system = telemetry.timestamp;
				_vehicle_status.parachute_system_present = true;
				_vehicle_status.parachute_system_healthy = healthy;
			}

			if (telemetry.heartbeat_type_open_drone_id) {
				if (_open_drone_id_system_lost) {
					_open_drone_id_system_lost = false;

					if (_datalink_last_heartbeat_open_drone_id_system != 0) {
						mavlink_log_info(&_mavlink_log_pub, "Remote ID system regained\t");
						events::send(events::ID("commander_open_drone_id_regained"), events::Log::Info, "Remote ID system regained");
					}
				}

				bool healthy = telemetry.open_drone_id_system_healthy;

				_datalink_last_heartbeat_open_drone_id_system = telemetry.timestamp;
				_vehicle_status.open_drone_id_system_present = true;
				_vehicle_status.open_drone_id_system_healthy = healthy;
			}
		}
	}


	// GCS data link loss failsafe
	if (!_vehicle_status.gcs_connection_lost) {
		if ((_datalink_last_heartbeat_gcs != 0)
		    && hrt_elapsed_time(&_datalink_last_heartbeat_gcs) > (_param_com_dl_loss_t.get() * 1_s)) {

			_vehicle_status.gcs_connection_lost = true;
			_vehicle_status.gcs_connection_lost_counter++;

			mavlink_log_info(&_mavlink_log_pub, "Connection to ground station lost\t");
			events::send(events::ID("commander_gcs_lost"), {events::Log::Warning, events::LogInternal::Info},
				     "Connection to ground control station lost");

			_status_changed = true;
		}
	}

	// ONBOARD CONTROLLER data link loss failsafe
	if ((_datalink_last_heartbeat_onboard_controller > 0)
	    && (hrt_elapsed_time(&_datalink_last_heartbeat_onboard_controller) > (_param_com_obc_loss_t.get() * 1_s))
	    && !_onboard_controller_lost) {

		mavlink_log_critical(&_mavlink_log_pub, "Connection to mission computer lost\t");
		events::send(events::ID("commander_mission_comp_lost"), events::Log::Critical, "Connection to mission computer lost");
		_onboard_controller_lost = true;
		_status_changed = true;
	}

	// Parachute system
	if ((hrt_elapsed_time(&_datalink_last_heartbeat_parachute_system) > 3_s)
	    && !_parachute_system_lost) {
		mavlink_log_critical(&_mavlink_log_pub, "Parachute system lost");
		_vehicle_status.parachute_system_present = false;
		_vehicle_status.parachute_system_healthy = false;
		_parachute_system_lost = true;
		_status_changed = true;
	}

	// Remote ID system
	if ((hrt_elapsed_time(&_datalink_last_heartbeat_open_drone_id_system) > 3_s)
	    && !_open_drone_id_system_lost) {
		mavlink_log_critical(&_mavlink_log_pub, "Remote ID system lost");
		events::send(events::ID("commander_remote_id_lost"), events::Log::Critical, "Remote ID system lost");
		_vehicle_status.open_drone_id_system_present = false;
		_vehicle_status.open_drone_id_system_healthy = false;
		_open_drone_id_system_lost = true;
		_status_changed = true;
	}
}

void SystemManager::battery_status_check()
{
	// Handle shutdown request from emergency battery action
	if (_battery_warning != _failsafe_flags.battery_warning) {

		if (_failsafe_flags.battery_warning == battery_status_s::WARNING_EMERGENCY) {
#if defined(BOARD_HAS_POWER_CONTROL)

			if (!is_armed() && (px4_shutdown_request(60_s) == 0)) {
				mavlink_log_critical(&_mavlink_log_pub, "Dangerously low battery! Shutting system down in 60 seconds\t");
				events::send(events::ID("commander_low_bat_shutdown"), {events::Log::Emergency, events::LogInternal::Warning},
					     "Dangerously low battery! Shutting system down");

				while (1) { px4_usleep(1); }

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "System does not support shutdown\t");
				/* EVENT
				 * @description Cannot shut down, most likely the system does not support it.
				 */
				events::send(events::ID("commander_low_bat_shutdown_failed"), {events::Log::Emergency, events::LogInternal::Error},
					     "Dangerously low battery! System shut down failed");
			}

#endif // BOARD_HAS_POWER_CONTROL
		}
	}

	_battery_warning = _failsafe_flags.battery_warning;
}

void SystemManager::manual_control_check()
{
	manual_control_setpoint_s manual_control_setpoint;
	const bool manual_control_updated = _manual_control_setpoint_sub.update(&manual_control_setpoint);

	if (manual_control_updated && manual_control_setpoint.valid) {

		_is_throttle_above_center = (manual_control_setpoint.throttle > 0.2f);
		_is_throttle_low = (manual_control_setpoint.throttle < -0.8f);

		const vehicle_control_mode_s &control_mode = _vehicle_control_mode_sub.get();

		if (is_armed()) {
			// Abort autonomous mode and switch to position mode if sticks are moved significantly
			// but only if actually in air.
			if (manual_control_setpoint.sticks_moving
			    && !control_mode.flag_control_manual_enabled
			    && (_vehicle_status.vehicle_type == vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING)
			   ) {
				bool override_enabled = false;

				if (control_mode.flag_control_auto_enabled) {
					if (_param_com_rc_override.get() & static_cast<int32_t>(RcOverrideBits::AUTO_MODE_BIT)) {
						override_enabled = true;
					}
				}

				if (control_mode.flag_control_offboard_enabled) {
					if (_param_com_rc_override.get() & static_cast<int32_t>(RcOverrideBits::OFFBOARD_MODE_BIT)) {
						override_enabled = true;
					}
				}

				if (override_enabled) {
					// If no failsafe is active, directly change the mode, otherwise pass the request to the failsafe state machine
					if (_failsafe_handler.selectedAction() <= FailsafeBase::Action::Warn) {
						// Publish mode change request to mode_manager (RC override)
						publish_mode_change_request(mode_status_s::OPERATION_MODE_POSCTL,
									 mode_change_request_s::SOURCE_USER);
						tune_positive(true);
						mavlink_log_info(&_mavlink_log_pub, "Pilot took over using sticks\t");
						events::send(events::ID("commander_rc_override"), events::Log::Info, "Pilot took over using sticks");

					} else {
						_failsafe_user_override_request = true;
					}
				}
			}

		} else {
			const bool is_mavlink = (manual_control_setpoint.data_source > manual_control_setpoint_s::SOURCE_RC);

			// if there's never been a mode change force position control as initial state
			if (!_ever_had_mode_change && (is_mavlink || !_mode_switch_mapped)) {
				// Publish initial mode request to mode_manager
				publish_mode_change_request(mode_status_s::OPERATION_MODE_POSCTL,
							 mode_change_request_s::SOURCE_USER);
				_ever_had_mode_change = true;
			}
		}
	}
}

void SystemManager::send_parachute_command()
{
	vehicle_command_s vcmd{};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	vcmd.param1 = static_cast<float>(vehicle_command_s::PARACHUTE_ACTION_RELEASE);

	vcmd.source_system = _vehicle_status.system_id;
	vcmd.target_system = _vehicle_status.system_id;
	vcmd.source_component = _vehicle_status.component_id;
	vcmd.target_component = 161; // MAV_COMP_ID_PARACHUTE

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	vcmd_pub.publish(vcmd);

	set_tune_override(tune_control_s::TUNE_ID_PARACHUTE_RELEASE);
}

// on_failsafe_notify_user moved to FailsafeHandler
// get_external_mode_replacement removed - external mode management now in mode_manager
// get_executor_in_charge removed - mode executor management now in mode_manager
// update_external_control_mode removed - external mode management now in mode_manager
// update_external_config_overrides removed - external mode management now in mode_manager

int SystemManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The system_manager module contains the state machine for mode switching and failsafe behavior.

Note: Command-related commands (arm, disarm, mode, calibrate, etc.) have been moved to the
command_processor module. Use: command_processor <command> [args]
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("system_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Enable HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 1;
}
