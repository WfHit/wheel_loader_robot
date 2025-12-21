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
 * @file arming_handler.cpp
 *
 * Arming/disarming logic extracted from system_manager
 */

#include "arming_handler.hpp"
#include "system_manager_helper.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/mode_status.h>

using namespace time_literals;

static constexpr const char *arm_disarm_reason_str(arm_disarm_reason_t calling_reason)
{
	switch (calling_reason) {
	case arm_disarm_reason_t::transition_to_standby:
		return "";
	case arm_disarm_reason_t::stick_gesture:
		return "Stick gesture";
	case arm_disarm_reason_t::rc_switch:
		return "RC switch";
	case arm_disarm_reason_t::command_internal:
		return "internal command";
	case arm_disarm_reason_t::command_external:
		return "external command";
	case arm_disarm_reason_t::mission_start:
		return "mission start";
	case arm_disarm_reason_t::auto_disarm_land:
		return "landing";
	case arm_disarm_reason_t::auto_disarm_prearm:
		return "auto prearm disarming";
	case arm_disarm_reason_t::kill_switch:
		return "kill-switch";
	case arm_disarm_reason_t::lockdown:
		return "lockdown";
	case arm_disarm_reason_t::failure_detector:
		return "failure detector";
	case arm_disarm_reason_t::shutdown:
		return "shutdown request";
	case arm_disarm_reason_t::unit_test:
		return "unit tests";
	case arm_disarm_reason_t::rc_button:
		return "RC (button)";
	case arm_disarm_reason_t::failsafe:
		return "failsafe";
	}
	return "";
}

ArmingHandler::ArmingHandler(SystemManager *parent,
			     HealthAndArmingChecks &health_checks,
			     Safety &safety,
			     HomePosition &home_position) :
	_parent(parent),
	_health_checks(health_checks),
	_safety(safety),
	_home_position(home_position)
{
}

bool ArmingHandler::is_armed() const
{
	return _vehicle_status && (_vehicle_status->arming_state == vehicle_status_s::ARMING_STATE_ARMED);
}

arming_transition_result_t ArmingHandler::arm(arm_disarm_reason_t calling_reason, bool run_prearm_checks)
{
	if (!_vehicle_status || !_vehicle_control_mode || !_failsafe_flags) {
		return TRANSITION_DENIED;
	}

	if (is_armed()) {
		return TRANSITION_NOT_CHANGED;
	}

	if (_vehicle_status->calibration_enabled
	    || _vehicle_status->rc_calibration_in_progress
	    || _actuator_armed.in_esc_calibration_mode) {

		mavlink_log_critical(&_mavlink_log_pub, "Arming denied: calibrating\t");
		events::send(events::ID("commander_arm_denied_calibrating"), {events::Log::Critical, events::LogInternal::Info},
			     "Arming denied: calibrating");
		tune_negative(true);
		return TRANSITION_DENIED;
	}

	// allow a grace period for re-arming: prearm checks don't need to pass during that time
	if (calling_reason == arm_disarm_reason_t::rc_switch
	    && ((_last_disarmed_timestamp != 0) && (hrt_elapsed_time(&_last_disarmed_timestamp) < 5_s))) {
		run_prearm_checks = false;
	}

	if (run_prearm_checks) {
		if (_vehicle_control_mode->flag_control_manual_enabled) {

			if (_vehicle_control_mode->flag_control_climb_rate_enabled &&
			    !_failsafe_flags->manual_control_signal_lost && _is_throttle_above_center) {

				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: throttle above center\t");
				events::send(events::ID("commander_arm_denied_throttle_center"), {events::Log::Critical, events::LogInternal::Info},
					     "Arming denied: throttle above center");
				tune_negative(true);
				return TRANSITION_DENIED;
			}

			if (!_vehicle_control_mode->flag_control_climb_rate_enabled &&
			    !_failsafe_flags->manual_control_signal_lost && !_is_throttle_low
			    && !is_ground_vehicle(*_vehicle_status)) {

				mavlink_log_critical(&_mavlink_log_pub, "Arming denied: high throttle\t");
				events::send(events::ID("commander_arm_denied_throttle_high"), {events::Log::Critical, events::LogInternal::Info},
					     "Arming denied: high throttle");
				tune_negative(true);
				return TRANSITION_DENIED;
			}

		} else if (calling_reason == arm_disarm_reason_t::stick_gesture
			   || calling_reason == arm_disarm_reason_t::rc_switch
			   || calling_reason == arm_disarm_reason_t::rc_button) {

			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: switch to manual mode first\t");
			events::send(events::ID("commander_arm_denied_not_manual"), {events::Log::Critical, events::LogInternal::Info},
				     "Arming denied: switch to manual mode first");
			tune_negative(true);
			return TRANSITION_DENIED;
		}

		_health_checks.update(_current_operation_mode, false, true);

		if (!_health_checks.canArm(_current_operation_mode)) {
			tune_negative(true);
			mavlink_log_critical(&_mavlink_log_pub, "Arming denied: Resolve system health failures first\t");
			events::send(events::ID("commander_arm_denied_resolve_failures"), {events::Log::Critical, events::LogInternal::Info},
				     "Arming denied: Resolve system health failures first");
			return TRANSITION_DENIED;
		}
	}

	_vehicle_status->armed_time = hrt_absolute_time();
	_vehicle_status->arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	_vehicle_status->latest_arming_reason = (uint8_t)calling_reason;

	mavlink_log_info(&_mavlink_log_pub, "Armed by %s\t", arm_disarm_reason_str(calling_reason));
	events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_armed_by"), events::Log::Info,
			"Armed by {1}", calling_reason);

	if (_param_home_en) {
		_home_position.setHomePosition();
	}

	return TRANSITION_CHANGED;
}

arming_transition_result_t ArmingHandler::disarm(arm_disarm_reason_t calling_reason, bool forced)
{
	if (!_vehicle_status || !_vehicle_control_mode || !_vehicle_land_detected) {
		return TRANSITION_DENIED;
	}

	if (!is_armed()) {
		return TRANSITION_NOT_CHANGED;
	}

	if (!forced) {
		const bool landed = (_vehicle_land_detected->landed || _vehicle_land_detected->maybe_landed
				     || is_ground_vehicle(*_vehicle_status));
		const bool mc_manual_thrust_mode = _vehicle_status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						   && _vehicle_control_mode->flag_control_manual_enabled
						   && !_vehicle_control_mode->flag_control_climb_rate_enabled;
		const bool commanded_by_rc = (calling_reason == arm_disarm_reason_t::stick_gesture)
					     || (calling_reason == arm_disarm_reason_t::rc_switch)
					     || (calling_reason == arm_disarm_reason_t::rc_button);

		if (!landed && !(mc_manual_thrust_mode && commanded_by_rc && _param_disarm_man)) {
			if (calling_reason != arm_disarm_reason_t::stick_gesture) {
				mavlink_log_critical(&_mavlink_log_pub, "Disarming denied: not landed\t");
				events::send(events::ID("commander_disarm_denied_not_landed"),
				{events::Log::Critical, events::LogInternal::Info},
				"Disarming denied: not landed");
			}
			return TRANSITION_DENIED;
		}
	}

	_vehicle_status->armed_time = 0;
	_vehicle_status->arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_vehicle_status->latest_disarming_reason = (uint8_t)calling_reason;
	_vehicle_status->takeoff_time = 0;

	_have_taken_off_since_arming = false;
	_last_disarmed_timestamp = hrt_absolute_time();

	// Notify mode_manager about disarm (it will restore safe mode)
	mode_change_request_s request{};
	request.timestamp = hrt_absolute_time();
	request.requested_mode = mode_status_s::OPERATION_MODE_AUTO_LOITER;
	request.source = mode_change_request_s::SOURCE_MODE_EXECUTOR;
	request.allow_fallback = false;
	request.force = false;
	_mode_change_request_pub.publish(request);

	mavlink_log_info(&_mavlink_log_pub, "Disarmed by %s\t", arm_disarm_reason_str(calling_reason));
	events::send<events::px4::enums::arm_disarm_reason_t>(events::ID("commander_disarmed_by"), events::Log::Info,
			"Disarmed by {1}", calling_reason);

	if (_param_force_safety) {
		_safety.activateSafety();
	}

	return TRANSITION_CHANGED;
}

bool ArmingHandler::handle_arming_requests()
{
	arming_request_s req;

	if (_arming_request_sub.update(&req)) {
		arming_transition_result_t result = TRANSITION_DENIED;
		arm_disarm_reason_t reason = arm_disarm_reason_t::command_external;

		if (req.request_arm) {
			result = arm(reason, !req.force);

		} else if (req.request_disarm) {
			result = disarm(reason, req.force);
		}

		if (result != TRANSITION_DENIED) {
			return true;
		}
	}

	return false;
}

void ArmingHandler::handle_prearm_check_requests()
{
	prearm_check_request_s req;

	if (_prearm_check_request_sub.update(&req)) {
		if (req.run_checks) {
			_health_checks.update(_current_operation_mode, true);
		}
	}
}

void ArmingHandler::handle_auto_disarm()
{
	if (!_vehicle_status || !_vehicle_land_detected) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (is_armed()) {
		// Auto-disarm when landed
		if (_param_disarm_land > 0.f && _have_taken_off_since_arming) {
			_auto_disarm_landed.set_hysteresis_time_from(false, _param_disarm_land * 1_s);
			_auto_disarm_landed.set_state_and_update(_vehicle_land_detected->landed, now);

			if (_auto_disarm_landed.get_state()) {
				if (disarm(arm_disarm_reason_t::auto_disarm_land) == TRANSITION_CHANGED) {
					return;
				}
			}
		}

		// Auto-disarm when killed
		if (_actuator_armed.manual_lockdown) {
			_auto_disarm_killed.set_state_and_update(true, now);

			if (_auto_disarm_killed.get_state()) {
				if (disarm(arm_disarm_reason_t::kill_switch, true) == TRANSITION_CHANGED) {
					return;
				}
			}

		} else {
			_auto_disarm_killed.set_state_and_update(false, now);
		}

	} else {
		// Auto-disarm when prearm timeout
		if (_param_disarm_prflt > 0.f && _actuator_armed.prearmed) {
			_auto_disarm_landed.set_hysteresis_time_from(false, _param_disarm_prflt * 1_s);
			_auto_disarm_landed.set_state_and_update(true, now);

			if (_auto_disarm_landed.get_state()) {
				disarm(arm_disarm_reason_t::auto_disarm_prearm);
			}

		} else {
			_auto_disarm_landed.set_state_and_update(false, now);
		}

		_auto_disarm_killed.set_state_and_update(false, now);
	}
}

bool ArmingHandler::handle_kill_switch(bool kill)
{
	bool changed = false;

	if (kill) {
		if (!_actuator_armed.manual_lockdown) {
			const char kill_switch_string[] = "Kill engaged\t";
			events::LogLevels log_levels{events::Log::Info};

			if (_vehicle_land_detected && _vehicle_land_detected->landed) {
				mavlink_log_info(&_mavlink_log_pub, kill_switch_string);
			} else {
				mavlink_log_critical(&_mavlink_log_pub, kill_switch_string);
				log_levels.external = events::Log::Critical;
			}

			events::send(events::ID("commander_kill_sw_engaged"), log_levels, "Kill engaged");
			_actuator_armed.manual_lockdown = true;
			changed = true;
		}

	} else {
		if (_actuator_armed.manual_lockdown) {
			mavlink_log_info(&_mavlink_log_pub, "Kill disengaged\t");
			events::send(events::ID("commander_kill_sw_disengaged"), events::Log::Info, "Kill disengaged");
			_actuator_armed.manual_lockdown = false;
			changed = true;
		}
	}

	return changed;
}

bool ArmingHandler::get_prearm_state() const
{
	if (!_vehicle_status || !_vehicle_control_mode) {
		return false;
	}

	switch (_vehicle_status->com_prearm_mode) {
	case 0:  // DISABLED
		return false;

	case 1:  // SAFETY_BUTTON
		if (_safety.isButtonAvailable()) {
			return _safety.isSafetyOff();
		}
		return is_armed();

	case 2:  // ALWAYS
	default:
		return true;
	}
}

bool ArmingHandler::handle_action_requests()
{
	if (!_action_request_sub.updated()) {
		return false;
	}

	const unsigned last_generation = _action_request_sub.get_last_generation();
	action_request_s action_request;

	if (!_action_request_sub.copy(&action_request)) {
		return false;
	}

	if (_action_request_sub.get_last_generation() != last_generation + 1) {
		PX4_ERR("action_request lost, generation %u -> %u", last_generation,
			_action_request_sub.get_last_generation());
	}

	// Silently ignore RC actions during RC calibration
	if (_rc_calibration_in_progress
	    && (action_request.source == action_request_s::SOURCE_STICK_GESTURE
		|| action_request.source == action_request_s::SOURCE_RC_SWITCH
		|| action_request.source == action_request_s::SOURCE_RC_BUTTON
		|| action_request.source == action_request_s::SOURCE_RC_MODE_SLOT)) {
		return false;
	}

	arm_disarm_reason_t arm_disarm_reason{};

	switch (action_request.source) {
	case action_request_s::SOURCE_STICK_GESTURE:
		arm_disarm_reason = arm_disarm_reason_t::stick_gesture;
		break;

	case action_request_s::SOURCE_RC_SWITCH:
		arm_disarm_reason = arm_disarm_reason_t::rc_switch;
		break;

	case action_request_s::SOURCE_RC_BUTTON:
		arm_disarm_reason = arm_disarm_reason_t::rc_button;
		break;

	default:
		arm_disarm_reason = arm_disarm_reason_t::command_external;
		break;
	}

	bool status_changed = false;

	switch (action_request.action) {
	case action_request_s::ACTION_DISARM:
		status_changed = (disarm(arm_disarm_reason) == TRANSITION_CHANGED);
		break;

	case action_request_s::ACTION_ARM:
		status_changed = (arm(arm_disarm_reason) == TRANSITION_CHANGED);
		break;

	case action_request_s::ACTION_TOGGLE_ARMING:
		if (is_armed()) {
			status_changed = (disarm(arm_disarm_reason) == TRANSITION_CHANGED);
		} else {
			status_changed = (arm(arm_disarm_reason) == TRANSITION_CHANGED);
		}
		break;

	case action_request_s::ACTION_UNKILL:
		status_changed = handle_kill_switch(false);
		break;

	case action_request_s::ACTION_KILL:
		status_changed = handle_kill_switch(true);
		break;

	case action_request_s::ACTION_SWITCH_MODE:
		// Publish mode change request to mode_manager
		{
			mode_change_request_s request{};
			request.timestamp = hrt_absolute_time();
			request.requested_mode = action_request.mode;
			request.source = mode_change_request_s::SOURCE_USER;
			request.allow_fallback = false;
			request.force = false;
			_mode_change_request_pub.publish(request);
		}
		break;

	default:
		break;
	}

	return status_changed;
}

void ArmingHandler::publish_actuator_armed()
{
	_actuator_armed.timestamp = hrt_absolute_time();
	_actuator_armed_pub.publish(_actuator_armed);
}
