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
 * @file failsafe_handler.cpp
 *
 * Failsafe handling logic extracted from system_manager
 */

#include "failsafe_handler.hpp"
#include "system_manager.hpp"
#include "HealthAndArmingChecks/HealthAndArmingChecks.hpp"

#include <drivers/drv_hrt.h>

FailsafeHandler::FailsafeHandler(SystemManager *parent) :
	_parent(parent),
	_failsafe_instance(parent)
{
}

bool FailsafeHandler::init()
{
	// Register ourselves for failsafe user notification callback
	_failsafe.setOnNotifyUserCallback(&FailsafeHandler::on_failsafe_notify_user_trampoline, this);
	return true;
}

bool FailsafeHandler::update(vehicle_status_s &vehicle_status,
			     const failsafe_flags_s &failsafe_flags,
			     const config_overrides_s &config_overrides,
			     uint8_t user_intended_mode,
			     bool had_mode_change,
			     bool user_override_request)
{
	const uint8_t prev_failsafe_defer_state = vehicle_status.failsafe_defer_state;
	const FailsafeBase::Action prev_failsafe_action = _failsafe.selectedAction();

	// Build current state for failsafe evaluation
	FailsafeBase::State state{};
	state.armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	state.vtol_in_transition_mode = vehicle_status.in_transition_mode;
	state.mission_finished = _mission_result_sub.get().finished;
	state.user_intended_mode = user_intended_mode;
	state.vehicle_type = vehicle_status.vehicle_type;

	// Run failsafe state machine - evaluates all failsafe_flags
	uint8_t failsafe_suggested_mode =
		_failsafe.update(
			hrt_absolute_time(),
			state,
			had_mode_change,
			user_override_request,
			failsafe_flags);

	// If failsafe wants to change mode, publish request to mode_manager
	if (state.user_intended_mode != failsafe_suggested_mode) {
		publish_failsafe_request(failsafe_suggested_mode);
	}

	// Handle immediate actions (disarm/terminate) that bypass mode_manager
	handle_critical_actions(vehicle_status);

	// Update failsafe status flags
	vehicle_status.failsafe = _failsafe.inFailsafe();
	vehicle_status.failsafe_and_user_took_over = _failsafe.userTakeoverActive();

	// Update failsafe deferring state
	update_defer_state(vehicle_status, config_overrides);

	return prev_failsafe_action != _failsafe.selectedAction() ||
	       prev_failsafe_defer_state != vehicle_status.failsafe_defer_state;
}

void FailsafeHandler::handle_flight_termination_requests()
{
	flight_termination_request_s req;

	if (_flight_termination_request_sub.update(&req)) {
		if (req.terminate) {
			// Force mode change to termination - use failsafe request with critical severity
			publish_failsafe_mode_request(mode_status_s::OPERATION_MODE_TERMINATION,
						      FailsafeBase::Action::Terminate,
						      failsafe_mode_request_s::SEVERITY_CRITICAL,
						      failsafe_mode_request_s::SOURCE_OTHER);
		}
	}
}

void FailsafeHandler::publish_failsafe_request(uint8_t requested_mode)
{
	uint8_t severity = failsafe_mode_request_s::SEVERITY_HIGH;

	// Map failsafe action to severity level
	switch (_failsafe.selectedAction()) {
	case FailsafeBase::Action::Warn:
		severity = failsafe_mode_request_s::SEVERITY_LOW;
		break;

	case FailsafeBase::Action::Hold:
	case FailsafeBase::Action::RTL:
	case FailsafeBase::Action::Land:
		severity = failsafe_mode_request_s::SEVERITY_MEDIUM;
		break;

	case FailsafeBase::Action::Descend:
		severity = failsafe_mode_request_s::SEVERITY_HIGH;
		break;

	case FailsafeBase::Action::Terminate:
	case FailsafeBase::Action::Disarm:
		severity = failsafe_mode_request_s::SEVERITY_CRITICAL;
		break;

	default:
		break;
	}

	publish_failsafe_mode_request(requested_mode, _failsafe.selectedAction(), severity,
				      failsafe_mode_request_s::SOURCE_OTHER);
}

void FailsafeHandler::handle_critical_actions(vehicle_status_s &vehicle_status)
{
	// Critical actions that must be handled immediately by system_manager
	// (not delegated to mode_manager)
	switch (_failsafe.selectedAction()) {
	case FailsafeBase::Action::Disarm:
		// Disarm is handled by parent SystemManager via callback
		if (_parent) {
			_parent->disarm(arm_disarm_reason_t::failsafe, true);
		}

		break;

	case FailsafeBase::Action::Terminate:
		// Force termination mode directly
		vehicle_status.operation_mode = vehicle_status_s::OPERATION_MODE_TERMINATION;
		break;

	default:
		break;
	}
}

void FailsafeHandler::update_defer_state(vehicle_status_s &vehicle_status,
		const config_overrides_s &config_overrides)
{
	// Apply failsafe deferring & update state
	_failsafe.deferFailsafes(config_overrides.defer_failsafes, config_overrides.defer_failsafes_timeout_s);

	if (_failsafe.failsafeDeferred()) {
		vehicle_status.failsafe_defer_state = vehicle_status_s::FAILSAFE_DEFER_STATE_WOULD_FAILSAFE;

	} else if (_failsafe.getDeferFailsafes()) {
		vehicle_status.failsafe_defer_state = vehicle_status_s::FAILSAFE_DEFER_STATE_ENABLED;

	} else {
		vehicle_status.failsafe_defer_state = vehicle_status_s::FAILSAFE_DEFER_STATE_DISABLED;
	}
}

void FailsafeHandler::publish_failsafe_mode_request(uint8_t requested_mode,
		FailsafeBase::Action action,
		uint8_t severity,
		uint8_t source)
{
	failsafe_mode_request_s request{};
	request.timestamp = hrt_absolute_time();
	request.requested_mode = requested_mode;
	request.failsafe_action = static_cast<uint8_t>(action);
	request.force = (severity >= failsafe_mode_request_s::SEVERITY_HIGH);
	request.allow_user_override = (severity <= failsafe_mode_request_s::SEVERITY_MEDIUM);
	request.severity = severity;
	request.source = source;

	_failsafe_mode_request_pub.publish(request);
}

void FailsafeHandler::on_failsafe_notify_user_trampoline(void *arg)
{
	FailsafeHandler *handler = static_cast<FailsafeHandler *>(arg);
	handler->on_failsafe_notify_user();
}

void FailsafeHandler::on_failsafe_notify_user()
{
	// If we are about to inform about a failsafe, we need to ensure any pending health report is sent out first,
	// as the failsafe message might reference that. This is only needed in case the report is currently rate-limited,
	// i.e. it had a recent previous change already.
	if (_health_and_arming_checks) {
		_health_and_arming_checks->reportIfUnreportedDifferences();
	}
}
