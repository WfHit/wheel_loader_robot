/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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

#include "UserModeIntention.hpp"

namespace mode_manager
{

UserModeIntention::UserModeIntention(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool UserModeIntention::update()
{
	mode_change_request_s request;

	if (_mode_change_request_sub.update(&request)) {
		_ever_had_mode_change = true;

		// Update subscriptions
		_vehicle_status_sub.update();
		_vehicle_type_config_sub.update();

		const uint8_t requested_mode = request.requested_mode;
		const bool is_armed = _vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		const bool force = request.force;
		const bool allow_fallback = request.allow_fallback;

		// Always allow mode change when disarmed or forced
		bool allow_change = !is_armed || force;

		if (!allow_change) {
			// Check if mode is available for this vehicle type
			allow_change = isModeAvailable(requested_mode);

			// Check fallback if position control not available
			if (!allow_change && allow_fallback && _param_com_posctl_navl.get() == 0) {
				if (requested_mode == vehicle_status_s::OPERATION_MODE_POSCTL) {
					allow_change = isModeAvailable(vehicle_status_s::OPERATION_MODE_ALTCTL);
				}
			}
		}

		// Never allow to change out of termination state
		if (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_TERMINATION) {
			allow_change = false;
		}

		uint8_t result_mode = _user_intended_mode;
		uint8_t result = mode_change_result_s::RESULT_DENIED;

		if (allow_change) {
			_had_mode_change = true;
			_user_intended_mode = requested_mode;
			result_mode = requested_mode;
			result = mode_change_result_s::RESULT_ACCEPTED;

			// Store mode for after disarm (unless it prevents arming)
			if (requested_mode != vehicle_status_s::OPERATION_MODE_TERMINATION) {
				_mode_after_disarm = requested_mode;
			}

			PX4_DEBUG("Mode changed to %d", requested_mode);

		} else {
			result = mode_change_result_s::RESULT_TEMPORARILY_REJECTED;
			PX4_DEBUG("Mode change to %d rejected", requested_mode);
		}

		// Publish result if this was from a command (for ACK routing)
		if (request.cmd_command != 0) {
			publishResult(request, result_mode, result);
		}

		return allow_change;
	}

	return false;
}

bool UserModeIntention::change(uint8_t mode, ModeChangeSource source, bool force)
{
	_ever_had_mode_change = true;

	_vehicle_status_sub.update();

	const bool is_armed = _vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	bool allow_change = !is_armed || force;

	if (!allow_change) {
		allow_change = isModeAvailable(mode);
	}

	// Never allow to change out of termination state
	if (_vehicle_status_sub.get().operation_mode == vehicle_status_s::OPERATION_MODE_TERMINATION) {
		allow_change = false;
	}

	if (allow_change) {
		_had_mode_change = true;
		_user_intended_mode = mode;

		if (mode != vehicle_status_s::OPERATION_MODE_TERMINATION) {
			_mode_after_disarm = mode;
		}
	}

	return allow_change;
}

void UserModeIntention::onDisarm()
{
	_user_intended_mode = _mode_after_disarm;
}

bool UserModeIntention::isModeAvailable(uint8_t mode) const
{
	const vehicle_type_config_s &vtc = _vehicle_type_config_sub.get();

	if (!vtc.config_valid) {
		// If config is not valid, allow all modes (fallback behavior)
		return true;
	}

	// Check if the mode bit is set in the available_modes_mask
	if (mode < 32) {
		return (vtc.available_modes_mask & (1u << mode)) != 0;
	}

	return false;
}

void UserModeIntention::publishResult(const mode_change_request_s &request, uint8_t result_mode, uint8_t result)
{
	mode_change_result_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.requested_mode = request.requested_mode;
	msg.result_mode = result_mode;
	msg.result = result;
	msg.cmd_source_system = request.cmd_source_system;
	msg.cmd_source_component = request.cmd_source_component;
	msg.cmd_command = request.cmd_command;

	_mode_change_result_pub.publish(msg);
}

} // namespace mode_manager
