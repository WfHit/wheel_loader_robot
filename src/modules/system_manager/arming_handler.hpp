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
 * @file arming_handler.hpp
 *
 * Arming/disarming logic extracted from system_manager
 */

#pragma once

#include "health_and_arming_checks/health_and_arming_checks.hpp"
#include "safety.hpp"
#include "home_position.hpp"

#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/action_request.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/arming_request.h>
#include <uORB/topics/mode_change_request.h>
#include <uORB/topics/prearm_check_request.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using arm_disarm_reason_t = events::px4::enums::arm_disarm_reason_t;

typedef enum {
	TRANSITION_DENIED = -1,
	TRANSITION_NOT_CHANGED = 0,
	TRANSITION_CHANGED
} arming_transition_result_t;

class SystemManager;  // Forward declaration

/**
 * @brief Handles all arming/disarming logic
 *
 * This class encapsulates:
 * - Arm/disarm state transitions
 * - Arming request handling
 * - Auto-disarm logic
 * - Prearm check handling
 * - Kill switch handling
 */
class ArmingHandler
{
public:
	explicit ArmingHandler(SystemManager *parent,
			       HealthAndArmingChecks &health_checks,
			       Safety &safety,
			       HomePosition &home_position);
	~ArmingHandler() = default;

	/**
	 * @brief Attempt to arm the vehicle
	 * @param calling_reason The reason for arming
	 * @param run_prearm_checks Whether to run prearm checks
	 * @return Transition result
	 */
	arming_transition_result_t arm(arm_disarm_reason_t calling_reason, bool run_prearm_checks = true);

	/**
	 * @brief Attempt to disarm the vehicle
	 * @param calling_reason The reason for disarming
	 * @param forced Whether to force disarm even if not landed
	 * @return Transition result
	 */
	arming_transition_result_t disarm(arm_disarm_reason_t calling_reason, bool forced = false);

	/**
	 * @brief Handle arming requests from command_processor
	 * @return true if status changed
	 */
	bool handle_arming_requests();

	/**
	 * @brief Handle action requests (RC-triggered actions like arm/disarm/kill)
	 * @param publish_mode_change Callback to publish mode change requests
	 * @return true if status changed
	 */
	bool handle_action_requests();

	/**
	 * @brief Handle prearm check requests
	 */
	void handle_prearm_check_requests();

	/**
	 * @brief Handle auto-disarm based on landing or kill switch
	 */
	void handle_auto_disarm();

	/**
	 * @brief Handle kill switch engage/disengage
	 * @param kill true to engage kill switch
	 * @return true if state changed
	 */
	bool handle_kill_switch(bool kill);

	/**
	 * @brief Check if vehicle is armed
	 */
	bool is_armed() const;

	/**
	 * @brief Get prearm state
	 */
	bool get_prearm_state() const;

	/**
	 * @brief Get actuator armed state
	 */
	const actuator_armed_s &actuator_armed() const { return _actuator_armed; }

	/**
	 * @brief Get mutable reference to actuator armed (for ESC calibration flag)
	 */
	actuator_armed_s &actuator_armed_mut() { return _actuator_armed; }

	/**
	 * @brief Update vehicle status reference
	 */
	void set_vehicle_status(vehicle_status_s *status) { _vehicle_status = status; }

	/**
	 * @brief Set vehicle control mode reference
	 */
	void set_vehicle_control_mode(const vehicle_control_mode_s *mode) { _vehicle_control_mode = mode; }

	/**
	 * @brief Set vehicle land detected reference
	 */
	void set_vehicle_land_detected(const vehicle_land_detected_s *land) { _vehicle_land_detected = land; }

	/**
	 * @brief Set current operation mode for arming checks
	 */
	void set_current_mode(uint8_t mode) { _current_operation_mode = mode; }

	/**
	 * @brief Set failsafe flags reference
	 */
	void set_failsafe_flags(const failsafe_flags_s *flags) { _failsafe_flags = flags; }

	/**
	 * @brief Set throttle state for arming checks
	 */
	void set_throttle_state(bool above_center, bool is_low) {
		_is_throttle_above_center = above_center;
		_is_throttle_low = is_low;
	}

	/**
	 * @brief Set parameters needed for auto-disarm
	 */
	void set_disarm_params(float disarm_land_s, float disarm_prflt_s, float kill_disarm_s,
			       bool force_safety, bool disarm_man, bool home_en) {
		_param_disarm_land = disarm_land_s;
		_param_disarm_prflt = disarm_prflt_s;
		_auto_disarm_killed.set_hysteresis_time_from(false, static_cast<hrt_abstime>(kill_disarm_s * 1e6f));
		_param_force_safety = force_safety;
		_param_disarm_man = disarm_man;
		_param_home_en = home_en;
	}

	/**
	 * @brief Set have taken off flag
	 */
	void set_have_taken_off(bool taken_off) { _have_taken_off_since_arming = taken_off; }

	/**
	 * @brief Get have taken off flag
	 */
	bool have_taken_off() const { return _have_taken_off_since_arming; }

	/**
	 * @brief Publish actuator armed state
	 */
	void publish_actuator_armed();

	/**
	 * @brief Get last disarmed timestamp
	 */
	hrt_abstime last_disarmed_timestamp() const { return _last_disarmed_timestamp; }

	/**
	 * @brief Check if action request subscription has updates
	 */
	bool action_request_updated() const { return _action_request_sub.updated(); }

	/**
	 * @brief Set RC calibration in progress flag (to ignore RC actions during calibration)
	 */
	void set_rc_calibration_in_progress(bool in_progress) { _rc_calibration_in_progress = in_progress; }

private:
	SystemManager *_parent{nullptr};
	HealthAndArmingChecks &_health_checks;
	Safety &_safety;
	HomePosition &_home_position;

	// State references (owned by SystemManager)
	vehicle_status_s *_vehicle_status{nullptr};
	const vehicle_control_mode_s *_vehicle_control_mode{nullptr};
	const vehicle_land_detected_s *_vehicle_land_detected{nullptr};
	const failsafe_flags_s *_failsafe_flags{nullptr};

	uint8_t _current_operation_mode{0};
	bool _is_throttle_above_center{false};
	bool _is_throttle_low{false};
	bool _have_taken_off_since_arming{false};
	bool _rc_calibration_in_progress{false};

	// Actuator state
	actuator_armed_s _actuator_armed{};

	// Auto-disarm hysteresis
	systemlib::Hysteresis _auto_disarm_landed{false};
	systemlib::Hysteresis _auto_disarm_killed{false};

	// Parameters
	float _param_disarm_land{0.f};
	float _param_disarm_prflt{0.f};
	bool _param_force_safety{false};
	bool _param_disarm_man{false};
	bool _param_home_en{true};

	hrt_abstime _last_disarmed_timestamp{0};

	// Subscriptions
	uORB::Subscription _arming_request_sub{ORB_ID(arming_request)};
	uORB::Subscription _action_request_sub{ORB_ID(action_request)};
	uORB::Subscription _prearm_check_request_sub{ORB_ID(prearm_check_request)};

	// Publications
	uORB::Publication<actuator_armed_s> _actuator_armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<mode_change_request_s> _mode_change_request_pub{ORB_ID(mode_change_request)};

	orb_advert_t _mavlink_log_pub{nullptr};
};
