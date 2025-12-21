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
 * @file failsafe_handler.hpp
 *
 * Failsafe handling logic extracted from system_manager
 */

#pragma once

#include "failsafe/failsafe.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionData.hpp>
#include <uORB/topics/failsafe_mode_request.h>
#include <uORB/topics/flight_termination_request.h>
#include <uORB/topics/mode_change_request.h>
#include <uORB/topics/mode_status.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/vehicle_status.h>

class SystemManager;  // Forward declaration
class HealthAndArmingChecks;  // Forward declaration

/**
 * @brief Handles all failsafe-related logic
 *
 * This class encapsulates:
 * - Failsafe state machine evaluation
 * - User notification callbacks
 * - Publishing failsafe mode requests to mode_manager
 * - Handling critical failsafe actions (disarm, terminate)
 * - Tracking failsafe defer state
 */
class FailsafeHandler
{
public:
	explicit FailsafeHandler(SystemManager *parent);
	~FailsafeHandler() = default;

	/**
	 * @brief Initialize the handler
	 * @return true on success
	 */
	bool init();

	/**
	 * @brief Evaluate failsafe conditions and request mode changes
	 *
	 * This function is responsible for:
	 * 1. Running the failsafe state machine with current conditions
	 * 2. Publishing failsafe mode requests to mode_manager when needed
	 * 3. Handling critical actions (disarm, terminate) that bypass mode_manager
	 * 4. Updating failsafe status flags in vehicle_status
	 *
	 * @param vehicle_status Current vehicle status (will be updated with failsafe flags)
	 * @param failsafe_flags Current failsafe condition flags
	 * @param config_overrides Config overrides for failsafe deferring
	 * @param user_intended_mode User's intended operation mode
	 * @param had_mode_change Whether a mode change was requested
	 * @param user_override_request User override request flag
	 * @return true if failsafe state changed
	 */
	bool update(vehicle_status_s &vehicle_status,
		    const failsafe_flags_s &failsafe_flags,
		    const config_overrides_s &config_overrides,
		    uint8_t user_intended_mode,
		    bool had_mode_change,
		    bool user_override_request);

	/**
	 * @brief Handle flight termination requests
	 */
	void handle_flight_termination_requests();

	/**
	 * @brief Check if currently in failsafe
	 */
	bool inFailsafe() const { return _failsafe.inFailsafe(); }

	/**
	 * @brief Check if user takeover is active
	 */
	bool userTakeoverActive() const { return _failsafe.userTakeoverActive(); }

	/**
	 * @brief Get the current failsafe action
	 */
	FailsafeBase::Action selectedAction() const { return _failsafe.selectedAction(); }

	/**
	 * @brief Get reference to failsafe instance (for callbacks)
	 */
	FailsafeBase &failsafe() { return _failsafe; }

	/**
	 * @brief Set reference to health and arming checks for notifications
	 */
	void setHealthAndArmingChecks(HealthAndArmingChecks *checks) { _health_and_arming_checks = checks; }

	/**
	 * @brief Notify user about failsafe (trampoline for callback)
	 */
	static void on_failsafe_notify_user_trampoline(void *arg);

	/**
	 * @brief Handle failsafe user notification
	 */
	void on_failsafe_notify_user();

private:
	/**
	 * @brief Publish failsafe mode request to mode_manager
	 */
	void publish_failsafe_request(uint8_t requested_mode);

	/**
	 * @brief Handle critical failsafe actions that bypass mode_manager
	 */
	void handle_critical_actions(vehicle_status_s &vehicle_status);

	/**
	 * @brief Update failsafe defer state
	 */
	void update_defer_state(vehicle_status_s &vehicle_status,
				const config_overrides_s &config_overrides);

	/**
	 * @brief Publish failsafe mode request with full details
	 */
	void publish_failsafe_mode_request(uint8_t requested_mode,
					   FailsafeBase::Action action,
					   uint8_t severity,
					   uint8_t source);

	SystemManager *_parent{nullptr};
	HealthAndArmingChecks *_health_and_arming_checks{nullptr};

	// Failsafe state machine
	Failsafe _failsafe_instance;
	FailsafeBase &_failsafe{_failsafe_instance};

	// Subscriptions
	uORB::Subscription _flight_termination_request_sub{ORB_ID(flight_termination_request)};
	uORB::SubscriptionData<mission_result_s> _mission_result_sub{ORB_ID(mission_result)};

	// Publications
	uORB::Publication<failsafe_mode_request_s> _failsafe_mode_request_pub{ORB_ID(failsafe_mode_request)};
};
