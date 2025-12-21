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
 * @file calibration_handler.hpp
 *
 * Calibration handling logic extracted from system_manager
 */

#pragma once

#include "safety.hpp"
#include "worker_thread.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/calibration_request.h>
#include <uORB/topics/storage_request.h>
#include <uORB/topics/reboot_request.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_test_request.h>
#include <uORB/Publication.hpp>

class SystemManager;  // Forward declaration

/**
 * @brief Handles all calibration and storage-related operations
 *
 * This class encapsulates:
 * - Sensor calibration requests (gyro, accel, mag, baro, etc.)
 * - RC calibration
 * - ESC calibration
 * - Storage operations (param save/load/reset)
 * - Reboot requests
 * - Actuator test requests
 */
class CalibrationHandler
{
public:
	CalibrationHandler(SystemManager *parent,
			   Safety &safety,
			   WorkerThread &worker_thread);
	~CalibrationHandler() = default;

	/**
	 * @brief Handle calibration requests from command_processor
	 * @return true if calibration started
	 */
	bool handle_calibration_requests();

	/**
	 * @brief Handle storage requests (param operations)
	 */
	void handle_storage_requests();

	/**
	 * @brief Handle reboot requests
	 */
	void handle_reboot_requests();

	/**
	 * @brief Handle actuator test requests
	 */
	void handle_actuator_test_requests();

	/**
	 * @brief Check if calibration is in progress
	 */
	bool is_calibration_enabled() const { return _calibration_enabled; }

	/**
	 * @brief Check if RC calibration is in progress
	 */
	bool is_rc_calibration_in_progress() const { return _rc_calibration_in_progress; }

	/**
	 * @brief Check if ESC calibration is in progress
	 */
	bool is_esc_calibration_in_progress() const { return _in_esc_calibration_mode; }

	/**
	 * @brief Set ESC calibration mode flag
	 */
	void set_esc_calibration_mode(bool in_esc_calib) { _in_esc_calibration_mode = in_esc_calib; }

	/**
	 * @brief Set armed state (for rejection of calibration)
	 */
	void set_armed(bool armed) { _is_armed = armed; }

	/**
	 * @brief Set motor test enable parameter
	 */
	void set_motor_test_enabled(bool enabled) { _motor_test_enabled = enabled; }

	/**
	 * @brief Check worker thread status
	 */
	void check_worker_thread();

	/**
	 * @brief Get calibration enabled state for vehicle_status
	 */
	bool get_calibration_enabled() const { return _calibration_enabled; }

	/**
	 * @brief Get RC calibration in progress state for vehicle_status
	 */
	bool get_rc_calibration_in_progress() const { return _rc_calibration_in_progress; }

private:
	SystemManager *_parent{nullptr};
	Safety &_safety;
	WorkerThread &_worker_thread;

	bool _calibration_enabled{false};
	bool _rc_calibration_in_progress{false};
	bool _in_esc_calibration_mode{false};
	bool _is_armed{false};
	bool _motor_test_enabled{false};

	// Subscriptions
	uORB::Subscription _calibration_request_sub{ORB_ID(calibration_request)};
	uORB::Subscription _storage_request_sub{ORB_ID(storage_request)};
	uORB::Subscription _reboot_request_sub{ORB_ID(reboot_request)};
	uORB::Subscription _actuator_test_request_sub{ORB_ID(actuator_test_request)};

	// Publications
	uORB::Publication<actuator_test_s> _actuator_test_pub{ORB_ID(actuator_test)};

	orb_advert_t _mavlink_log_pub{nullptr};
};
