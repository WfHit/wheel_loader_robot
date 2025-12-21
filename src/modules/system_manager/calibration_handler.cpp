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
 * @file calibration_handler.cpp
 *
 * Calibration handling logic extracted from system_manager
 */

#include "calibration_handler.hpp"
#include "calibration/esc_calibration.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/shutdown.h>
#include <systemlib/mavlink_log.h>

CalibrationHandler::CalibrationHandler(SystemManager *parent,
				       Safety &safety,
				       WorkerThread &worker_thread) :
	_parent(parent),
	_safety(safety),
	_worker_thread(worker_thread)
{
}

bool CalibrationHandler::handle_calibration_requests()
{
	calibration_request_s req;

	if (_calibration_request_sub.update(&req)) {
		if (_is_armed || _worker_thread.isBusy()) {
			// Reject if armed or busy
			return false;
		}

		_calibration_enabled = true;

		switch (req.calibration_type) {
		case calibration_request_s::CALIBRATION_TYPE_GYRO:
			_worker_thread.startTask(WorkerThread::Request::GyroCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_MAG:
			_worker_thread.startTask(WorkerThread::Request::MagCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_BARO:
			_worker_thread.startTask(WorkerThread::Request::BaroCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_RC_START:
			_rc_calibration_in_progress = true;
			mavlink_log_info(&_mavlink_log_pub, "Calibration: Disabling RC input\t");
			events::send(events::ID("commander_calib_rc_off"), events::Log::Info,
				     "Calibration: Disabling RC input");
			break;

		case calibration_request_s::CALIBRATION_TYPE_RC_TRIM:
			_worker_thread.startTask(WorkerThread::Request::RCTrimCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_RC_END:
			if (_rc_calibration_in_progress) {
				_rc_calibration_in_progress = false;
				mavlink_log_info(&_mavlink_log_pub, "Calibration: Restoring RC input\t");
				events::send(events::ID("commander_calib_rc_on"), events::Log::Info,
					     "Calibration: Restoring RC input");
			}
			break;

		case calibration_request_s::CALIBRATION_TYPE_ACCEL:
			_worker_thread.startTask(WorkerThread::Request::AccelCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_LEVEL:
			_worker_thread.startTask(WorkerThread::Request::LevelCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_ACCEL_QUICK:
			_worker_thread.startTask(WorkerThread::Request::AccelCalibrationQuick);
			break;

		case calibration_request_s::CALIBRATION_TYPE_AIRSPEED:
			_worker_thread.startTask(WorkerThread::Request::AirspeedCalibration);
			break;

		case calibration_request_s::CALIBRATION_TYPE_ESC:
			if (check_battery_disconnected(&_mavlink_log_pub)) {
				if (_safety.isButtonAvailable() && !_safety.isSafetyOff()) {
					mavlink_log_critical(&_mavlink_log_pub, "ESC calibration denied! Press safety button first\t");
					events::send(events::ID("commander_esc_calibration_denied"), events::Log::Critical,
						     "ESCs calibration denied");

				} else {
					_in_esc_calibration_mode = true;
					_worker_thread.startTask(WorkerThread::Request::ESCCalibration);
				}
			}
			break;

		case calibration_request_s::CALIBRATION_TYPE_MAG_QUICK:
			_worker_thread.setMagQuickData(req.mag_heading_rad, req.latitude, req.longitude);
			_worker_thread.startTask(WorkerThread::Request::MagCalibrationQuick);
			break;

		case calibration_request_s::CALIBRATION_TYPE_TEMPERATURE:
			// Temperature calibration is handled in events module
			break;

		default:
			_calibration_enabled = false;
			return false;
		}

		return true;
	}

	return false;
}

void CalibrationHandler::handle_storage_requests()
{
	storage_request_s req;

	if (_storage_request_sub.update(&req)) {
		if (_is_armed || _worker_thread.isBusy()) {
			return;  // Reject if armed or busy
		}

		switch (req.operation) {
		case storage_request_s::STORAGE_OP_LOAD_DEFAULT:
			_worker_thread.startTask(WorkerThread::Request::ParamLoadDefault);
			break;

		case storage_request_s::STORAGE_OP_SAVE_DEFAULT:
			_worker_thread.startTask(WorkerThread::Request::ParamSaveDefault);
			break;

		case storage_request_s::STORAGE_OP_RESET_ALL_CONFIG:
			_worker_thread.startTask(WorkerThread::Request::ParamResetAllConfig);
			break;

		case storage_request_s::STORAGE_OP_RESET_SENSOR_FACTORY:
			_worker_thread.startTask(WorkerThread::Request::ParamResetSensorFactory);
			break;

		case storage_request_s::STORAGE_OP_RESET_ALL:
			_worker_thread.startTask(WorkerThread::Request::ParamResetAll);
			break;

		default:
			break;
		}
	}
}

void CalibrationHandler::handle_reboot_requests()
{
	reboot_request_s req;

	if (_reboot_request_sub.update(&req)) {
		if (_is_armed) {
			return;  // Cannot reboot while armed
		}

		switch (req.reboot_type) {
		case reboot_request_s::REBOOT_TYPE_NONE:
			// Do nothing
			break;

#if defined(CONFIG_BOARDCTL_RESET)

		case reboot_request_s::REBOOT_TYPE_REBOOT:
			if (px4_reboot_request(REBOOT_REQUEST, 400_ms) == 0) {
				while (1) { px4_usleep(1); }
			}
			break;

		case reboot_request_s::REBOOT_TYPE_BOOTLOADER:
			if (px4_reboot_request(REBOOT_TO_BOOTLOADER, 400_ms) == 0) {
				while (1) { px4_usleep(1); }
			}
			break;
#endif // CONFIG_BOARDCTL_RESET

#if defined(BOARD_HAS_POWER_CONTROL)

		case reboot_request_s::REBOOT_TYPE_SHUTDOWN:
			if (px4_shutdown_request(400_ms) == 0) {
				while (1) { px4_usleep(1); }
			}
			break;
#endif // BOARD_HAS_POWER_CONTROL

		default:
			break;
		}
	}
}

void CalibrationHandler::handle_actuator_test_requests()
{
	actuator_test_request_s req;

	if (_actuator_test_request_sub.update(&req)) {
		if (_is_armed || (_safety.isButtonAvailable() && !_safety.isSafetyOff())) {
			return;
		}

		if (!_motor_test_enabled) {
			return;
		}

		actuator_test_s actuator_test{};
		actuator_test.timestamp = hrt_absolute_time();
		actuator_test.function = req.function;

		if (actuator_test.function < 1000) {
			// Note: This mapping logic may need adjustment based on actual vehicle type
			return;
		}

		actuator_test.value = req.value;
		actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;

		if (!PX4_ISFINITE(actuator_test.value)) {
			actuator_test.action = actuator_test_s::ACTION_RELEASE_CONTROL;

		} else if (actuator_test.value < -1.f || actuator_test.value > 1.f) {
			return;
		}

		actuator_test.timeout_ms = static_cast<uint32_t>(req.timeout_ms);

		_actuator_test_pub.publish(actuator_test);
	}
}

void CalibrationHandler::check_worker_thread()
{
	WorkerThread::Result result = _worker_thread.getResult();

	if (result != WorkerThread::Result::Busy) {
		// Worker finished - update calibration state
		_calibration_enabled = false;

		if (_in_esc_calibration_mode) {
			_in_esc_calibration_mode = false;
		}
	}
}
