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

#include "safety_manager.hpp"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

SafetyManager::SafetyManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

SafetyManager::~SafetyManager()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool SafetyManager::init()
{
	ScheduleOnInterval(SAFETY_CHECK_INTERVAL_US);
	return true;
}

void SafetyManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update all subscriptions
	update_subscriptions();

	// Monitor safety conditions
	monitor_safety();

	// Assess overall safety level
	assess_safety_level();

	// Publish safety status
	publish_safety_status();

	// Update counters
	_safety_check_counter++;
	_last_safety_check = hrt_absolute_time();

	perf_end(_loop_perf);
}

void SafetyManager::update_subscriptions()
{
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_drivetrain_status_sub.update(&_drivetrain_status);
	_steering_status_sub.update(&_steering_status);
	_hbridge_status_sub.update(&_hbridge_status);
	_bucket_status_sub.update(&_bucket_status);
}

void SafetyManager::monitor_safety()
{
	_speed_safe = is_speed_safe();
	_steering_safe = is_steering_safe();
	_stability_safe = is_stability_safe();
	_communication_safe = is_communication_safe();
	_hbridge_safe = is_hbridge_safe();
	_bucket_safe = is_bucket_safe();
}

void SafetyManager::assess_safety_level()
{
	_previous_safety_level = _current_safety_level;

	// Start with normal level
	_current_safety_level = SafetyLevel::NORMAL;

	// Check for warning conditions
	if (!_speed_safe || !_steering_safe || !_bucket_safe) {
		_current_safety_level = SafetyLevel::WARNING;
	}

	// Check for critical conditions
	if (!_stability_safe || !_hbridge_safe) {
		_current_safety_level = SafetyLevel::CRITICAL;
	}

	// Check for emergency conditions
	if (!_communication_safe) {
		_current_safety_level = SafetyLevel::EMERGENCY;
	}

	// Count safety violations
	if (_current_safety_level > SafetyLevel::NORMAL) {
		if (_previous_safety_level == SafetyLevel::NORMAL) {
			_safety_violations++;
			_last_fault_time = hrt_absolute_time();
		}
	}
}

void SafetyManager::publish_safety_status()
{
	system_safety_s safety_status{};
	safety_status.timestamp = hrt_absolute_time();

	// Overall safety status
	safety_status.safety_level = static_cast<uint8_t>(_current_safety_level);
	safety_status.safety_mode = system_safety_s::SAFETY_MODE_NORMAL;
	safety_status.overall_risk_factor = static_cast<float>(_current_safety_level) / 4.0f;
	safety_status.emergency_active = (_current_safety_level == SafetyLevel::EMERGENCY);
	safety_status.safety_override_active = false;

	// Fault status
	safety_status.active_faults = 0;
	if (!_speed_safe) {
		safety_status.active_faults |= (1 << 0); // Speed fault
	}
	if (!_steering_safe) {
		safety_status.active_faults |= (1 << 1); // Steering fault
	}
	if (!_stability_safe) {
		safety_status.active_faults |= (1 << 2); // Stability fault
	}
	if (!_communication_safe) {
		safety_status.active_faults |= (1 << 3); // Communication fault
	}
	if (!_hbridge_safe) {
		safety_status.active_faults |= (1 << 4); // H-bridge fault
	}
	if (!_bucket_safe) {
		safety_status.active_faults |= (1 << 5); // Bucket fault
	}

	safety_status.fault_history = safety_status.active_faults; // Simplified
	safety_status.last_fault_time = _last_fault_time;
	safety_status.safety_violation_count = _safety_violations;

	// Safety permits
	safety_status.motion_permitted = (_current_safety_level != SafetyLevel::EMERGENCY);
	safety_status.steering_permitted = _steering_safe;
	safety_status.autonomous_permitted = (_current_safety_level == SafetyLevel::NORMAL);
	safety_status.electric_actuator_permitted = (_current_safety_level != SafetyLevel::EMERGENCY);
	safety_status.boom_operation_permitted = (_current_safety_level != SafetyLevel::EMERGENCY);
	safety_status.bucket_operation_permitted = (_current_safety_level != SafetyLevel::EMERGENCY);
	safety_status.articulation_permitted = (_current_safety_level != SafetyLevel::EMERGENCY);
	safety_status.engine_start_permitted = (_current_safety_level != SafetyLevel::EMERGENCY);
	safety_status.emergency_override_active = false;

	// System monitoring status
	safety_status.chassis_safe = _speed_safe && _steering_safe;
	safety_status.hydraulic_safe = _hbridge_safe && _bucket_safe; // Electric actuators mapped to hydraulic_safe
	safety_status.sensor_safe = _stability_safe;
	safety_status.communication_safe = _communication_safe;
	safety_status.power_safe = _hbridge_safe; // H-bridge represents power system health
	safety_status.thermal_safe = true; // Temperature monitoring from bucket status

	// Emergency response
	safety_status.emergency_stop_commanded = (_current_safety_level == SafetyLevel::EMERGENCY);
	safety_status.controlled_stop_active = (_current_safety_level >= SafetyLevel::CRITICAL);
	safety_status.emergency_shutdown_active = false;
	safety_status.backup_systems_active = false;

	// Performance metrics
	safety_status.safety_check_frequency_hz = 1.0e6f / static_cast<float>(SAFETY_CHECK_INTERVAL_US);
	safety_status.intervention_response_time_ms = 0.0f; // Not measured yet
	safety_status.safety_system_availability = 1.0f; // Assume 100% for now
	safety_status.total_safety_checks = _safety_check_counter;
	safety_status.safety_interventions = _safety_violations;

	_system_safety_pub.publish(safety_status);
}

bool SafetyManager::is_speed_safe() const
{
	const float max_speed = _param_max_speed.get();
	const float current_speed = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
					  _vehicle_local_position.vy * _vehicle_local_position.vy);

	return (current_speed <= max_speed);
}

bool SafetyManager::is_steering_safe() const
{
	const float max_steering_angle = _param_max_steering_angle.get();
	const float current_steering_angle = fabsf(_steering_status.actual_angle_rad);

	return (current_steering_angle <= max_steering_angle);
}

bool SafetyManager::is_stability_safe() const
{
	const float max_roll = _param_max_roll.get();
	const float max_pitch = _param_max_pitch.get();

	const matrix::Eulerf euler(matrix::Quatf(_vehicle_attitude.q));
	const float current_roll = fabsf(euler.phi());
	const float current_pitch = fabsf(euler.theta());

	return (current_roll <= max_roll) && (current_pitch <= max_pitch);
}

bool SafetyManager::is_communication_safe() const
{
	const uint64_t now = hrt_absolute_time();
	const uint64_t timeout_us = static_cast<uint64_t>(_param_comm_timeout.get() * 1e6f);

	// Check if we have recent data from critical subscriptions
	bool position_recent = (now - _vehicle_local_position.timestamp) < timeout_us;
	bool attitude_recent = (now - _vehicle_attitude.timestamp) < timeout_us;
	bool drivetrain_recent = (now - _drivetrain_status.timestamp) < timeout_us;
	bool hbridge_recent = (now - _hbridge_status.timestamp) < timeout_us;
	bool bucket_recent = (now - _bucket_status.timestamp) < timeout_us;

	return position_recent && attitude_recent && drivetrain_recent &&
	       hbridge_recent && bucket_recent;
}

bool SafetyManager::is_hbridge_safe() const
{
	// Check H-bridge enable state and fault conditions
	if (!_hbridge_status.enabled) {
		return false; // H-bridge disabled indicates unsafe condition
	}

	// Check for limit sensor activation (could indicate mechanical problem)
	if (_hbridge_status.forward_limit && _hbridge_status.reverse_limit) {
		return false; // Both limits active is impossible and indicates fault
	}

	// Check for excessive duty cycle (could indicate overload)
	const float max_safe_duty_cycle = 0.95f;
	if (fabsf(_hbridge_status.duty_cycle) > max_safe_duty_cycle) {
		return false; // Duty cycle too high
	}

	return true;
}

bool SafetyManager::is_bucket_safe() const
{
	// Check for motor faults
	if (_bucket_status.motor_fault || _bucket_status.encoder_fault) {
		return false;
	}

	// Check motor temperature
	const float max_motor_temp = 100.0f; // 100°C max temperature
	if (_bucket_status.motor_temperature_c > max_motor_temp) {
		return false;
	}

	// Check for excessive motor current (overload condition)
	const float max_motor_current = 50.0f; // 50A max current
	if (fabsf(_bucket_status.motor_current) > max_motor_current) {
		return false;
	}

	// Check system state (error state is unsafe)
	if (_bucket_status.state == 4) { // 4 = error state
		return false;
	}

	// Check stability warning from bucket system
	if (_bucket_status.stability_warning) {
		return false;
	}

	return true;
}

int SafetyManager::task_spawn(int argc, char *argv[])
{
	SafetyManager *instance = new SafetyManager();

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

int SafetyManager::print_status()
{
	PX4_INFO("Safety Manager Status:");
	PX4_INFO("  Safety Level: %d", static_cast<int>(_current_safety_level));
	PX4_INFO("  Safety Checks: %lu", (unsigned long)_safety_check_counter);
	PX4_INFO("  Safety Violations: %lu", (unsigned long)_safety_violations);
	PX4_INFO("  Speed Safe: %s", _speed_safe ? "YES" : "NO");
	PX4_INFO("  Steering Safe: %s", _steering_safe ? "YES" : "NO");
	PX4_INFO("  Stability Safe: %s", _stability_safe ? "YES" : "NO");
	PX4_INFO("  Communication Safe: %s", _communication_safe ? "YES" : "NO");
	PX4_INFO("  H-bridge Safe: %s", _hbridge_safe ? "YES" : "NO");
	PX4_INFO("  Bucket Safe: %s", _bucket_safe ? "YES" : "NO");

	perf_print_counter(_loop_perf);

	return 0;
}

int SafetyManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SafetyManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Safety Manager for Wheel Loader Robot

The safety manager monitors critical systems and publishes safety status:
- Speed monitoring with configurable limits
- Steering angle monitoring
- Vehicle stability monitoring (roll/pitch angles)
- Communication timeout detection
- Safety level assessment (Normal/Warning/Critical/Emergency)
- Safety permits for operational control

### Examples
Start the safety manager:
$ safety_manager start

Stop the safety manager:
$ safety_manager stop

Check status:
$ safety_manager status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safety_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int safety_manager_main(int argc, char *argv[])
{
	return SafetyManager::main(argc, argv);
}
