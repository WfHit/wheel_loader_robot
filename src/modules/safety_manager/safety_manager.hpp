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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/system_safety.h>
#include <uORB/topics/drivetrain_status.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/bucket_status.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

class SafetyManager : public ModuleBase<SafetyManager>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SafetyManager();
	~SafetyManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	void Run() override;

	/**
	 * Update uORB subscriptions
	 */
	void update_subscriptions();

	/**
	 * Monitor safety conditions
	 */
	void monitor_safety();

	/**
	 * Assess overall safety level
	 */
	void assess_safety_level();

	/**
	 * Publish safety status
	 */
	void publish_safety_status();

	/**
	 * Check if speed is within safe limits
	 */
	bool is_speed_safe() const;

	/**
	 * Check if steering is within safe limits
	 */
	bool is_steering_safe() const;

	/**
	 * Check if vehicle stability is safe
	 */
	bool is_stability_safe() const;

	/**
	 * Check if communication is healthy
	 */
	bool is_communication_safe() const;

	/**
	 * Check if H-bridge system is safe
	 */
	bool is_hbridge_safe() const;

	/**
	 * Check if bucket system is safe
	 */
	bool is_bucket_safe() const;

	// Safety levels
	enum class SafetyLevel : uint8_t {
		NORMAL = 0,
		CAUTION = 1,
		WARNING = 2,
		CRITICAL = 3,
		EMERGENCY = 4
	};

	// uORB subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _drivetrain_status_sub{ORB_ID(drivetrain_status)};
	uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};
	uORB::Subscription _hbridge_status_sub{ORB_ID(hbridge_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};

	// uORB publications
	uORB::Publication<system_safety_s> _system_safety_pub{ORB_ID(system_safety)};

	// uORB data structures
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};
	drivetrain_status_s _drivetrain_status{};
	steering_status_s _steering_status{};
	hbridge_status_s _hbridge_status{};
	bucket_status_s _bucket_status{};

	// Safety state
	SafetyLevel _current_safety_level{SafetyLevel::NORMAL};
	SafetyLevel _previous_safety_level{SafetyLevel::NORMAL};
	uint64_t _last_safety_check{0};
	uint32_t _safety_check_counter{0};
	uint32_t _safety_violations{0};
	uint64_t _last_fault_time{0};

	// Monitoring flags
	bool _speed_safe{true};
	bool _steering_safe{true};
	bool _stability_safe{true};
	bool _communication_safe{true};
	bool _hbridge_safe{true};
	bool _bucket_safe{true};

	// Timing
	static constexpr uint32_t SAFETY_CHECK_INTERVAL_US = 20000; // 50Hz
	static constexpr uint64_t COMMUNICATION_TIMEOUT_US = 1000000; // 1 second

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SM_MAX_SPEED>) _param_max_speed,
		(ParamFloat<px4::params::SM_MAX_STR_ANG>) _param_max_steering_angle,
		(ParamFloat<px4::params::SM_MAX_ROLL>) _param_max_roll,
		(ParamFloat<px4::params::SM_MAX_PITCH>) _param_max_pitch,
		(ParamFloat<px4::params::SM_COMM_TIMEOUT>) _param_comm_timeout
	)
};
