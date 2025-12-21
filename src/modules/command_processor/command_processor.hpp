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
 * @file command_processor.hpp
 *
 * Command Processor Module
 *
 * Central module for processing vehicle_command_s messages.
 * Dispatches commands to vehicle-specific handlers and publishes
 * mode_change_request and automation_task messages.
 */

#pragma once

#include "vehicle_command_handler.hpp"
#include "airplane_command_handler.hpp"
#include "wheel_loader_command_handler.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_identity.h>
#include <uORB/topics/mode_status.h>
#include <uORB/topics/mode_change_request.h>
#include <uORB/topics/mode_change_result.h>
#include <uORB/topics/automation_task.h>
#include <uORB/topics/automation_task_result.h>
#include <uORB/topics/arming_request.h>
#include <uORB/topics/calibration_request.h>
#include <uORB/topics/reboot_request.h>
#include <uORB/topics/storage_request.h>
#include <uORB/topics/actuator_test_request.h>
#include <uORB/topics/prearm_check_request.h>
#include <uORB/topics/flight_termination_request.h>
#include <uORB/topics/set_home_request.h>

using namespace time_literals;

namespace command_processor
{

class CommandProcessor : public ModuleBase<CommandProcessor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CommandProcessor();
	~CommandProcessor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/**
	 * @brief Select the appropriate command handler for the current vehicle type
	 */
	void selectHandler();

	/**
	 * @brief Process a single vehicle command
	 * @param cmd The command to process
	 * @return true if command was handled
	 */
	bool processCommand(const vehicle_command_s &cmd);

	/**
	 * @brief Publish mode change request
	 */
	void publishModeChangeRequest(const vehicle_command_s &cmd,
				      const CommandContext &ctx);

	/**
	 * @brief Publish automation task
	 */
	void publishAutomationTask(const CommandContext &ctx);

	/**
	 * @brief Publish arming request
	 */
	void publishArmingRequest(const vehicle_command_s &cmd,
				  const CommandContext &ctx);

	/**
	 * @brief Publish calibration request
	 */
	void publishCalibrationRequest(const vehicle_command_s &cmd,
				       const CommandContext &ctx);

	/**
	 * @brief Publish reboot request
	 */
	void publishRebootRequest(const vehicle_command_s &cmd,
				  const CommandContext &ctx);

	/**
	 * @brief Publish storage request
	 */
	void publishStorageRequest(const vehicle_command_s &cmd,
				   const CommandContext &ctx);

	/**
	 * @brief Publish actuator test request
	 */
	void publishActuatorTestRequest(const vehicle_command_s &cmd,
					const CommandContext &ctx);

	/**
	 * @brief Publish prearm check request
	 */
	void publishPrearmCheckRequest(const vehicle_command_s &cmd,
				       const CommandContext &ctx);

	/**
	 * @brief Publish flight termination request
	 */
	void publishFlightTerminationRequest(const vehicle_command_s &cmd,
					     const CommandContext &ctx);

	/**
	 * @brief Publish set home request
	 */
	void publishSetHomeRequest(const vehicle_command_s &cmd,
				   const CommandContext &ctx);

	/**
	 * @brief Send command acknowledgment
	 */
	void answerCommand(const vehicle_command_s &cmd, uint8_t result);

	/**
	 * @brief Check for and handle results
	 */
	void handleResults();

	// Vehicle-specific handlers (statically allocated)
	AirPlaneCommandHandler _airplane_handler;
	WheelLoaderCommandHandler _wheel_loader_handler;

	// Current active handler
	const VehicleCommandHandler *_active_handler{&_airplane_handler};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<mode_status_s> _mode_status_sub{ORB_ID(mode_status)};
	uORB::Subscription _mode_change_result_sub{ORB_ID(mode_change_result)};
	uORB::Subscription _automation_task_result_sub{ORB_ID(automation_task_result)};

	// Publications
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<mode_change_request_s> _mode_change_request_pub{ORB_ID(mode_change_request)};
	uORB::Publication<automation_task_s> _automation_task_pub{ORB_ID(automation_task)};
	uORB::Publication<arming_request_s> _arming_request_pub{ORB_ID(arming_request)};
	uORB::Publication<calibration_request_s> _calibration_request_pub{ORB_ID(calibration_request)};
	uORB::Publication<reboot_request_s> _reboot_request_pub{ORB_ID(reboot_request)};
	uORB::Publication<storage_request_s> _storage_request_pub{ORB_ID(storage_request)};
	uORB::Publication<actuator_test_request_s> _actuator_test_request_pub{ORB_ID(actuator_test_request)};
	uORB::Publication<prearm_check_request_s> _prearm_check_request_pub{ORB_ID(prearm_check_request)};
	uORB::Publication<flight_termination_request_s> _flight_termination_request_pub{ORB_ID(flight_termination_request)};
	uORB::Publication<set_home_request_s> _set_home_request_pub{ORB_ID(set_home_request)};

	// State tracking
	uint8_t _current_vehicle_type{vehicle_identity_s::VEHICLE_TYPE_ROTARY_WING};


	// Pending command for async ACK
	struct PendingCommand {
		uint16_t command{0};
		uint8_t source_system{0};
		uint8_t source_component{0};
		hrt_abstime timestamp{0};
		bool waiting_for_mode{false};
		bool waiting_for_task{false};
	};
	PendingCommand _pending_command{};

	// Performance counter
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_VEHICLE_TYPE>) _param_sys_vehicle_type
	);
};

} // namespace command_processor
