/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Mode.hpp"
#include "Modes_generated.hpp"
#include "mode_intention.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_type_config.h>
#include <uORB/topics/mode_status.h>
#include <uORB/topics/failsafe_mode_request.h>

#include <new>

enum class ModeError : int {
	NoError = 0,
	InvalidMode = -1,
	ActivationFailed = -2
};

class ModeManager : public ModuleBase<ModeManager>, public ModuleParams, public px4::WorkItem
{
public:
	ModeManager();
	~ModeManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;
	void updateParams() override;
	void selectAndActivateMode();
	void handleCommand();
	void generateControlSetpoint(const float dt, const vehicle_local_position_s &vehicle_local_position);

	/**
	 * Wheel loader specific mode selection
	 * Handles VLA autonomous and manual modes for articulated wheel loaders
	 */
	void selectWheelLoaderMode();

	/**
	 * Rover specific mode selection
	 * Uses mode_change_logic from vehicle_type_config
	 */
	void selectRoverMode();

	/**
	 * Rotary wing (multicopter) specific mode selection
	 * Handles position control, altitude control, and auto modes for multirotors
	 */
	void selectRotaryWingMode();

	/**
	 * Fixed wing specific mode selection
	 * Handles altitude control, position control, and auto modes for fixed wing aircraft
	 */
	void selectFixedWingMode();

	/**
	 * Standard mode selection (fallback when no vehicle-specific selection is available)
	 * Uses _user_mode_intention from mode_change_request
	 */
	void selectStandardMode();

	/**
	 * Check if the requested mode is available for the current vehicle type
	 * @param operation_mode Operation mode to check
	 * @return true if mode is available, false otherwise
	 */
	bool isModeAvailableForVehicleType(uint8_t operation_mode) const;

	/**
	 * Switch to a specific task (for normal usage)
	 * @param task index to switch to
	 * @return 0 on success, <0 on error
	 */
	ModeError switchMode(ModeIndex new_mode_index);
	ModeError switchMode(int new_mode_index);

	/**
	 * Call this method to get the description of a mode error.
	 */
	const char *errorToString(const ModeError error);

	/**
	 * Check if any mode is active
	 * @return true if a mode is active, false if not
	 */
	bool isAnyModeActive() const { return _current_mode.mode; }

	void tryApplyCommandIfAny();

	/**
	 * @brief Handle mode change requests from system_manager
	 *
	 * Processes incoming mode_change_request messages and updates user mode intention.
	 */
	void handleModeChangeRequest();

	// generated
	int _initMode(ModeIndex mode_index);

	/**
	 * User mode intention handler - manages requested operation mode
	 */
	mode_manager::ModeIntention _mode_intention{this};

	/**
	 * Union with all existing modes: we use it to make sure that only the memory of the largest existing
	 * mode is needed, and to avoid using dynamic memory allocations.
	 */
	ModeUnion _mode_union; /**< storage for the currently active mode */

	struct mode_t {
		Mode *mode{nullptr};
		ModeIndex index{ModeIndex::None};
	} _current_mode{};

	/**
	 * Current operation mode value (uint8_t) - tracked when modes are activated
	 * This is the authoritative source for mode_status.current_mode
	 */
	uint8_t _current_operation_mode{mode_status_s::OPERATION_MODE_AUTO_LOITER};

	int8_t _old_landing_gear_position{landing_gear_s::GEAR_KEEP};
	uint8_t _takeoff_state{takeoff_status_s::TAKEOFF_STATE_UNINITIALIZED};

	bool _no_matching_mode_error_printed{false};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")}; ///< loop duration performance counter
	hrt_abstime _time_stamp_last_loop{0}; ///< time stamp of last loop iteration

	vehicle_command_s _current_command{};
	bool _command_failed{false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _takeoff_status_sub{ORB_ID(takeoff_status)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::SubscriptionData<vehicle_control_mode_s> _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::SubscriptionData<vehicle_land_detected_s> _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_type_config_s> _vehicle_type_config_sub{ORB_ID(vehicle_type_config)};

	uORB::Publication<landing_gear_s> _landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<trajectory_setpoint_s> _control_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_constraints_s> _vehicle_constraints_pub{ORB_ID(vehicle_constraints)};
	uORB::Publication<mode_status_s> _mode_status_pub{ORB_ID(mode_status)};
	uORB::Publication<vehicle_control_mode_s> _vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};

	uORB::Subscription _failsafe_mode_request_sub{ORB_ID(failsafe_mode_request)};

	/**
	 * @brief Handle failsafe mode requests from system_manager
	 */
	void handleFailsafeModeRequest();

	/**
	 * @brief Publish current mode status for system_manager and other modules
	 */
	void publishModeStatus();

	/**
	 * @brief Update and publish vehicle control mode based on current operation mode
	 */
	void updateControlMode();

	/**
	 * @brief Get bitmask of valid modes for current vehicle type
	 */
	uint32_t getValidModesMask() const;

	/**
	 * @brief Get bitmask of modes that can be set in current state
	 */
	uint32_t getCanSetModesMask() const;

	bool _failsafe_mode_active{false};		///< Current mode is due to failsafe
	uint8_t _last_mode_change_result{mode_status_s::RESULT_SUCCESS};
	uint8_t _previous_mode{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MPC_POS_MODE>) _param_mpc_pos_mode
	);
};
