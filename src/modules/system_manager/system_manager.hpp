/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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

/*   Helper classes  */
#include "arming_handler.hpp"
#include "calibration_handler.hpp"
#include "failsafe_handler.hpp"
#include "failsafe/failsafe.h"
#include "failure_detector/failure_detector.hpp"
#include "health_and_arming_checks/health_and_arming_checks.hpp"
#include "home_position.hpp"
#include "safety.hpp"
// Mode status received from mode_manager via uORB (replaces local ModeIntention)
#include "worker_thread.hpp"

#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/failsafe_mode_request.h>
#include <uORB/topics/mode_change_result.h>
#include <uORB/topics/mode_status.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_identity.h>
#include <uORB/topics/vehicle_status.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/action_request.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/arming_request.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/flight_termination_request.h>
#include <uORB/topics/iridiumsbd_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/mode_change_request.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/throw_launch_status.h>
#include <uORB/topics/prearm_check_request.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/set_home_request.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vtol_vehicle_status.h>

using math::constrain;
using systemlib::Hysteresis;

typedef enum {
	TRANSITION_DENIED = -1,
	TRANSITION_NOT_CHANGED = 0,
	TRANSITION_CHANGED
} transition_result_t;

using arm_disarm_reason_t = events::px4::enums::arm_disarm_reason_t;

using namespace time_literals;

class SystemManager : public ModuleBase<SystemManager>, public ModuleParams
{
public:
	SystemManager();
	~SystemManager();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SystemManager *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void enable_hil();

private:
	bool is_armed() const {
		return (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED); }

	void answer_command(const vehicle_command_s &cmd, uint8_t result);

	transition_result_t arm(arm_disarm_reason_t calling_reason, bool run_prearm_checks = true);

	transition_result_t disarm(arm_disarm_reason_t calling_reason, bool forced = false);

	void battery_status_check();

	void control_status_leds(bool changed, const uint8_t battery_warning);

	/**
	 * Checks the status of all available data links and handles switching between different system telemetry states.
	 */
	void data_link_check();

	void manual_control_check();

	/**
	 * @brief Handle mode change requests from CommandProcessor
	 */
	void handle_mode_change_requests();



	/**
	 * @brief Handle set home requests from CommandProcessor
	 */
	void handle_set_home_requests();

	// Note: update_control_mode moved to mode_manager

	void send_parachute_command();

	void check_mission_result();

	void handle_power_button_state();

	void system_power_update();

	void land_detector_update();

	void safety_button_update();

	bool is_throw_launch_in_progress() const;

	void throw_launch_update();

	void vtol_status_update();

	void update_tunes();

	void check_worker_thread();

	bool get_prearm_state() const;

	/**
	 * @brief Publish mode change request to mode_manager
	 * Called for RC/action-based mode changes (not from command_processor)
	 */
	void publish_mode_change_request(uint8_t requested_mode, uint8_t source);

	/**
	 * @brief Update vehicle_status from ModeStatus subscription
	 * Populates operation_mode from mode_manager's ModeStatus
	 */
	void update_mode_status();

	// External/executor mode functions removed - now handled by mode_manager

	void update_parameters();

	void check_and_inform_ready_for_takeoff();

	// on_failsafe_notify_user moved to FailsafeHandler
	// handle_commands_from_mode_executors removed - mode executor management now in mode_manager

	enum class PrearmedMode {
		DISABLED = 0,
		SAFETY_BUTTON = 1,
		ALWAYS = 2
	};

	enum class RcOverrideBits : int32_t {
		AUTO_MODE_BIT = (1 << 0),
	};

	/* Decouple update interval and hysteresis counters, all depends on intervals */
	static constexpr uint64_t COMMANDER_MONITORING_INTERVAL{10_ms};

	vehicle_status_s        _vehicle_status{};
	vehicle_identity_s      _vehicle_identity{};

	FailsafeHandler		_failsafe_handler{this};
	FailureDetector		_failure_detector{this};
	HealthAndArmingChecks	_health_and_arming_checks{this, _vehicle_status};
	const failsafe_flags_s &_failsafe_flags{_health_and_arming_checks.failsafeFlags()};
	HomePosition 		_home_position{_failsafe_flags};
	Safety			_safety{};
	WorkerThread 		_worker_thread{};
	CalibrationHandler	_calibration_handler{this, _safety, _worker_thread};
	ArmingHandler		_arming_handler{this, _health_and_arming_checks, _safety, _home_position};

	// Mode tracking (mode_manager is the authority, we track locally for failsafe logic)
	uint8_t _cached_user_intended_mode{mode_status_s::OPERATION_MODE_AUTO_LOITER};
	uint8_t _current_operation_mode{mode_status_s::OPERATION_MODE_AUTO_LOITER};  // Current active mode from mode_manager
	hrt_abstime _operation_mode_timestamp{0};  // Time when current operation mode was activated
	bool _ever_had_mode_change{false};
	bool _had_mode_change{false};
	config_overrides_s   _config_overrides{};

	hrt_abstime _datalink_last_heartbeat_open_drone_id_system{0};
	hrt_abstime _datalink_last_heartbeat_gcs{0};
	hrt_abstime _datalink_last_heartbeat_onboard_controller{0};
	hrt_abstime _datalink_last_heartbeat_parachute_system{0};

	hrt_abstime _high_latency_datalink_timestamp{0};
	hrt_abstime _high_latency_datalink_lost{0};
	hrt_abstime _high_latency_datalink_regained{0};

	hrt_abstime _boot_timestamp{0};
	hrt_abstime _last_disarmed_timestamp{0};
	hrt_abstime _overload_start{0};		///< time when CPU overload started

#if !defined(CONFIG_ARCH_LEDS) && defined(BOARD_HAS_CONTROL_STATUS_LEDS)
	hrt_abstime _led_armed_state_toggle {0};
#endif
	hrt_abstime _led_overload_toggle {0};

	hrt_abstime _last_health_and_arming_check{0};

	uint8_t		_battery_warning{battery_status_s::WARNING_NONE};

	bool _failsafe_user_override_request{false}; ///< override request due to stick movements

	bool _open_drone_id_system_lost{true};
	bool _onboard_controller_lost{false};
	bool _parachute_system_lost{true};

	bool _last_overload{false};
	bool _mode_switch_mapped{false};

	bool _is_throttle_above_center{false};
	bool _is_throttle_low{false};

	bool _arm_tune_played{false};
	bool _have_taken_off_since_arming{false};
	bool _status_changed{true};

	vehicle_land_detected_s	_vehicle_land_detected{};

	// system_manager publications
	vtol_vehicle_status_s	_vtol_vehicle_status{};

	// Subscriptions
	uORB::Subscription					_cpuload_sub{ORB_ID(cpuload)};
	uORB::SubscriptionData<vehicle_control_mode_s>		_vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription					_iridiumsbd_status_sub{ORB_ID(iridiumsbd_status)};
	uORB::Subscription					_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription					_set_home_request_sub{ORB_ID(set_home_request)};
	uORB::Subscription					_system_power_sub{ORB_ID(system_power)};
	uORB::Subscription					_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription					_vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription					_vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};

	uORB::SubscriptionInterval				_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionMultiArray<telemetry_status_s>	_telemetry_status_subs{ORB_ID::telemetry_status};

#if defined(BOARD_HAS_POWER_CONTROL)
	uORB::Subscription					_power_button_state_sub {ORB_ID(power_button_state)};
#endif // BOARD_HAS_POWER_CONTROL

	uORB::SubscriptionData<mission_result_s>		_mission_result_sub{ORB_ID(mission_result)};
	uORB::SubscriptionData<mode_status_s>			_mode_status_sub{ORB_ID(mode_status)};
	uORB::SubscriptionData<throw_launch_status_s>		_throw_launch_status_sub{ORB_ID(throw_launch_status)};

	// Publications
	uORB::Publication<failure_detector_status_s>		_failure_detector_status_pub{ORB_ID(failure_detector_status)};
	uORB::Publication<mode_change_request_s>		_mode_change_request_pub{ORB_ID(mode_change_request)};
	uORB::Publication<mode_change_result_s>			_mode_change_result_pub{ORB_ID(mode_change_result)};
	uORB::Publication<vehicle_command_ack_s>		_vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s>			_vehicle_command_pub{ORB_ID(vehicle_command)};
	// Note: vehicle_control_mode publication moved to mode_manager
	uORB::Publication<vehicle_identity_s>			_vehicle_identity_pub{ORB_ID(vehicle_identity)};
	uORB::Publication<vehicle_status_s>			_vehicle_status_pub{ORB_ID(vehicle_status)};

	orb_advert_t _mavlink_log_pub{nullptr};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _prearm_check_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": prearm check")};

	// optional parameters
	param_t _param_mav_type{PARAM_INVALID};
	param_t _param_rc_map_fltmode{PARAM_INVALID};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::COM_DISARM_LAND>)  _param_com_disarm_land,
		(ParamFloat<px4::params::COM_DISARM_PRFLT>) _param_com_disarm_prflt,
		(ParamBool<px4::params::COM_DISARM_MAN>)    _param_com_disarm_man,
		(ParamInt<px4::params::COM_DL_LOSS_T>)      _param_com_dl_loss_t,
		(ParamInt<px4::params::COM_HLDL_LOSS_T>)    _param_com_hldl_loss_t,
		(ParamInt<px4::params::COM_HLDL_REG_T>)     _param_com_hldl_reg_t,
		(ParamBool<px4::params::COM_HOME_EN>)       _param_com_home_en,
		(ParamBool<px4::params::COM_HOME_IN_AIR>)   _param_com_home_in_air,
		(ParamInt<px4::params::COM_FLT_PROFILE>)    _param_com_flt_profile,
		(ParamBool<px4::params::COM_FORCE_SAFETY>)  _param_com_force_safety,
		(ParamFloat<px4::params::COM_KILL_DISARM>)  _param_com_kill_disarm,
		(ParamBool<px4::params::COM_MOT_TEST_EN>)   _param_com_mot_test_en,
		(ParamFloat<px4::params::COM_OBC_LOSS_T>)   _param_com_obc_loss_t,
		(ParamInt<px4::params::COM_PREARM_MODE>)    _param_com_prearm_mode,
		(ParamInt<px4::params::COM_RC_OVERRIDE>)    _param_com_rc_override,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,
		(ParamInt<px4::params::COM_FLIGHT_UUID>)    _param_com_flight_uuid,
		(ParamFloat<px4::params::COM_CPU_MAX>)      _param_com_cpu_max
	)
};
