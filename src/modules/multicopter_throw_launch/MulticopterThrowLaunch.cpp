/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file MulticopterThrowLaunch.cpp
 *
 * Standalone module to manage takeoff of a multicopter by manually throwing it into the air.
 *
 * @author Michał Barciś <mbarcis@mbarcis.net>
 */

#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/throw_launch_status.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class MulticopterThrowLaunch : public ModuleBase<MulticopterThrowLaunch>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	MulticopterThrowLaunch();
	~MulticopterThrowLaunch() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Main update of the state
	 * @param armed true if vehicle is armed
	 */
	void update(const bool armed);

	/**
	 * Publish the current throw launch status
	 */
	void publishStatus();

	enum class ThrowLaunchState : uint8_t {
		DISABLED = throw_launch_status_s::STATE_DISABLED,
		IDLE = throw_launch_status_s::STATE_IDLE,
		ARMED = throw_launch_status_s::STATE_ARMED,
		UNSAFE = throw_launch_status_s::STATE_UNSAFE,
		FLYING = throw_launch_status_s::STATE_FLYING
	};

	// Subscriptions
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// Publications
	uORB::Publication<throw_launch_status_s> _throw_launch_status_pub{ORB_ID(throw_launch_status)};

	ThrowLaunchState _throw_launch_state{ThrowLaunchState::DISABLED};
	matrix::Vector3f _last_velocity{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::COM_THROW_EN>) _param_com_throw_en,
		(ParamFloat<px4::params::COM_THROW_SPEED>) _param_com_throw_min_speed
	)
};

MulticopterThrowLaunch::MulticopterThrowLaunch() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool MulticopterThrowLaunch::init()
{
	ScheduleOnInterval(20_ms); // 50 Hz
	return true;
}

void MulticopterThrowLaunch::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Get armed state
	actuator_armed_s actuator_armed{};
	bool armed = false;

	if (_actuator_armed_sub.copy(&actuator_armed)) {
		armed = actuator_armed.armed;
	}

	update(armed);
	publishStatus();
}

void MulticopterThrowLaunch::update(const bool armed)
{
	if (_param_com_throw_en.get()) {
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position{};

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_last_velocity = matrix::Vector3f(vehicle_local_position.vx, vehicle_local_position.vy,
								  vehicle_local_position.vz);
			}
		}

		if (!armed && _throw_launch_state != ThrowLaunchState::IDLE) {
			events::send(events::ID("mc_throw_launch_not_ready"), events::Log::Critical, "Disarmed, don't throw");
			_throw_launch_state = ThrowLaunchState::IDLE;
		}

		switch (_throw_launch_state) {
		case ThrowLaunchState::IDLE:
			if (armed) {
				events::send(events::ID("mc_throw_launch_ready"), events::Log::Critical, "Ready for throw launch");
				_throw_launch_state = ThrowLaunchState::ARMED;
			}

			break;

		case ThrowLaunchState::ARMED:
			if (_last_velocity.longerThan(_param_com_throw_min_speed.get())) {
				PX4_INFO("Throw detected, motors will start once falling");
				_throw_launch_state = ThrowLaunchState::UNSAFE;
			}

			break;

		case ThrowLaunchState::UNSAFE:
			if (_last_velocity(2) > 0.f) {
				PX4_INFO("Throw and fall detected, starting motors");
				_throw_launch_state = ThrowLaunchState::FLYING;
			}

			break;

		case ThrowLaunchState::DISABLED:
		case ThrowLaunchState::FLYING:
			// Nothing to do
			break;
		}

	} else if (_throw_launch_state != ThrowLaunchState::DISABLED) {
		// make sure everything is reset when the throw launch is disabled
		_throw_launch_state = ThrowLaunchState::DISABLED;
	}
}

void MulticopterThrowLaunch::publishStatus()
{
	throw_launch_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = static_cast<uint8_t>(_throw_launch_state);
	status.throw_launch_in_progress = (_throw_launch_state != ThrowLaunchState::DISABLED
					   && _throw_launch_state != ThrowLaunchState::FLYING);
	status.ready_to_throw = (_throw_launch_state == ThrowLaunchState::ARMED);

	_throw_launch_status_pub.publish(status);
}

int MulticopterThrowLaunch::task_spawn(int argc, char *argv[])
{
	MulticopterThrowLaunch *instance = new MulticopterThrowLaunch();

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

int MulticopterThrowLaunch::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterThrowLaunch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module that manages takeoff of a multicopter by manually throwing it into the air.

When enabled, the module monitors vehicle velocity after arming. Once the throw
velocity threshold is exceeded and the vehicle starts falling, motors are started
automatically to begin flight.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_throw_launch", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_throw_launch_main(int argc, char *argv[])
{
	return MulticopterThrowLaunch::main(argc, argv);
}
