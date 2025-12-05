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
 * @file FlightTaskAutoVLA.hpp
 *
 * Flight task for autonomous VLA (Vision-Language-Action) trajectory following
 * for wheel loader robot. This task receives VLA trajectory setpoints from the
 * navigator and generates appropriate position/velocity setpoints for the vehicle
 * chassis control, along with bucket and boom angle setpoints.
 */

#pragma once

#include "../FlightTask/FlightTask.hpp"
#include <uORB/topics/vla_trajectory_setpoint.h>
#include <uORB/topics/chassis_trajectory_setpoint.h>
#include <uORB/topics/end_effector_trajectory_setpoint.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>

class FlightTaskAutoVLA : public FlightTask
{
public:
	FlightTaskAutoVLA();
	virtual ~FlightTaskAutoVLA() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;
	bool updateInitialize() override;
	bool update() override;

protected:
	/**
	 * Process VLA trajectory setpoint and generate vehicle setpoints
	 */
	void _processVlaTrajectory();

	/**
	 * Decompose VLA bucket position into chassis position and end effector angles
	 */
	void _decomposeVlaSetpoint();

	/**
	 * Publish chassis and end effector setpoints
	 */
	void _publishTrajectorySetpoints();

	/**
	 * Check if VLA trajectory is valid and recent
	 */
	bool _isVlaTrajectoryValid() const;

private:
	// Subscriptions
	uORB::Subscription _sub_vla_trajectory_setpoint{ORB_ID(vla_trajectory_setpoint)};

	// Publications for wheel loader specific setpoints
	uORB::Publication<chassis_trajectory_setpoint_s> _pub_chassis_setpoint{ORB_ID(chassis_trajectory_setpoint)};
	uORB::Publication<end_effector_trajectory_setpoint_s> _pub_end_effector_setpoint{ORB_ID(end_effector_trajectory_setpoint)};

	// VLA trajectory data
	vla_trajectory_setpoint_s _vla_trajectory{};

	// Generated setpoints
	chassis_trajectory_setpoint_s _chassis_setpoint{};
	end_effector_trajectory_setpoint_s _end_effector_setpoint{};

	// Timing
	hrt_abstime _last_vla_update{0};
	static constexpr hrt_abstime VLA_TIMEOUT{500000}; // 500ms timeout

	// Wheel loader specific parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AUTOVLA_MAX_VEL>) _param_autovla_max_vel,
		(ParamFloat<px4::params::AUTOVLA_MAX_ACC>) _param_autovla_max_acc,
		(ParamFloat<px4::params::AUTOVLA_BOOM_REACH>) _param_autovla_boom_reach
	)
};
