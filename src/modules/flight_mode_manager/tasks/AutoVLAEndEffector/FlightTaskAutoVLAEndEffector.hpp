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
 * @file FlightTaskAutoVLAEndEffector.hpp
 *
 * Flight task for autonomous VLA (Vision-Language-Action) end effector trajectory following
 * for wheel loader robot. This task receives VLA end effector trajectory setpoints from the
 * navigator and generates appropriate position/velocity setpoints for the vehicle
 * chassis control, along with bucket and boom angle setpoints.
 */

#pragma once

#include "../FlightTask/FlightTask.hpp"
#include <uORB/topics/vla_end_effector_setpoint_triplet.h>
#include <uORB/topics/chassis_trajectory_setpoint.h>
#include <uORB/topics/boom_trajectory_setpoint.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>

class FlightTaskAutoVLAEndEffector : public FlightTask
{
public:
	FlightTaskAutoVLAEndEffector();
	virtual ~FlightTaskAutoVLAEndEffector() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;
	bool updateInitialize() override;
	bool update() override;

protected:
	/**
	 * Process VLA end effector setpoint triplet and perform motion planning
	 */
	void _processVlaEndEffectorSetpointTriplet();

	/**
	 * Generate chassis trajectory using motion planning with position smoothing
	 */
	void _planChassisTrajectory();

	/**
	 * Generate boom and bucket trajectories using motion planning
	 */
	void _planEndEffectorTrajectories();

	/**
	 * Publish chassis, boom, and bucket setpoints
	 */
	void _publishTrajectorySetpoints();

	/**
	 * Check if VLA end effector setpoint triplet is valid
	 */
	bool _isVlaEndEffectorSetpointValid() const;

	/**
	 * Update trajectory constraints based on parameters
	 */
	void _updateTrajConstraints();

	/**
	 * Check for emergency braking condition
	 */
	void _checkEmergencyBraking();

	/** Reset position or velocity setpoints in case of EKF reset event */
	void _ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy) override;
	void _ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy) override;
	void _ekfResetHandlerPositionZ(float delta_z) override;
	void _ekfResetHandlerVelocityZ(float delta_vz) override;
	void _ekfResetHandlerHeading(float delta_psi) override;

private:
	// Subscriptions
	uORB::Subscription _sub_vla_end_effector_setpoint_triplet{ORB_ID(vla_end_effector_setpoint_triplet)};

	// Publications for wheel loader specific setpoints
	uORB::Publication<chassis_trajectory_setpoint_s> _pub_chassis_setpoint{ORB_ID(chassis_trajectory_setpoint)};
	uORB::Publication<boom_trajectory_setpoint_s> _pub_boom_setpoint{ORB_ID(boom_trajectory_setpoint)};
	uORB::Publication<bucket_trajectory_setpoint_s> _pub_bucket_setpoint{ORB_ID(bucket_trajectory_setpoint)};

	// VLA end effector setpoint triplet data
	vla_end_effector_setpoint_triplet_s _vla_setpoint_triplet{};

	// Generated setpoints
	chassis_trajectory_setpoint_s _chassis_setpoint{};
	boom_trajectory_setpoint_s _boom_setpoint{};
	bucket_trajectory_setpoint_s _bucket_setpoint{};

	// Timing
	hrt_abstime _last_vla_setpoint_update{0};
	static constexpr hrt_abstime VLA_EE_TIMEOUT{500000}; // 500ms timeout

	// Position smoothing for chassis trajectory
	PositionSmoothing _position_smoothing;
	matrix::Vector3f _unsmoothed_velocity_setpoint;

	// Target waypoints for position smoothing
	matrix::Vector3f _prev_wp{}; /**< Previous waypoint (local frame) */
	matrix::Vector3f _target{}; /**< Target waypoint (local frame) */
	matrix::Vector3f _next_wp{}; /**< Next waypoint (local frame) */

	// Emergency braking state
	bool _is_emergency_braking_active{false};

	// Yaw control
	float _yaw_sp_prev{NAN};
	AlphaFilter<float> _yawspeed_filter;

	// Wheel loader specific parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AUTOVLA_EE_MAX_VEL>) _param_autovla_ee_max_vel,
		(ParamFloat<px4::params::AUTOVLA_EE_MAX_ACC>) _param_autovla_ee_max_acc,
		(ParamFloat<px4::params::AUTOVLA_EE_BOOM_REACH>) _param_autovla_ee_boom_reach,
		(ParamFloat<px4::params::AUTOVLA_EE_JERK>) _param_autovla_ee_jerk,
		(ParamFloat<px4::params::AUTOVLA_EE_XY_ERR_MAX>) _param_autovla_ee_xy_err_max
	)
};
