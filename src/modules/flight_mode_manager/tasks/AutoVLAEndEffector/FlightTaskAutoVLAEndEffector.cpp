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
 * @file FlightTaskAutoVLAEndEffector.cpp
 * 
 * Flight task with motion planning for VLA end effector trajectory following
 */

#include "FlightTaskAutoVLAEndEffector.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

// Constants
static constexpr float MIN_POSITION_THRESHOLD = 0.01f;  // Minimum distance threshold in meters

FlightTaskAutoVLAEndEffector::FlightTaskAutoVLAEndEffector() :
	FlightTask(),
	_yawspeed_filter(0.2f) // Initialize yaw speed filter with time constant
{
}

bool FlightTaskAutoVLAEndEffector::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	// Initialize with current position
	_position_setpoint = _position;
	_velocity_setpoint = _velocity;
	_yaw_setpoint = _yaw;
	_yawspeed_setpoint = 0.0f;

	// Initialize position smoothing from last setpoint or current state
	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f pos_prev{last_setpoint.position};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current position
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// No acceleration estimate available, set to zero if the setpoint is NAN
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = 0.f; }
	}

	_position_smoothing.reset(accel_prev, vel_prev, pos_prev);

	_yaw_sp_prev = PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw;
	_updateTrajConstraints();

	// Reset VLA end effector setpoint triplet
	_vla_setpoint_triplet = {};
	_last_vla_setpoint_update = 0;

	// Reset setpoints
	_chassis_setpoint = {};
	_boom_setpoint = {};
	_bucket_setpoint = {};

	// Reset waypoints
	_prev_wp = _position;
	_target = _position;
	_next_wp = _position;

	_is_emergency_braking_active = false;

	return ret;
}

void FlightTaskAutoVLAEndEffector::reActivate()
{
	FlightTask::reActivate();

	// On ground, reset acceleration and velocity to zero
	_position_smoothing.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, _position);
}

bool FlightTaskAutoVLAEndEffector::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	// Update VLA end effector setpoint triplet subscription
	_sub_vla_end_effector_setpoint_triplet.update(&_vla_setpoint_triplet);

	// Check if we have valid position and velocity
	ret = ret && _position.isAllFinite() && _velocity.isAllFinite();

	return ret;
}

bool FlightTaskAutoVLAEndEffector::update()
{
	bool ret = FlightTask::update();

	// Always reset constraints because they might change
	_setDefaultConstraints();

	if (!_isVlaEndEffectorSetpointValid()) {
		// No valid VLA end effector setpoint - hold current position
		_position_setpoint = _position;
		_velocity_setpoint.setZero();
		_yawspeed_setpoint = 0.0f;

		// Set invalid setpoints for chassis, boom, and bucket
		_chassis_setpoint.valid = false;
		_boom_setpoint.valid = false;
		_bucket_setpoint.valid = false;

	} else {
		// Process VLA end effector setpoint triplet
		_processVlaEndEffectorSetpointTriplet();

		// Check for emergency braking
		_checkEmergencyBraking();

		// Plan chassis trajectory with position smoothing
		_planChassisTrajectory();

		// Plan end effector trajectories
		_planEndEffectorTrajectories();
	}

	// Publish trajectory setpoints for wheel loader
	_publishTrajectorySetpoints();

	return ret;
}

void FlightTaskAutoVLAEndEffector::_processVlaEndEffectorSetpointTriplet()
{
	// Update timestamp
	_last_vla_setpoint_update = _vla_setpoint_triplet.timestamp;

	// Extract current bucket target position from setpoint triplet
	Vector3f bucket_target(_vla_setpoint_triplet.current_bucket_x,
			       _vla_setpoint_triplet.current_bucket_y,
			       _vla_setpoint_triplet.current_bucket_z);

	// Set up waypoints for position smoothing
	// Previous waypoint
	if (_vla_setpoint_triplet.previous_valid) {
		_prev_wp(0) = _vla_setpoint_triplet.previous_bucket_x;
		_prev_wp(1) = _vla_setpoint_triplet.previous_bucket_y;
		_prev_wp(2) = _position(2); // Z at ground level
	} else {
		_prev_wp = _position; // Use current position if no previous
	}

	// Current target waypoint
	// For chassis control, use bucket XY position
	// Z-component will be handled by boom/bucket kinematics
	_target(0) = bucket_target(0);
	_target(1) = bucket_target(1);
	_target(2) = _position(2); // Maintain current Z position (ground level)

	// Next waypoint
	if (_vla_setpoint_triplet.next_valid) {
		_next_wp(0) = _vla_setpoint_triplet.next_bucket_x;
		_next_wp(1) = _vla_setpoint_triplet.next_bucket_y;
		_next_wp(2) = _position(2); // Z at ground level
	} else {
		_next_wp = _target; // Use current target if no next
	}

	// Set position setpoint for trajectory generation
	_position_setpoint = _target;

	// Set yaw from VLA end effector bucket orientation
	_yaw_setpoint = _vla_setpoint_triplet.current_bucket_yaw;
	_yawspeed_setpoint = 0.0f;
}

void FlightTaskAutoVLAEndEffector::_planChassisTrajectory()
{
	// Motion planning for chassis trajectory using PositionSmoothing library
	// This provides jerk-limited trajectory generation similar to FlightTaskAuto

	// Update trajectory constraints
	_updateTrajConstraints();

	// Prepare waypoints array for position smoothing [previous, current, next]
	Vector3f waypoints[] = {_prev_wp, _position_setpoint, _next_wp};

	// Check if emergency braking is active
	const bool force_zero_velocity_setpoint = _is_emergency_braking_active;

	// Generate smoothed setpoints using PositionSmoothing
	PositionSmoothing::PositionSmoothingSetpoints smoothed_setpoints;
	_position_smoothing.generateSetpoints(
		_position,
		waypoints,
		_velocity_setpoint,
		_deltatime,
		force_zero_velocity_setpoint,
		smoothed_setpoints
	);

	// Apply smoothed setpoints
	_jerk_setpoint = smoothed_setpoints.jerk;
	_acceleration_setpoint = smoothed_setpoints.acceleration;
	_velocity_setpoint = smoothed_setpoints.velocity;
	_position_setpoint = smoothed_setpoints.position;
	_unsmoothed_velocity_setpoint = smoothed_setpoints.unsmoothed_velocity;

	// Generate chassis trajectory setpoint
	_chassis_setpoint.timestamp = hrt_absolute_time();
	_chassis_setpoint.x_position = _position_setpoint(0);
	_chassis_setpoint.y_position = _position_setpoint(1);
	_chassis_setpoint.yaw = _yaw_setpoint;
	_chassis_setpoint.x_velocity = _velocity_setpoint(0);
	_chassis_setpoint.y_velocity = _velocity_setpoint(1);
	_chassis_setpoint.z_velocity = _velocity_setpoint(2);
	_chassis_setpoint.yaw_rate = _yawspeed_setpoint;
	_chassis_setpoint.valid = true;

	// Note: Wheel slip rates are set from VLA setpoint in chassis controller
}

void FlightTaskAutoVLAEndEffector::_planEndEffectorTrajectories()
{
	// Motion planning for boom and bucket trajectories
	// Decomposes VLA end effector bucket position into boom height and bucket angle

	// Calculate bucket height and reach from bucket position
	Vector3f bucket_pos(_vla_setpoint_triplet.current_bucket_x,
			    _vla_setpoint_triplet.current_bucket_y,
			    _vla_setpoint_triplet.current_bucket_z);

	// Calculate relative bucket position from chassis
	Vector2f bucket_relative(bucket_pos(0) - _position(0), bucket_pos(1) - _position(1));
	float bucket_reach = bucket_relative.norm();
	float bucket_height = bucket_pos(2);

	// Simple inverse kinematics: assume boom angle based on reach and height
	// This is simplified - real implementation would use proper IK
	float boom_reach = _param_autovla_ee_boom_reach.get();
	float boom_angle = atan2f(bucket_height, math::min(bucket_reach, boom_reach));

	// Bucket angle from VLA end effector orientation (pitch component)
	float bucket_angle = _vla_setpoint_triplet.current_bucket_pitch;

	// Plan smooth boom trajectory
	_boom_setpoint.timestamp = hrt_absolute_time();
	_boom_setpoint.angle = boom_angle;
	_boom_setpoint.angular_velocity = 0.0f; // Could be derived from trajectory
	_boom_setpoint.angular_acceleration = 0.0f;
	_boom_setpoint.max_velocity = _param_autovla_ee_max_vel.get();
	_boom_setpoint.max_acceleration = _param_autovla_ee_max_acc.get();
	_boom_setpoint.control_mode = boom_trajectory_setpoint_s::MODE_POSITION;
	_boom_setpoint.trajectory_time = 0.0f;
	_boom_setpoint.time_from_start = 0.0f;
	_boom_setpoint.priority = 100;
	_boom_setpoint.valid = true;

	// Plan smooth bucket trajectory
	_bucket_setpoint.timestamp = hrt_absolute_time();
	_bucket_setpoint.control_mode = bucket_trajectory_setpoint_s::MODE_POSITION;
	_bucket_setpoint.target_angle = bucket_angle;
	_bucket_setpoint.angular_velocity = 0.0f; // Could be derived from trajectory
	_bucket_setpoint.angular_acceleration = 0.0f;
	_bucket_setpoint.max_velocity = _param_autovla_ee_max_vel.get();
	_bucket_setpoint.max_acceleration = _param_autovla_ee_max_acc.get();
	_bucket_setpoint.trajectory_time = 0.0f;
	_bucket_setpoint.time_from_start = 0.0f;
	_bucket_setpoint.priority = 100;
	_bucket_setpoint.valid = true;
}

void FlightTaskAutoVLAEndEffector::_publishTrajectorySetpoints()
{
	// Publish chassis trajectory setpoint
	_pub_chassis_setpoint.publish(_chassis_setpoint);

	// Publish boom trajectory setpoint (for bucket height control)
	_pub_boom_setpoint.publish(_boom_setpoint);

	// Publish bucket trajectory setpoint (for bucket angle control)
	_pub_bucket_setpoint.publish(_bucket_setpoint);
}

bool FlightTaskAutoVLAEndEffector::_isVlaEndEffectorSetpointValid() const
{
	// Check if current setpoint is valid
	if (!_vla_setpoint_triplet.current_valid) {
		return false;
	}

	// Check if setpoint is recent
	if (_vla_setpoint_triplet.timestamp > 0 && hrt_elapsed_time(&_vla_setpoint_triplet.timestamp) < VLA_EE_TIMEOUT) {
		return true;
	}

	return false;
}

void FlightTaskAutoVLAEndEffector::_updateTrajConstraints()
{
	// Update trajectory constraints for position smoothing
	// Based on wheel loader specific parameters

	// Set maximum horizontal velocity
	float max_velocity = _param_autovla_ee_max_vel.get();
	
	// Allow VLA setpoint to further constrain velocity if needed
	if (_vla_setpoint_triplet.current_valid && _vla_setpoint_triplet.max_velocity > 0.1f) {
		max_velocity = math::min(max_velocity, _vla_setpoint_triplet.max_velocity);
	}

	_position_smoothing.setMaxVelocityXY(max_velocity);
	_position_smoothing.setMaxVelocityZ(max_velocity * 0.5f); // Slower vertical

	// Set maximum horizontal acceleration
	float max_acceleration = _param_autovla_ee_max_acc.get();
	
	// Allow VLA setpoint to further constrain acceleration if needed
	if (_vla_setpoint_triplet.current_valid && _vla_setpoint_triplet.max_acceleration > 0.1f) {
		max_acceleration = math::min(max_acceleration, _vla_setpoint_triplet.max_acceleration);
	}

	_position_smoothing.setMaxAccelerationXY(max_acceleration);
	_position_smoothing.setMaxAccelerationZ(max_acceleration * 0.5f); // Slower vertical

	// Set maximum jerk
	_position_smoothing.setMaxJerk(_param_autovla_ee_jerk.get());

	// Set trajectory P gain for position tracking
	_position_smoothing.setHorizontalTrajectoryGain(3.0f); // Moderate tracking
}

void FlightTaskAutoVLAEndEffector::_checkEmergencyBraking()
{
	// Check if we need emergency braking based on position error
	// This is a safety feature to prevent runaway

	Vector2f pos_error_xy(_position_setpoint(0) - _position(0), _position_setpoint(1) - _position(1));
	float distance_xy = pos_error_xy.norm();

	// If position error exceeds maximum, activate emergency braking
	if (distance_xy > _param_autovla_ee_xy_err_max.get()) {
		if (!_is_emergency_braking_active) {
			PX4_WARN("Emergency braking activated: position error %.2f m", (double)distance_xy);
			_is_emergency_braking_active = true;
		}
	} else if (distance_xy < _param_autovla_ee_xy_err_max.get() * 0.5f) {
		// Deactivate when error is reduced
		if (_is_emergency_braking_active) {
			PX4_INFO("Emergency braking deactivated");
			_is_emergency_braking_active = false;
		}
	}
}

void FlightTaskAutoVLAEndEffector::_ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy)
{
	// Reset position setpoint on EKF position reset
	_position_setpoint(0) += delta_xy(0);
	_position_setpoint(1) += delta_xy(1);
}

void FlightTaskAutoVLAEndEffector::_ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy)
{
	// Reset velocity setpoint on EKF velocity reset
	_velocity_setpoint(0) += delta_vxy(0);
	_velocity_setpoint(1) += delta_vxy(1);
}

void FlightTaskAutoVLAEndEffector::_ekfResetHandlerPositionZ(float delta_z)
{
	// Reset Z position setpoint on EKF position Z reset
	_position_setpoint(2) += delta_z;
}

void FlightTaskAutoVLAEndEffector::_ekfResetHandlerVelocityZ(float delta_vz)
{
	// Reset Z velocity setpoint on EKF velocity Z reset
	_velocity_setpoint(2) += delta_vz;
}

void FlightTaskAutoVLAEndEffector::_ekfResetHandlerHeading(float delta_psi)
{
	// Reset yaw setpoint on EKF heading reset
	_yaw_setpoint += delta_psi;
	_yaw_sp_prev += delta_psi;
}
