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
 * @file FlightTaskAutoVLA.cpp
 */

#include "FlightTaskAutoVLA.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

FlightTaskAutoVLA::FlightTaskAutoVLA() :
	FlightTask()
{
}

bool FlightTaskAutoVLA::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	// Initialize with current position
	_position_setpoint = _position;
	_velocity_setpoint = _velocity;
	_yaw_setpoint = _yaw;
	_yawspeed_setpoint = 0.0f;

	// Reset VLA trajectory
	_vla_trajectory = {};
	_last_vla_update = 0;

	// Reset setpoints
	_chassis_setpoint = {};
	_end_effector_setpoint = {};

	return ret;
}

void FlightTaskAutoVLA::reActivate()
{
	FlightTask::reActivate();
}

bool FlightTaskAutoVLA::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	// Update VLA trajectory subscription
	_sub_vla_trajectory_setpoint.update(&_vla_trajectory);

	// Check if we have valid position and velocity
	ret = ret && _position.isAllFinite() && _velocity.isAllFinite();

	return ret;
}

bool FlightTaskAutoVLA::update()
{
	bool ret = FlightTask::update();

	if (!_isVlaTrajectoryValid()) {
		// No valid VLA trajectory - hold current position
		_position_setpoint = _position;
		_velocity_setpoint.setZero();
		_yawspeed_setpoint = 0.0f;

		// Set invalid setpoints for chassis and end effector
		_chassis_setpoint.valid = false;
		_end_effector_setpoint.valid = false;

	} else {
		// Process VLA trajectory and decompose into chassis and end effector
		_processVlaTrajectory();
		_decomposeVlaSetpoint();
	}

	// Publish trajectory setpoints for wheel loader
	_publishTrajectorySetpoints();

	return ret;
}

void FlightTaskAutoVLA::_processVlaTrajectory()
{
	// Extract bucket target position from VLA trajectory
	// VLA provides bucket position in world coordinates
	Vector3f bucket_target(_vla_trajectory.bucket_position_x,
			       _vla_trajectory.bucket_position_y,
			       _vla_trajectory.bucket_position_z);

	// For now, use bucket XY position as chassis target position
	// Z-component will be handled by boom/bucket kinematics
	_position_setpoint(0) = bucket_target(0);
	_position_setpoint(1) = bucket_target(1);
	_position_setpoint(2) = _position(2); // Maintain current Z position (ground level)

	// Set velocity based on trajectory constraints
	float max_vel = math::constrain(_vla_trajectory.max_velocity, 0.1f, _param_autovla_max_vel.get());

	// Calculate desired velocity towards target
	Vector2f pos_error(bucket_target(0) - _position(0), bucket_target(1) - _position(1));
	float distance = pos_error.norm();

	if (distance > 0.01f) { // Avoid division by zero
		Vector2f velocity_dir = pos_error.normalized();
		float desired_speed = math::min(distance, max_vel);
		_velocity_setpoint(0) = velocity_dir(0) * desired_speed;
		_velocity_setpoint(1) = velocity_dir(1) * desired_speed;
	} else {
		_velocity_setpoint(0) = 0.0f;
		_velocity_setpoint(1) = 0.0f;
	}

	_velocity_setpoint(2) = 0.0f;

	// Set yaw from VLA trajectory
	_yaw_setpoint = _vla_trajectory.bucket_orientation_yaw;
	_yawspeed_setpoint = 0.0f; // Could be derived from trajectory if needed
}

void FlightTaskAutoVLA::_decomposeVlaSetpoint()
{
	// Decompose VLA bucket position into chassis position and end effector angles
	// This is a simplified kinematic decomposition

	// Chassis setpoint (XY position and yaw)
	_chassis_setpoint.timestamp = hrt_absolute_time();
	_chassis_setpoint.x_position = _position_setpoint(0);
	_chassis_setpoint.y_position = _position_setpoint(1);
	_chassis_setpoint.yaw = _yaw_setpoint;
	_chassis_setpoint.x_velocity = _velocity_setpoint(0);
	_chassis_setpoint.y_velocity = _velocity_setpoint(1);
	_chassis_setpoint.z_velocity = _velocity_setpoint(2);
	_chassis_setpoint.yaw_rate = _yawspeed_setpoint;
	_chassis_setpoint.valid = true;

	// End effector setpoint (boom and bucket angles)
	// Calculate bucket height and reach from bucket position
	Vector3f bucket_pos(_vla_trajectory.bucket_position_x,
			    _vla_trajectory.bucket_position_y,
			    _vla_trajectory.bucket_position_z);

	// Calculate relative bucket position from chassis
	Vector2f bucket_relative(bucket_pos(0) - _position(0), bucket_pos(1) - _position(1));
	float bucket_reach = bucket_relative.norm();
	float bucket_height = bucket_pos(2);

	// Simple inverse kinematics: assume boom angle based on reach and height
	// This is simplified - real implementation would use proper IK
	float boom_reach = _param_autovla_boom_reach.get();
	float boom_angle = atan2f(bucket_height, math::min(bucket_reach, boom_reach));

	// Bucket angle from VLA orientation (pitch component)
	float bucket_angle = _vla_trajectory.bucket_orientation_pitch;

	_end_effector_setpoint.timestamp = hrt_absolute_time();
	_end_effector_setpoint.boom_angle = boom_angle;
	_end_effector_setpoint.bucket_angle = bucket_angle;
	_end_effector_setpoint.boom_angle_rate = 0.0f; // Could be derived from trajectory
	_end_effector_setpoint.bucket_angle_rate = 0.0f;
	_end_effector_setpoint.valid = true;
	_end_effector_setpoint.priority = 100;
	_end_effector_setpoint.hold_position = false;
	_end_effector_setpoint.emergency_stop = _vla_trajectory.emergency_stop;
	_end_effector_setpoint.sequence_id = _vla_trajectory.sequence_id;
	_end_effector_setpoint.sequence_complete = _vla_trajectory.sequence_complete;
	_end_effector_setpoint.confidence_score = _vla_trajectory.confidence_score;
}

void FlightTaskAutoVLA::_publishTrajectorySetpoints()
{
	// Publish chassis trajectory setpoint
	_pub_chassis_setpoint.publish(_chassis_setpoint);

	// Publish end effector trajectory setpoint
	_pub_end_effector_setpoint.publish(_end_effector_setpoint);
}

bool FlightTaskAutoVLA::_isVlaTrajectoryValid() const
{
	if (!_vla_trajectory.valid_output || !_vla_trajectory.enable_trajectory) {
		return false;
	}

	// Check if trajectory is recent
	hrt_abstime now = hrt_absolute_time();
	if (_vla_trajectory.timestamp > 0 && (now - _vla_trajectory.timestamp) < VLA_TIMEOUT) {
		return true;
	}

	return false;
}
