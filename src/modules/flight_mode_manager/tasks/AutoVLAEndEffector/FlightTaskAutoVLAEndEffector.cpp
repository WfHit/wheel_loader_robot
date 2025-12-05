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
 */

#include "FlightTaskAutoVLAEndEffector.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

// Constants
static constexpr float MIN_POSITION_THRESHOLD = 0.01f;  // Minimum distance threshold in meters

FlightTaskAutoVLAEndEffector::FlightTaskAutoVLAEndEffector() :
	FlightTask()
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

	// Reset VLA end effector trajectory
	_vla_end_effector_trajectory = {};
	_last_vla_end_effector_update = 0;

	// Reset setpoints
	_chassis_setpoint = {};
	_end_effector_setpoint = {};

	return ret;
}

void FlightTaskAutoVLAEndEffector::reActivate()
{
	FlightTask::reActivate();
}

bool FlightTaskAutoVLAEndEffector::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	// Update VLA end effector trajectory subscription
	_sub_vla_end_effector_trajectory_setpoint.update(&_vla_end_effector_trajectory);

	// Check if we have valid position and velocity
	ret = ret && _position.isAllFinite() && _velocity.isAllFinite();

	return ret;
}

bool FlightTaskAutoVLAEndEffector::update()
{
	bool ret = FlightTask::update();

	if (!_isVlaEndEffectorTrajectoryValid()) {
		// No valid VLA end effector trajectory - hold current position
		_position_setpoint = _position;
		_velocity_setpoint.setZero();
		_yawspeed_setpoint = 0.0f;

		// Set invalid setpoints for chassis and end effector
		_chassis_setpoint.valid = false;
		_end_effector_setpoint.valid = false;

	} else {
		// Process VLA end effector trajectory and decompose into chassis and end effector
		_processVlaEndEffectorTrajectory();
		_decomposeVlaEndEffectorSetpoint();
	}

	// Publish trajectory setpoints for wheel loader
	_publishTrajectorySetpoints();

	return ret;
}

void FlightTaskAutoVLAEndEffector::_processVlaEndEffectorTrajectory()
{
	// Extract bucket target position from VLA end effector trajectory
	// VLA provides bucket position in world coordinates
	Vector3f bucket_target(_vla_end_effector_trajectory.bucket_position_x,
			       _vla_end_effector_trajectory.bucket_position_y,
			       _vla_end_effector_trajectory.bucket_position_z);

	// For now, use bucket XY position as chassis target position
	// Z-component will be handled by boom/bucket kinematics
	_position_setpoint(0) = bucket_target(0);
	_position_setpoint(1) = bucket_target(1);
	_position_setpoint(2) = _position(2); // Maintain current Z position (ground level)

	// Set velocity based on trajectory constraints
	float max_vel = math::constrain(_vla_end_effector_trajectory.max_velocity, 0.1f, _param_autovla_ee_max_vel.get());

	// Calculate desired velocity towards target
	Vector2f pos_error(bucket_target(0) - _position(0), bucket_target(1) - _position(1));
	float distance = pos_error.norm();

	if (distance > MIN_POSITION_THRESHOLD) { // Threshold to prevent normalization of very small vectors
		Vector2f velocity_dir = pos_error.normalized();
		float desired_speed = math::min(distance, max_vel);
		_velocity_setpoint(0) = velocity_dir(0) * desired_speed;
		_velocity_setpoint(1) = velocity_dir(1) * desired_speed;
	} else {
		_velocity_setpoint(0) = 0.0f;
		_velocity_setpoint(1) = 0.0f;
	}

	_velocity_setpoint(2) = 0.0f;

	// Set yaw from VLA end effector trajectory
	_yaw_setpoint = _vla_end_effector_trajectory.bucket_orientation_yaw;
	_yawspeed_setpoint = 0.0f; // Could be derived from trajectory if needed
}

void FlightTaskAutoVLAEndEffector::_decomposeVlaEndEffectorSetpoint()
{
	// Decompose VLA end effector bucket position into chassis position and end effector angles
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
	Vector3f bucket_pos(_vla_end_effector_trajectory.bucket_position_x,
			    _vla_end_effector_trajectory.bucket_position_y,
			    _vla_end_effector_trajectory.bucket_position_z);

	// Calculate relative bucket position from chassis
	Vector2f bucket_relative(bucket_pos(0) - _position(0), bucket_pos(1) - _position(1));
	float bucket_reach = bucket_relative.norm();
	float bucket_height = bucket_pos(2);

	// Simple inverse kinematics: assume boom angle based on reach and height
	// This is simplified - real implementation would use proper IK
	float boom_reach = _param_autovla_ee_boom_reach.get();
	float boom_angle = atan2f(bucket_height, math::min(bucket_reach, boom_reach));

	// Bucket angle from VLA end effector orientation (pitch component)
	float bucket_angle = _vla_end_effector_trajectory.bucket_orientation_pitch;

	_end_effector_setpoint.timestamp = hrt_absolute_time();
	_end_effector_setpoint.boom_angle = boom_angle;
	_end_effector_setpoint.bucket_angle = bucket_angle;
	_end_effector_setpoint.boom_angle_rate = 0.0f; // Could be derived from trajectory
	_end_effector_setpoint.bucket_angle_rate = 0.0f;
	_end_effector_setpoint.valid = true;
	_end_effector_setpoint.priority = 100;
	_end_effector_setpoint.hold_position = false;
	_end_effector_setpoint.emergency_stop = _vla_end_effector_trajectory.emergency_stop;
	_end_effector_setpoint.sequence_id = _vla_end_effector_trajectory.sequence_id;
	_end_effector_setpoint.sequence_complete = _vla_end_effector_trajectory.sequence_complete;
	_end_effector_setpoint.confidence_score = _vla_end_effector_trajectory.confidence_score;
}

void FlightTaskAutoVLAEndEffector::_publishTrajectorySetpoints()
{
	// Publish chassis trajectory setpoint
	_pub_chassis_setpoint.publish(_chassis_setpoint);

	// Publish end effector trajectory setpoint
	_pub_end_effector_setpoint.publish(_end_effector_setpoint);
}

bool FlightTaskAutoVLAEndEffector::_isVlaEndEffectorTrajectoryValid() const
{
	if (!_vla_end_effector_trajectory.valid_output || !_vla_end_effector_trajectory.enable_trajectory) {
		return false;
	}

	// Check if trajectory is recent
	if (_vla_end_effector_trajectory.timestamp > 0 && hrt_elapsed_time(&_vla_end_effector_trajectory.timestamp) < VLA_EE_TIMEOUT) {
		return true;
	}

	return false;
}
