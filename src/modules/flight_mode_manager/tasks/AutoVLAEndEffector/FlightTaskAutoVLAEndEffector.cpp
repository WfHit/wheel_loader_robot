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

	// Reset VLA end effector setpoint triplet
	_vla_setpoint_triplet = {};
	_last_vla_setpoint_update = 0;

	// Reset setpoints
	_chassis_setpoint = {};
	_boom_setpoint = {};
	_bucket_setpoint = {};

	return ret;
}

void FlightTaskAutoVLAEndEffector::reActivate()
{
	FlightTask::reActivate();
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
		// Process VLA end effector setpoint triplet with motion planning
		_processVlaEndEffectorSetpointTriplet();
		_planChassisTrajectory();
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

	// For now, use bucket XY position as chassis target position
	// Z-component will be handled by boom/bucket kinematics
	_position_setpoint(0) = bucket_target(0);
	_position_setpoint(1) = bucket_target(1);
	_position_setpoint(2) = _position(2); // Maintain current Z position (ground level)

	// Set yaw from VLA end effector setpoint
	_yaw_setpoint = _vla_setpoint_triplet.current_bucket_yaw;
	_yawspeed_setpoint = 0.0f;
}

void FlightTaskAutoVLAEndEffector::_planChassisTrajectory()
{
	// Motion planning for chassis trajectory
	// Uses setpoint triplet (previous, current, next) for smooth trajectory generation

	// Set velocity based on trajectory constraints
	float max_vel = math::constrain(_vla_setpoint_triplet.max_velocity, 0.1f, _param_autovla_ee_max_vel.get());
	float max_acc = math::constrain(_vla_setpoint_triplet.max_acceleration, 0.1f, _param_autovla_ee_max_acc.get());

	// Calculate desired velocity towards target using motion planning
	Vector2f pos_error(_position_setpoint(0) - _position(0), _position_setpoint(1) - _position(1));
	float distance = pos_error.norm();

	if (distance > MIN_POSITION_THRESHOLD) {
		Vector2f velocity_dir = pos_error.normalized();
		
		// Apply velocity planning based on distance and constraints
		float desired_speed = math::min(distance, max_vel);
		
		// If we have a next setpoint, plan smoother trajectory
		if (_vla_setpoint_triplet.next_valid) {
			// Look-ahead planning: adjust speed based on upcoming trajectory
			Vector2f next_pos(_vla_setpoint_triplet.next_bucket_x, _vla_setpoint_triplet.next_bucket_y);
			Vector2f current_to_next = next_pos - Vector2f(_position_setpoint(0), _position_setpoint(1));
			float next_distance = current_to_next.norm();
			
			// Reduce speed if we need to make a sharp turn
			float direction_change = acosf(math::constrain(velocity_dir.dot(current_to_next.normalized()), -1.0f, 1.0f));
			if (direction_change > M_PI_F / 4.0f) {  // > 45 degrees
				desired_speed *= 0.7f;  // Reduce speed for turn
			}
		}
		
		_velocity_setpoint(0) = velocity_dir(0) * desired_speed;
		_velocity_setpoint(1) = velocity_dir(1) * desired_speed;
	} else {
		_velocity_setpoint(0) = 0.0f;
		_velocity_setpoint(1) = 0.0f;
	}

	_velocity_setpoint(2) = 0.0f;

	// Generate chassis trajectory setpoint with slip rates
	_chassis_setpoint.timestamp = hrt_absolute_time();
	_chassis_setpoint.x_position = _position_setpoint(0);
	_chassis_setpoint.y_position = _position_setpoint(1);
	_chassis_setpoint.yaw = _yaw_setpoint;
	_chassis_setpoint.x_velocity = _velocity_setpoint(0);
	_chassis_setpoint.y_velocity = _velocity_setpoint(1);
	_chassis_setpoint.z_velocity = _velocity_setpoint(2);
	_chassis_setpoint.yaw_rate = _yawspeed_setpoint;
	_chassis_setpoint.valid = true;
	
	// Note: Wheel slip rates would be set in chassis controller based on VLA setpoint
	// For now, we just ensure the setpoint structure is populated
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
