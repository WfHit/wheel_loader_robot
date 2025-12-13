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
 * @file ModeVLA.cpp
 *
 * VLA Mode implementation - receives triplet, interpolates to 50Hz, publishes control setpoints
 */

#include "ModeVLA.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

ModeVLA::ModeVLA() :
	Mode()
{
	// Configure position smoothing for XY
	_position_smoothing.setMaxJerkXY(DEFAULT_MAX_JERK);
	_position_smoothing.setMaxAccelerationXY(DEFAULT_MAX_ACCEL);
	_position_smoothing.setMaxVelocityXY(DEFAULT_MAX_VEL_XY);
	_position_smoothing.setCruiseSpeed(DEFAULT_MAX_VEL_XY);
	_position_smoothing.setHorizontalTrajectoryGain(1.0f);

	// Configure yaw smoothing
	_yaw_smoothing.setMaxJerk(DEFAULT_MAX_JERK);
	_yaw_smoothing.setMaxAccel(DEFAULT_MAX_ACCEL);
	_yaw_smoothing.setMaxVel(DEFAULT_MAX_VEL_YAW);

	// Configure bucket height smoothing
	_bucket_height_smoothing.setMaxJerk(DEFAULT_MAX_JERK);
	_bucket_height_smoothing.setMaxAccel(DEFAULT_MAX_ACCEL);
	_bucket_height_smoothing.setMaxVel(DEFAULT_MAX_VEL_HEIGHT);

	// Configure tilt smoothing
	_tilt_smoothing.setMaxJerk(DEFAULT_MAX_JERK);
	_tilt_smoothing.setMaxAccel(DEFAULT_MAX_ACCEL);
	_tilt_smoothing.setMaxVel(DEFAULT_MAX_VEL_TILT);
}

bool ModeVLA::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = Mode::activate(last_setpoint);

	// Reset state
	_triplet = {};
	_triplet_timestamp = 0;
	_interpolation_progress = 0.0f;
	_last_sequence = 0;

	// Initialize setpoints
	_chassis_setpoint = {};
	_boom_setpoint = {};
	_tilt_setpoint = {};

	// Initialize last setpoints with current position
	_last_chassis_setpoint = {};
	_last_chassis_setpoint.x = _position(0);
	_last_chassis_setpoint.y = _position(1);
	_last_chassis_setpoint.heading = _yaw;
	_last_chassis_setpoint.valid = true;

	_last_boom_setpoint = {};
	_last_boom_setpoint.bucket_height = 0.0f;
	_last_boom_setpoint.valid = true;

	_last_tilt_setpoint = {};
	_last_tilt_setpoint.angle = 0.0f;
	_last_tilt_setpoint.valid = true;

	// Reset smoothing states with current position
	Vector3f current_pos(_position(0), _position(1), 0.0f);
	_position_smoothing.reset(Vector3f(0.f, 0.f, 0.f), Vector3f(0.f, 0.f, 0.f), current_pos);
	_yaw_smoothing.reset(0.0f, 0.0f, _yaw);
	_bucket_height_smoothing.reset(0.0f, 0.0f, 0.0f);
	_tilt_smoothing.reset(0.0f, 0.0f, 0.0f);

	return ret;
}

void ModeVLA::reActivate()
{
	Mode::reActivate();

	// Reset interpolation progress
	_interpolation_progress = 0.0f;
}

bool ModeVLA::updateInitialize()
{
	bool ret = Mode::updateInitialize();

	// Update VLA setpoint triplet subscription
	if (_sub_vla_triplet.updated()) {
		_sub_vla_triplet.copy(&_triplet);

		// Check if this is a new triplet
		if (_triplet.sequence != _last_sequence) {
			_last_sequence = _triplet.sequence;
			_triplet_timestamp = hrt_absolute_time();
			_interpolation_progress = 0.0f;
		}
	}

	return ret;
}

bool ModeVLA::update()
{
	bool ret = Mode::update();

	if (!isVlaTripletValid()) {
		// No valid triplet - publish hold setpoints
		publishHoldSetpoints();
		return ret;
	}

	// Calculate dt using base class time tracking
	float dt = 0.02f; // 50Hz default
	if (_time_stamp_last > 0) {
		dt = math::constrain((hrt_absolute_time() - _time_stamp_last) * 1e-6f, 0.001f, 0.1f);
	}

	// Interpolate setpoints based on time progress
	interpolateSetpoints(dt);

	// Publish 50Hz control setpoints
	publishChassisSetpoint();
	publishBoomSetpoint();
	publishTiltSetpoint();

	return ret;
}

bool ModeVLA::isVlaTripletValid() const
{
	// Check if triplet is valid and not timed out
	if (!_triplet.valid) {
		return false;
	}

	if (_triplet_timestamp == 0) {
		return false;
	}

	if (hrt_elapsed_time(&_triplet_timestamp) > VLA_TRIPLET_TIMEOUT) {
		return false;
	}

	// Check if current setpoint is valid
	if (!_triplet.current.valid) {
		return false;
	}

	return true;
}

void ModeVLA::interpolateSetpoints(float dt)
{
	const vla_setpoint_s &curr = _triplet.current;
	const vla_setpoint_s &next = _triplet.next;

	// Build waypoints for position smoothing (prev, current, next)
	Vector3f waypoints[3];
	waypoints[0] = Vector3f(_triplet.previous.valid ? _triplet.previous.chassis_x : curr.chassis_x,
				_triplet.previous.valid ? _triplet.previous.chassis_y : curr.chassis_y,
				0.0f);
	waypoints[1] = Vector3f(curr.chassis_x, curr.chassis_y, 0.0f);
	waypoints[2] = Vector3f(next.valid ? next.chassis_x : curr.chassis_x,
				next.valid ? next.chassis_y : curr.chassis_y,
				0.0f);

	// Current position for smoothing
	Vector3f current_position(_position_smoothing.getCurrentPositionX(),
				  _position_smoothing.getCurrentPositionY(),
				  0.0f);

	// Feedforward velocity from triplet
	Vector3f feedforward_vel(curr.chassis_vx, curr.chassis_vy, 0.0f);

	// Generate smooth position setpoints
	PositionSmoothing::PositionSmoothingSetpoints pos_setpoints;
	_position_smoothing.generateSetpoints(current_position, waypoints, feedforward_vel, dt, false, pos_setpoints);

	// Generate smooth yaw setpoint
	float target_yaw = next.valid ? next.chassis_heading : curr.chassis_heading;
	// Handle yaw wrap-around
	float current_yaw = _yaw_smoothing.getCurrentPosition();
	float yaw_error = target_yaw - current_yaw;
	if (yaw_error > M_PI_F) { yaw_error -= 2.0f * M_PI_F; }
	if (yaw_error < -M_PI_F) { yaw_error += 2.0f * M_PI_F; }
	float unwrapped_target = current_yaw + yaw_error;
	_yaw_smoothing.updateDurations(unwrapped_target);
	_yaw_smoothing.updateTraj(dt);

	// Generate smooth bucket height setpoint
	float target_bucket_height = next.valid ? next.bucket_height : curr.bucket_height;
	_bucket_height_smoothing.updateDurations(target_bucket_height);
	_bucket_height_smoothing.updateTraj(dt);

	// Generate smooth tilt setpoint
	float target_tilt = next.valid ? next.tilt_angle : curr.tilt_angle;
	_tilt_smoothing.updateDurations(target_tilt);
	_tilt_smoothing.updateTraj(dt);

	// Fill chassis setpoint from smoothed values
	_chassis_setpoint.timestamp = hrt_absolute_time();
	_chassis_setpoint.x = pos_setpoints.position(0);
	_chassis_setpoint.y = pos_setpoints.position(1);
	_chassis_setpoint.vx = pos_setpoints.velocity(0);
	_chassis_setpoint.vy = pos_setpoints.velocity(1);
	_chassis_setpoint.ax = pos_setpoints.acceleration(0);
	_chassis_setpoint.ay = pos_setpoints.acceleration(1);

	// Yaw from smoothing
	_chassis_setpoint.heading = math::wrap_pi(_yaw_smoothing.getCurrentPosition());
	_chassis_setpoint.yaw_rate = _yaw_smoothing.getCurrentVelocity();
	_chassis_setpoint.yaw_acceleration = _yaw_smoothing.getCurrentAcceleration();

	// Steering - linear interpolation (no smoothing needed)
	float t = _interpolation_progress;
	_interpolation_progress += dt / TRIPLET_INTERVAL;
	_interpolation_progress = math::constrain(_interpolation_progress, 0.0f, 1.0f);

	if (next.valid) {
		_chassis_setpoint.steering_angle = curr.steering_angle + t * (next.steering_angle - curr.steering_angle);
		_chassis_setpoint.steering_rate = curr.steering_rate + t * (next.steering_rate - curr.steering_rate);
		_chassis_setpoint.target_slip_rate = curr.target_slip_rate + t * (next.target_slip_rate - curr.target_slip_rate);
	} else {
		_chassis_setpoint.steering_angle = curr.steering_angle;
		_chassis_setpoint.steering_rate = curr.steering_rate;
		_chassis_setpoint.target_slip_rate = curr.target_slip_rate;
	}

	// Fill boom setpoint from smoothed bucket height
	_boom_setpoint.timestamp = hrt_absolute_time();
	_boom_setpoint.bucket_height = _bucket_height_smoothing.getCurrentPosition();
	_boom_setpoint.bucket_height_velocity = _bucket_height_smoothing.getCurrentVelocity();

	// Fill tilt setpoint from smoothed tilt angle
	_tilt_setpoint.timestamp = hrt_absolute_time();
	_tilt_setpoint.angle = _tilt_smoothing.getCurrentPosition();
	_tilt_setpoint.angular_velocity = _tilt_smoothing.getCurrentVelocity();
	_tilt_setpoint.angular_acceleration = _tilt_smoothing.getCurrentAcceleration();

	// Set sequence and validity
	_chassis_setpoint.sequence = _triplet.sequence;
	_chassis_setpoint.valid = true;

	_boom_setpoint.sequence = _triplet.sequence;
	_boom_setpoint.valid = true;

	_tilt_setpoint.sequence = _triplet.sequence;
	_tilt_setpoint.valid = true;

	// Store as last valid setpoints
	_last_chassis_setpoint = _chassis_setpoint;
	_last_boom_setpoint = _boom_setpoint;
	_last_tilt_setpoint = _tilt_setpoint;
}

void ModeVLA::publishChassisSetpoint()
{
	_pub_chassis.publish(_chassis_setpoint);
}

void ModeVLA::publishBoomSetpoint()
{
	_pub_boom.publish(_boom_setpoint);
}

void ModeVLA::publishTiltSetpoint()
{
	_pub_tilt.publish(_tilt_setpoint);
}

void ModeVLA::publishHoldSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Publish last valid chassis setpoint with zero velocity
	chassis_control_setpoint_s chassis_hold = _last_chassis_setpoint;
	chassis_hold.timestamp = now;
	chassis_hold.vx = 0.0f;
	chassis_hold.vy = 0.0f;
	chassis_hold.yaw_rate = 0.0f;
	chassis_hold.ax = 0.0f;
	chassis_hold.ay = 0.0f;
	chassis_hold.yaw_acceleration = 0.0f;
	chassis_hold.steering_rate = 0.0f;
	_pub_chassis.publish(chassis_hold);

	// Publish last valid boom setpoint with zero velocity
	boom_control_setpoint_s boom_hold = _last_boom_setpoint;
	boom_hold.timestamp = now;
	boom_hold.bucket_height_velocity = 0.0f;
	_pub_boom.publish(boom_hold);

	// Publish last valid tilt setpoint with zero velocity
	tilt_control_setpoint_s tilt_hold = _last_tilt_setpoint;
	tilt_hold.timestamp = now;
	tilt_hold.angular_velocity = 0.0f;
	tilt_hold.angular_acceleration = 0.0f;
	_pub_tilt.publish(tilt_hold);
}
