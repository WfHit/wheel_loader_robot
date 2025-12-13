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
 * @file ModeManualWheelLoader.cpp
 *
 * Manual mode implementation for wheel loader robot
 */

#include "ModeManualWheelLoader.hpp"
#include <mathlib/mathlib.h>

ModeManualWheelLoader::ModeManualWheelLoader() :
	Mode()
{
	// Configure smoothing with default values
	// These will be updated from parameters in activate()
	_velocity_smoothing.setMaxJerk(5.0f);
	_velocity_smoothing.setMaxAccel(2.0f);
	_velocity_smoothing.setMaxVel(2.0f);

	_steering_smoothing.setMaxJerk(5.0f);
	_steering_smoothing.setMaxAccel(2.0f);
	_steering_smoothing.setMaxVel(1.0f);

	_bucket_height_smoothing.setMaxJerk(3.0f);
	_bucket_height_smoothing.setMaxAccel(1.0f);
	_bucket_height_smoothing.setMaxVel(0.5f);

	_tilt_smoothing.setMaxJerk(3.0f);
	_tilt_smoothing.setMaxAccel(1.0f);
	_tilt_smoothing.setMaxVel(1.0f);
}

bool ModeManualWheelLoader::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = Mode::activate(last_setpoint);

	// Reset manual control state
	_manual_control = {};
	_last_manual_control_time = 0;

	// Initialize setpoints
	_chassis_setpoint = {};
	_boom_setpoint = {};
	_tilt_setpoint = {};

	// Initialize integrated positions with current state
	// In a real implementation, these would come from feedback
	_current_bucket_height = 0.5f;  // Default mid-height
	_current_tilt_angle = 0.0f;
	_current_steering_angle = 0.0f;

	// Reset smoothing states
	_velocity_smoothing.reset(0.0f, 0.0f, 0.0f);
	_steering_smoothing.reset(0.0f, 0.0f, _current_steering_angle);
	_bucket_height_smoothing.reset(0.0f, 0.0f, _current_bucket_height);
	_tilt_smoothing.reset(0.0f, 0.0f, _current_tilt_angle);

	// Update smoothing limits from parameters
	_velocity_smoothing.setMaxJerk(_param_max_jerk.get());
	_velocity_smoothing.setMaxAccel(_param_max_accel.get());
	_velocity_smoothing.setMaxVel(_param_max_velocity.get());

	_steering_smoothing.setMaxJerk(_param_max_jerk.get());
	_steering_smoothing.setMaxAccel(_param_max_accel.get());
	_steering_smoothing.setMaxVel(_param_max_steering_rate.get());

	_bucket_height_smoothing.setMaxJerk(_param_max_jerk.get());
	_bucket_height_smoothing.setMaxAccel(_param_max_accel.get());
	_bucket_height_smoothing.setMaxVel(_param_bucket_height_vel.get());

	_tilt_smoothing.setMaxJerk(_param_max_jerk.get());
	_tilt_smoothing.setMaxAccel(_param_max_accel.get());
	_tilt_smoothing.setMaxVel(_param_tilt_vel.get());

	return ret;
}

void ModeManualWheelLoader::reActivate()
{
	Mode::reActivate();
}

bool ModeManualWheelLoader::updateInitialize()
{
	bool ret = Mode::updateInitialize();

	// Update manual control subscription
	if (_manual_control_sub.update(&_manual_control)) {
		if (_manual_control.valid) {
			_last_manual_control_time = hrt_absolute_time();
		}
	}

	return ret;
}

bool ModeManualWheelLoader::update()
{
	bool ret = Mode::update();

	// Calculate dt
	float dt = 0.02f; // 50Hz default
	if (_time_stamp_last_loop > 0) {
		dt = math::constrain((hrt_absolute_time() - _time_stamp_last_loop) * 1e-6f, 0.001f, 0.1f);
	}

	if (!isManualControlValid()) {
		// No valid manual control - publish hold setpoints
		publishHoldSetpoints();
		return ret;
	}

	// Process stick inputs
	processStickInputs(dt);

	// Publish control setpoints
	publishChassisSetpoint();
	publishBoomSetpoint();
	publishTiltSetpoint();

	return ret;
}

bool ModeManualWheelLoader::isManualControlValid() const
{
	if (!_manual_control.valid) {
		return false;
	}

	if (_last_manual_control_time == 0) {
		return false;
	}

	if (hrt_elapsed_time(&_last_manual_control_time) > MANUAL_CONTROL_TIMEOUT) {
		return false;
	}

	return true;
}

float ModeManualWheelLoader::applyDeadzone(float input, float deadzone) const
{
	if (fabsf(input) < deadzone) {
		return 0.0f;
	}

	// Scale remaining range to [0, 1]
	float sign = (input > 0.0f) ? 1.0f : -1.0f;
	return sign * (fabsf(input) - deadzone) / (1.0f - deadzone);
}

void ModeManualWheelLoader::processStickInputs(float dt)
{
	// === Chassis Control ===
	// Right stick Y (pitch): Forward/backward velocity
	// Positive pitch = push forward = positive velocity
	float velocity_cmd = applyDeadzone(_manual_control.pitch, STICK_DEADZONE);
	float target_velocity = velocity_cmd * _param_max_velocity.get();

	// Smooth velocity command
	_velocity_smoothing.updateDurations(target_velocity);
	_velocity_smoothing.updateTraj(dt);

	// Right stick X (roll): Steering angle (direct control)
	// Positive roll = push right = turn right = positive steering
	float steering_cmd = applyDeadzone(_manual_control.roll, STICK_DEADZONE);
	float target_steering = steering_cmd * _param_max_steering.get();

	// Smooth steering
	_steering_smoothing.updateDurations(target_steering);
	_steering_smoothing.updateTraj(dt);
	_current_steering_angle = _steering_smoothing.getCurrentPosition();

	// Fill chassis setpoint
	_chassis_setpoint.timestamp = hrt_absolute_time();
	_chassis_setpoint.x = NAN;  // No position control in manual mode
	_chassis_setpoint.y = NAN;
	_chassis_setpoint.heading = NAN;
	_chassis_setpoint.vx = _velocity_smoothing.getCurrentVelocity();  // Forward velocity
	_chassis_setpoint.vy = 0.0f;  // No lateral velocity for articulated steering
	_chassis_setpoint.yaw_rate = NAN;  // Derived from steering
	_chassis_setpoint.ax = _velocity_smoothing.getCurrentAcceleration();
	_chassis_setpoint.ay = 0.0f;
	_chassis_setpoint.yaw_acceleration = NAN;
	_chassis_setpoint.steering_angle = _current_steering_angle;
	_chassis_setpoint.steering_rate = _steering_smoothing.getCurrentVelocity();
	_chassis_setpoint.target_slip_rate = 0.0f;
	_chassis_setpoint.sequence = MANUAL_SEQUENCE;
	_chassis_setpoint.valid = true;

	// === Boom Control ===
	// Left stick Y (throttle): Bucket height velocity
	// Note: throttle is [-1, 1] where -1 is down, 1 is up
	float height_vel_cmd = applyDeadzone(-_manual_control.throttle, STICK_DEADZONE);
	float target_height_vel = height_vel_cmd * _param_bucket_height_vel.get();

	// Integrate height with velocity command, but limit to bounds
	float new_height = _current_bucket_height + target_height_vel * dt;
	new_height = math::constrain(new_height, _param_bucket_height_min.get(), _param_bucket_height_max.get());

	// Smooth bucket height
	_bucket_height_smoothing.updateDurations(new_height);
	_bucket_height_smoothing.updateTraj(dt);
	_current_bucket_height = _bucket_height_smoothing.getCurrentPosition();

	// Fill boom setpoint
	_boom_setpoint.timestamp = hrt_absolute_time();
	_boom_setpoint.bucket_height = _current_bucket_height;
	_boom_setpoint.bucket_height_velocity = _bucket_height_smoothing.getCurrentVelocity();
	_boom_setpoint.sequence = MANUAL_SEQUENCE;
	_boom_setpoint.valid = true;

	// === Tilt Control ===
	// Left stick X (yaw): Tilt angle velocity
	// Positive yaw = push right = curl bucket = positive tilt
	float tilt_vel_cmd = applyDeadzone(_manual_control.yaw, STICK_DEADZONE);
	float target_tilt_vel = tilt_vel_cmd * _param_tilt_vel.get();

	// Integrate tilt with velocity command, but limit to bounds
	float new_tilt = _current_tilt_angle + target_tilt_vel * dt;
	new_tilt = math::constrain(new_tilt, _param_tilt_min.get(), _param_tilt_max.get());

	// Smooth tilt angle
	_tilt_smoothing.updateDurations(new_tilt);
	_tilt_smoothing.updateTraj(dt);
	_current_tilt_angle = _tilt_smoothing.getCurrentPosition();

	// Fill tilt setpoint
	_tilt_setpoint.timestamp = hrt_absolute_time();
	_tilt_setpoint.angle = _current_tilt_angle;
	_tilt_setpoint.angular_velocity = _tilt_smoothing.getCurrentVelocity();
	_tilt_setpoint.angular_acceleration = _tilt_smoothing.getCurrentAcceleration();
	_tilt_setpoint.sequence = MANUAL_SEQUENCE;
	_tilt_setpoint.valid = true;
}

void ModeManualWheelLoader::publishChassisSetpoint()
{
	_pub_chassis.publish(_chassis_setpoint);
}

void ModeManualWheelLoader::publishBoomSetpoint()
{
	_pub_boom.publish(_boom_setpoint);
}

void ModeManualWheelLoader::publishTiltSetpoint()
{
	_pub_tilt.publish(_tilt_setpoint);
}

void ModeManualWheelLoader::publishHoldSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Publish chassis hold (zero velocity, maintain steering)
	chassis_control_setpoint_s chassis_hold{};
	chassis_hold.timestamp = now;
	chassis_hold.x = NAN;
	chassis_hold.y = NAN;
	chassis_hold.heading = NAN;
	chassis_hold.vx = 0.0f;
	chassis_hold.vy = 0.0f;
	chassis_hold.yaw_rate = 0.0f;
	chassis_hold.ax = 0.0f;
	chassis_hold.ay = 0.0f;
	chassis_hold.yaw_acceleration = 0.0f;
	chassis_hold.steering_angle = _current_steering_angle;
	chassis_hold.steering_rate = 0.0f;
	chassis_hold.target_slip_rate = 0.0f;
	chassis_hold.sequence = MANUAL_SEQUENCE;
	chassis_hold.valid = true;
	_pub_chassis.publish(chassis_hold);

	// Publish boom hold (maintain height)
	boom_control_setpoint_s boom_hold{};
	boom_hold.timestamp = now;
	boom_hold.bucket_height = _current_bucket_height;
	boom_hold.bucket_height_velocity = 0.0f;
	boom_hold.sequence = MANUAL_SEQUENCE;
	boom_hold.valid = true;
	_pub_boom.publish(boom_hold);

	// Publish tilt hold (maintain angle)
	tilt_control_setpoint_s tilt_hold{};
	tilt_hold.timestamp = now;
	tilt_hold.angle = _current_tilt_angle;
	tilt_hold.angular_velocity = 0.0f;
	tilt_hold.angular_acceleration = 0.0f;
	tilt_hold.sequence = MANUAL_SEQUENCE;
	tilt_hold.valid = true;
	_pub_tilt.publish(tilt_hold);
}
