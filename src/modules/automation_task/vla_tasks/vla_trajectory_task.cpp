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
 * @file vla_trajectory_task.cpp
 *
 * VLA Trajectory Task implementation for Automation module
 * Publishes VlaSetpointTriplet at ~10Hz for ModeVLA interpolation
 *
 * @author PX4 Development Team
 */

#include "vla_trajectory_task.h"
#include "../automation_task.hpp"
#include <lib/geo/geo.h>

using namespace matrix;

VlaTrajectoryTask::VlaTrajectoryTask(Automation *automation) :
	TaskBlock(automation, mode_status_s::OPERATION_MODE_AUTO_VLA),
	ModuleParams(automation)
{
}

void VlaTrajectoryTask::initialize()
{
	// Reset state
	_vla_trajectory = {};
	_last_vla_sequence = 0;
	_processor_state = ProcessorState::IDLE;
	_smoothed.num_points = 0;
	_smoothed.current_index = 0;
}

void VlaTrajectoryTask::on_inactive()
{
	// Nothing to do when inactive
}

void VlaTrajectoryTask::on_activation()
{
	// Reset on activation
	initialize();

	// Initialize position setpoint triplet to current position
	position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

	_automation->reset_position_setpoint(pos_sp_triplet->previous);
	_automation->reset_position_setpoint(pos_sp_triplet->current);
	_automation->reset_position_setpoint(pos_sp_triplet->next);

	// Set current position as hold point
	if (_automation->get_land_detected()->landed) {
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

	} else {
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		pos_sp_triplet->current.lat = _automation->get_global_position()->lat;
		pos_sp_triplet->current.lon = _automation->get_global_position()->lon;
		pos_sp_triplet->current.alt = _automation->get_global_position()->alt;
		pos_sp_triplet->current.yaw = _automation->get_local_position()->heading;
		pos_sp_triplet->current.valid = true;
	}

	_automation->set_position_setpoint_triplet_updated();
	_automation->reset_cruising_speed();

	// Initialize vehicle state
	updateVehicleState();

	// Initialize last setpoint with current state
	_last_setpoint = {};
	_last_setpoint.chassis_x = _vehicle_state.x;
	_last_setpoint.chassis_y = _vehicle_state.y;
	_last_setpoint.chassis_heading = _vehicle_state.heading;
	_last_setpoint.chassis_vx = 0.0f;
	_last_setpoint.chassis_vy = 0.0f;
	_last_setpoint.chassis_yaw_rate = 0.0f;
	_last_setpoint.bucket_height = 0.5f;  // Default height
	_last_setpoint.tilt_angle = 0.0f;
	_last_setpoint.valid = true;
}

void VlaTrajectoryTask::on_active()
{
	// Update vehicle state
	updateVehicleState();

	// Check for new VLA trajectory
	if (_sub_vla_trajectory.updated()) {
		_sub_vla_trajectory.copy(&_vla_trajectory);

		// Check if this is a new trajectory
		if (_vla_trajectory.sequence_id != _last_vla_sequence &&
		    _vla_trajectory.valid &&
		    _vla_trajectory.num_steps > 0) {

			processNewVlaTrajectory();
			_last_vla_sequence = _vla_trajectory.sequence_id;
		}
	}

	// Publish VLA setpoint triplet
	if (_processor_state == ProcessorState::EXECUTING) {
		publishVlaSetpointTriplet();

	} else {
		publishHoldTriplet();
	}
}

void VlaTrajectoryTask::updateVehicleState()
{
	const vehicle_local_position_s *local_pos = _automation->get_local_position();

	if (local_pos->xy_valid && local_pos->v_xy_valid) {
		_vehicle_state.x = local_pos->x;
		_vehicle_state.y = local_pos->y;
		_vehicle_state.vx = local_pos->vx;
		_vehicle_state.vy = local_pos->vy;
		_vehicle_state.heading = local_pos->heading;
		_vehicle_state.valid = true;
	}
}

bool VlaTrajectoryTask::isVlaTrajectoryValid() const
{
	if (!_vla_trajectory.valid || _vla_trajectory.num_steps == 0) {
		return false;
	}

	if (_vla_trajectory.timestamp > 0 &&
	    hrt_elapsed_time(&_vla_trajectory.timestamp) < VLA_TIMEOUT) {
		return true;
	}

	return false;
}

bool VlaTrajectoryTask::processNewVlaTrajectory()
{
	if (!_vehicle_state.valid) {
		return false;
	}

	// Step 1: Decompose 7-DOF trajectory into channels
	decomposeTrajectory();

	// Step 2: Compute synchronized timing
	computeSynchronizedTiming();

	// Step 3: Apply smoothing (output at ~10Hz waypoint rate)
	smoothTrajectories();

	// Step 4: Reset execution state
	_trajectory_start_time = hrt_absolute_time();
	_smoothed.current_index = 0;
	_current_trajectory_sequence = _vla_trajectory.sequence_id;
	_processor_state = ProcessorState::EXECUTING;

	return true;
}

void VlaTrajectoryTask::decomposeTrajectory()
{
	const int num_steps = math::min((int)_vla_trajectory.num_steps, MAX_TRAJECTORY_POINTS);
	_decomposed.num_points = num_steps;

	// Start from current vehicle state
	float x = _vehicle_state.x;
	float y = _vehicle_state.y;
	float heading = _vehicle_state.heading;

	for (int i = 0; i < num_steps; i++) {
		// Transform body-frame deltas to world frame
		const float cos_h = cosf(heading);
		const float sin_h = sinf(heading);

		const float dx_body = _vla_trajectory.delta_x[i];
		const float dy_body = _vla_trajectory.delta_y[i];

		// Rotate body delta to world frame
		const float dx_world = dx_body * cos_h - dy_body * sin_h;
		const float dy_world = dx_body * sin_h + dy_body * cos_h;

		// Accumulate position
		x += dx_world;
		y += dy_world;
		heading += _vla_trajectory.delta_heading[i];

		// Normalize heading to [-π, π]
		heading = matrix::wrap_pi(heading);

		// Store world-frame trajectory
		_decomposed.x[i] = x;
		_decomposed.y[i] = y;
		_decomposed.heading[i] = heading;
		_decomposed.steering[i] = math::constrain(_vla_trajectory.steering_angle[i],
					  -_param_vtp_str_ang_max.get(),
					  _param_vtp_str_ang_max.get());

		// Pass bucket_height through directly (IK done in ModeVLA)
		_decomposed.bucket_height[i] = math::constrain(_vla_trajectory.bucket_height[i],
						_param_vtp_bkt_hgt_min.get(),
						_param_vtp_bkt_hgt_max.get());

		// Tilt angle with constraints
		_decomposed.tilt_angle[i] = math::constrain(_vla_trajectory.tilt_angle[i],
					    _param_vtp_tilt_ang_min.get(),
					    _param_vtp_tilt_ang_max.get());

		// Slip rate passthrough
		_decomposed.slip_rate[i] = math::constrain(_vla_trajectory.target_slip_rate[i], 0.0f, 1.0f);
	}
}

void VlaTrajectoryTask::computeSynchronizedTiming()
{
	if (_decomposed.num_points < 2) {
		_timing.scale_factor = 1.0f;
		return;
	}

	// Compute minimum time for chassis (position)
	float chassis_max_time = 0.0f;

	for (int i = 1; i < _decomposed.num_points; i++) {
		float dx = _decomposed.x[i] - _decomposed.x[i - 1];
		float dy = _decomposed.y[i] - _decomposed.y[i - 1];
		float dist = sqrtf(dx * dx + dy * dy);

		// Time limited by max velocity
		float time_vel = dist / _param_vtp_chs_vel_max.get();

		// Time limited by acceleration (simplified: assume start/stop each segment)
		float time_acc = 2.0f * sqrtf(dist / _param_vtp_chs_acc_max.get());

		float segment_time = fmaxf(time_vel, time_acc);
		chassis_max_time = fmaxf(chassis_max_time, segment_time);
	}

	_timing.chassis_time = chassis_max_time * _decomposed.num_points;

	// Compute minimum time for bucket height
	float boom_max_time = 0.0f;

	for (int i = 1; i < _decomposed.num_points; i++) {
		float d_height = fabsf(_decomposed.bucket_height[i] - _decomposed.bucket_height[i - 1]);
		float time_vel = d_height / _param_vtp_bkt_vel_max.get();
		float time_acc = 2.0f * sqrtf(d_height / _param_vtp_bkt_acc_max.get());
		boom_max_time = fmaxf(boom_max_time, fmaxf(time_vel, time_acc));
	}

	_timing.boom_time = boom_max_time * _decomposed.num_points;

	// Compute minimum time for tilt
	float tilt_max_time = 0.0f;

	for (int i = 1; i < _decomposed.num_points; i++) {
		float d_angle = fabsf(_decomposed.tilt_angle[i] - _decomposed.tilt_angle[i - 1]);
		float time_vel = d_angle / _param_vtp_tilt_vel_max.get();
		float time_acc = 2.0f * sqrtf(d_angle / _param_vtp_tilt_acc_max.get());
		tilt_max_time = fmaxf(tilt_max_time, fmaxf(time_vel, time_acc));
	}

	_timing.tilt_time = tilt_max_time * _decomposed.num_points;

	// Synchronized time is maximum (slowest channel)
	_timing.sync_time = fmaxf(_timing.chassis_time,
				  fmaxf(_timing.boom_time, _timing.tilt_time));

	// Add safety margin
	_timing.sync_time += _param_vtp_sync_margin.get();

	// VLA trajectory nominal duration
	float vla_horizon = _decomposed.num_points * VLA_DT;

	// Scale factor (>= 1.0 means we need to slow down)
	_timing.scale_factor = fmaxf(1.0f, _timing.sync_time / vla_horizon);
}

void VlaTrajectoryTask::smoothTrajectories()
{
	// Output smoothed waypoints at the same rate as VLA input (no interpolation here)
	// ModeVLA will handle 50Hz interpolation
	_smoothed.num_points = math::min(_decomposed.num_points, SMOOTHED_BUFFER_SIZE);

	// Scale constraints by time scale factor for synchronized motion
	const float scale = _timing.scale_factor;
	const float scale2 = scale * scale;
	const float scale3 = scale2 * scale;

	// Smooth chassis X
	smoothSingleDof(
		_decomposed.x,
		_decomposed.num_points,
		_smoothed.chassis_x_pos,
		_smoothed.chassis_x_vel,
		_smoothed.chassis_x_acc,
		_smoothed.num_points,
		_param_vtp_chs_vel_max.get() / scale,
		_param_vtp_chs_acc_max.get() / scale2,
		_param_vtp_chs_jerk_max.get() / scale3,
		scale,
		_vehicle_state.x,
		_vehicle_state.vx
	);

	// Smooth chassis Y
	int dummy;
	smoothSingleDof(
		_decomposed.y,
		_decomposed.num_points,
		_smoothed.chassis_y_pos,
		_smoothed.chassis_y_vel,
		_smoothed.chassis_y_acc,
		dummy,
		_param_vtp_chs_vel_max.get() / scale,
		_param_vtp_chs_acc_max.get() / scale2,
		_param_vtp_chs_jerk_max.get() / scale3,
		scale,
		_vehicle_state.y,
		_vehicle_state.vy
	);

	// Smooth heading
	smoothSingleDof(
		_decomposed.heading,
		_decomposed.num_points,
		_smoothed.chassis_heading,
		_smoothed.chassis_yaw_rate,
		_smoothed.chassis_yaw_acc,
		dummy,
		_param_vtp_chs_yaw_max.get() / scale,
		_param_vtp_chs_yacc_max.get() / scale2,
		_param_vtp_chs_jerk_max.get() / scale3,
		scale,
		_vehicle_state.heading,
		_vehicle_state.yaw_rate
	);

	// Smooth steering
	smoothSingleDof(
		_decomposed.steering,
		_decomposed.num_points,
		_smoothed.chassis_steering,
		_smoothed.chassis_steering_rate,
		nullptr,
		dummy,
		_param_vtp_str_rate_max.get() / scale,
		_param_vtp_str_rate_max.get() * 2.0f / scale2,
		_param_vtp_chs_jerk_max.get() / scale3,
		scale,
		_decomposed.steering[0],
		0.0f
	);

	// Smooth bucket height (IK done in ModeVLA)
	smoothSingleDof(
		_decomposed.bucket_height,
		_decomposed.num_points,
		_smoothed.bucket_height,
		_smoothed.bucket_height_vel,
		nullptr,
		dummy,
		_param_vtp_bkt_vel_max.get() / scale,
		_param_vtp_bkt_acc_max.get() / scale2,
		_param_vtp_bkt_jerk_max.get() / scale3,
		scale,
		_decomposed.bucket_height[0],
		0.0f
	);

	// Smooth tilt
	smoothSingleDof(
		_decomposed.tilt_angle,
		_decomposed.num_points,
		_smoothed.tilt_pos,
		_smoothed.tilt_vel,
		_smoothed.tilt_acc,
		dummy,
		_param_vtp_tilt_vel_max.get() / scale,
		_param_vtp_tilt_acc_max.get() / scale2,
		_param_vtp_tilt_jerk_max.get() / scale3,
		scale,
		_decomposed.tilt_angle[0],
		0.0f
	);

	// Copy slip rate (simple passthrough, no dynamics)
	for (int i = 0; i < _smoothed.num_points; i++) {
		_smoothed.slip_rate[i] = _decomposed.slip_rate[math::min(i, _decomposed.num_points - 1)];
	}
}

void VlaTrajectoryTask::smoothSingleDof(
	const float *input_points,
	int num_input,
	float *output_pos,
	float *output_vel,
	float *output_acc,
	int &num_output,
	float max_vel,
	float max_acc,
	float max_jerk,
	float time_scale,
	float initial_pos,
	float initial_vel)
{
	if (num_input < 1) {
		num_output = 0;
		return;
	}

	// Output at waypoint rate (no 50Hz interpolation - ModeVLA handles that)
	num_output = math::min(num_input, SMOOTHED_BUFFER_SIZE);

	// State for jerk-limited trajectory
	float pos = initial_pos;
	float vel = initial_vel;
	float acc = 0.0f;

	const float segment_time = VLA_DT * time_scale;

	for (int i = 0; i < num_output; i++) {
		float target = input_points[math::min(i, num_input - 1)];
		float time_remaining = segment_time;

		if (time_remaining > 0.001f) {
			// Position error to target
			float pos_error = target - pos;

			// Desired velocity to reach target
			float desired_vel = pos_error / time_remaining;
			desired_vel = math::constrain(desired_vel, -max_vel, max_vel);

			// Desired acceleration
			float desired_acc = (desired_vel - vel) / segment_time;
			desired_acc = math::constrain(desired_acc, -max_acc, max_acc);

			// Apply jerk limit
			float jerk = (desired_acc - acc) / segment_time;
			jerk = math::constrain(jerk, -max_jerk, max_jerk);

			// Update state
			acc += jerk * segment_time;
			acc = math::constrain(acc, -max_acc, max_acc);

			vel += acc * segment_time;
			vel = math::constrain(vel, -max_vel, max_vel);

			pos += vel * segment_time;
		}

		// Store output
		output_pos[i] = pos;

		if (output_vel) {
			output_vel[i] = vel;
		}

		if (output_acc) {
			output_acc[i] = acc;
		}
	}
}

vla_setpoint_s VlaTrajectoryTask::buildSetpointFromIndex(int idx)
{
	vla_setpoint_s sp{};
	sp.timestamp = hrt_absolute_time();

	if (idx < 0 || idx >= _smoothed.num_points) {
		// Return invalid setpoint
		sp.valid = false;
		return sp;
	}

	// Chassis state
	sp.chassis_x = _smoothed.chassis_x_pos[idx];
	sp.chassis_y = _smoothed.chassis_y_pos[idx];
	sp.chassis_heading = _smoothed.chassis_heading[idx];
	sp.chassis_vx = _smoothed.chassis_x_vel[idx];
	sp.chassis_vy = _smoothed.chassis_y_vel[idx];
	sp.chassis_yaw_rate = _smoothed.chassis_yaw_rate[idx];
	sp.chassis_ax = _smoothed.chassis_x_acc[idx];
	sp.chassis_ay = _smoothed.chassis_y_acc[idx];
	sp.chassis_yaw_acc = _smoothed.chassis_yaw_acc[idx];
	sp.steering_angle = _smoothed.chassis_steering[idx];
	sp.steering_rate = _smoothed.chassis_steering_rate[idx];
	sp.target_slip_rate = _smoothed.slip_rate[idx];

	// Bucket height (IK done in ModeVLA)
	sp.bucket_height = _smoothed.bucket_height[idx];
	sp.bucket_height_velocity = _smoothed.bucket_height_vel[idx];

	// Tilt state
	sp.tilt_angle = _smoothed.tilt_pos[idx];
	sp.tilt_angular_velocity = _smoothed.tilt_vel[idx];
	sp.tilt_angular_acceleration = _smoothed.tilt_acc[idx];

	// Progress
	sp.progress = (float)idx / (float)(_smoothed.num_points - 1);
	sp.valid = true;

	return sp;
}

void VlaTrajectoryTask::publishVlaSetpointTriplet()
{
	const int idx = _smoothed.current_index;

	if (idx >= _smoothed.num_points) {
		// Trajectory complete, switch to hold
		_processor_state = ProcessorState::IDLE;
		publishHoldTriplet();
		return;
	}

	vla_setpoint_triplet_s triplet{};
	triplet.timestamp = hrt_absolute_time();

	// Build triplet: previous, current, next
	triplet.previous = buildSetpointFromIndex(idx - 1);
	triplet.current = buildSetpointFromIndex(idx);
	triplet.next = buildSetpointFromIndex(idx + 1);

	// If previous is invalid, use current
	if (!triplet.previous.valid) {
		triplet.previous = triplet.current;
	}

	// If next is invalid, use current
	if (!triplet.next.valid) {
		triplet.next = triplet.current;
	}

	triplet.sequence = _current_trajectory_sequence;
	triplet.vla_confidence = _vla_trajectory.confidence;
	triplet.valid = true;

	_pub_vla_triplet.publish(triplet);

	// Save last setpoint for hold mode
	_last_setpoint = triplet.current;

	// Advance index for next cycle (publish at ~10Hz)
	_smoothed.current_index++;
}

void VlaTrajectoryTask::publishHoldTriplet()
{
	vla_setpoint_triplet_s triplet{};
	triplet.timestamp = hrt_absolute_time();

	// Use last setpoint with zero velocities
	vla_setpoint_s hold_sp = _last_setpoint;
	hold_sp.timestamp = hrt_absolute_time();
	hold_sp.chassis_vx = 0.0f;
	hold_sp.chassis_vy = 0.0f;
	hold_sp.chassis_yaw_rate = 0.0f;
	hold_sp.chassis_ax = 0.0f;
	hold_sp.chassis_ay = 0.0f;
	hold_sp.chassis_yaw_acc = 0.0f;
	hold_sp.steering_rate = 0.0f;
	hold_sp.bucket_height_velocity = 0.0f;
	hold_sp.tilt_angular_velocity = 0.0f;
	hold_sp.tilt_angular_acceleration = 0.0f;
	hold_sp.target_slip_rate = 0.0f;
	hold_sp.valid = true;

	triplet.previous = hold_sp;
	triplet.current = hold_sp;
	triplet.next = hold_sp;
	triplet.sequence = _current_trajectory_sequence;
	triplet.vla_confidence = 0.0f;
	triplet.valid = true;

	_pub_vla_triplet.publish(triplet);
}
