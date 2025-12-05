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
 * @file autovla.cpp
 *
 * Navigator mode for autonomous VLA trajectory following
 *
 * @author PX4 Development Team
 */

#include "autovla.h"
#include "navigator.h"
#include <lib/geo/geo.h>

AutoVLA::AutoVLA(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_VLA),
	ModuleParams(navigator)
{
}

void AutoVLA::initialize()
{
	// Reset VLA trajectory
	_vla_trajectory = {};
	_last_vla_update = 0;
}

void AutoVLA::on_inactive()
{
	// Nothing to do when inactive
}

void AutoVLA::on_activation()
{
	// Reset on activation
	_vla_trajectory = {};
	_last_vla_update = 0;

	// Set initial position setpoint to current position
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Initialize position setpoint triplet
	_navigator->reset_position_setpoint(pos_sp_triplet->previous);
	_navigator->reset_position_setpoint(pos_sp_triplet->current);
	_navigator->reset_position_setpoint(pos_sp_triplet->next);

	// Set current position as loiter point if landed
	if (_navigator->get_land_detected()->landed) {
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	} else {
		// Set to current position
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.yaw = _navigator->get_local_position()->heading;
		pos_sp_triplet->current.valid = true;
	}

	_navigator->set_position_setpoint_triplet_updated();

	// Reset cruising speed to default
	_navigator->reset_cruising_speed();
}

void AutoVLA::on_active()
{
	// Update VLA trajectory
	update_vla_trajectory();

	// Generate position setpoint from VLA trajectory
	if (is_vla_trajectory_valid()) {
		generate_position_setpoint();
	} else {
		// No valid trajectory - hold current position
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		if (!_navigator->get_land_detected()->landed) {
			// Hold current position
			pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
			pos_sp_triplet->current.loiter_radius = _navigator->get_loiter_radius();
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
			pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
			pos_sp_triplet->current.yaw = _navigator->get_local_position()->heading;
			pos_sp_triplet->current.valid = true;

			_navigator->set_position_setpoint_triplet_updated();
		}
	}
}

void AutoVLA::update_vla_trajectory()
{
	// Update subscription
	if (_vla_trajectory_sub.updated()) {
		_vla_trajectory_sub.copy(&_vla_trajectory);
		_last_vla_update = hrt_absolute_time();
	}
}

void AutoVLA::generate_position_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Convert VLA bucket position to global coordinates
	// For simplicity, we use the bucket XY position as the target position
	// The bucket Z position will be handled by the boom/bucket control

	// Get current global position
	double current_lat = _navigator->get_global_position()->lat;
	double current_lon = _navigator->get_global_position()->lon;
	float current_alt = _navigator->get_global_position()->alt;

	// Convert bucket position (local NED) to global lat/lon
	// This is simplified - in reality you'd use proper coordinate transformation
	double target_lat, target_lon;
	float target_alt;

	// Get local position
	float x_local = _vla_trajectory.bucket_position_x;
	float y_local = _vla_trajectory.bucket_position_y;
	float z_local = _vla_trajectory.bucket_position_z;

	// Validate coordinates
	if (!PX4_ISFINITE(x_local) || !PX4_ISFINITE(y_local)) {
		// Invalid coordinates - skip update
		return;
	}

	// Use current position as reference and add offset
	// This is a simplified conversion - proper implementation would use map projection
	struct map_projection_reference_s ref;
	if (map_projection_init(&ref, current_lat, current_lon) != 0) {
		// Failed to initialize map projection
		return;
	}

	if (map_projection_project(&ref, x_local, y_local, &target_lat, &target_lon) != 0) {
		// Failed to project coordinates
		return;
	}

	target_alt = current_alt; // Keep altitude at ground level for wheel loader

	// Set position setpoint
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	pos_sp_triplet->current.lat = target_lat;
	pos_sp_triplet->current.lon = target_lon;
	pos_sp_triplet->current.alt = target_alt;
	pos_sp_triplet->current.yaw = _vla_trajectory.bucket_orientation_yaw;
	pos_sp_triplet->current.valid = true;

	// Set velocity constraints from VLA trajectory
	pos_sp_triplet->current.cruising_speed = math::constrain(_vla_trajectory.max_velocity,
								 0.1f,
								 _param_nav_autovla_vel.get());

	// Mark triplet as updated
	_navigator->set_position_setpoint_triplet_updated();
}

bool AutoVLA::is_vla_trajectory_valid() const
{
	if (!_vla_trajectory.valid_output || !_vla_trajectory.enable_trajectory) {
		return false;
	}

	// Check if trajectory is recent
	if (_vla_trajectory.timestamp > 0 && hrt_elapsed_time(&_vla_trajectory.timestamp) < VLA_TIMEOUT) {
		return true;
	}

	return false;
}
