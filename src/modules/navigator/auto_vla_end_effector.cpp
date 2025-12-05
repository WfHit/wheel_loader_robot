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
 * @file auto_vla_end_effector.cpp
 *
 * Navigator mode for autonomous VLA end effector trajectory following
 * Supports MAVLink trajectory upload similar to mission protocol
 *
 * @author PX4 Development Team
 */

#include "auto_vla_end_effector.h"
#include "navigator.h"
#include <lib/geo/geo.h>

AutoVLAEndEffector::AutoVLAEndEffector(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_VLA_END_EFFECTOR),
	ModuleParams(navigator)
{
}

void AutoVLAEndEffector::initialize()
{
	// Reset VLA end effector trajectory
	_vla_trajectory = {};
	_current_trajectory_item = {};
	_current_trajectory_index = -1;
	_trajectory_item_reached = false;
	_trajectory_item_start_time = 0;
}

void AutoVLAEndEffector::on_inactive()
{
	// Nothing to do when inactive
}

void AutoVLAEndEffector::on_activation()
{
	// Reset on activation
	_current_trajectory_index = -1;
	_trajectory_item_reached = false;
	_trajectory_item_start_time = 0;

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

	// Load first trajectory item if available
	if (_vla_end_effector_trajectory_sub.get().count > 0) {
		load_vla_trajectory_item(0);
	}
}

void AutoVLAEndEffector::on_active()
{
	// Update VLA end effector trajectory
	update_vla_end_effector_trajectory();

	// Check if we need to advance to next trajectory item
	if (is_trajectory_item_reached()) {
		advance_vla_trajectory();
	}

	// Generate VLA end effector setpoint triplet from current trajectory items
	if (is_vla_end_effector_trajectory_valid()) {
		generate_vla_setpoint_triplet();
	} else {
		// No valid trajectory - publish invalid setpoint triplet
		_vla_setpoint_triplet.timestamp = hrt_absolute_time();
		_vla_setpoint_triplet.current_valid = false;
		_vla_setpoint_triplet.previous_valid = false;
		_vla_setpoint_triplet.next_valid = false;
		_vla_setpoint_triplet_pub.publish(_vla_setpoint_triplet);
	}
}

bool AutoVLAEndEffector::load_vla_trajectory_item(int index)
{
	if (index < 0 || index >= _vla_trajectory.count) {
		return false;
	}

	// Load trajectory item from dataman
	bool success = _dataman_client.readSync(_dataman_id, index,
						reinterpret_cast<uint8_t *>(&_current_trajectory_item),
						sizeof(vla_end_effector_trajectory_item_s));

	if (success) {
		_current_trajectory_index = index;
		_trajectory_item_reached = false;
		_trajectory_item_start_time = hrt_absolute_time();
		
		// Also load previous and next items for trajectory planning
		if (index > 0) {
			_dataman_client.readSync(_dataman_id, index - 1,
						reinterpret_cast<uint8_t *>(&_previous_trajectory_item),
						sizeof(vla_end_effector_trajectory_item_s));
		}
		
		if (index + 1 < _vla_trajectory.count) {
			_dataman_client.readSync(_dataman_id, index + 1,
						reinterpret_cast<uint8_t *>(&_next_trajectory_item),
						sizeof(vla_end_effector_trajectory_item_s));
		}
		
		return true;
	}

	return false;
}

void AutoVLAEndEffector::update_vla_end_effector_trajectory()
{
	// Update subscription and check for changes
	if (_vla_end_effector_trajectory_sub.updated()) {
		_vla_end_effector_trajectory_sub.copy(&_vla_trajectory);

		// If trajectory was updated, reload current item
		if (_current_trajectory_index >= 0 && _current_trajectory_index < _vla_trajectory.count) {
			load_vla_trajectory_item(_current_trajectory_index);
		} else if (_vla_trajectory.count > 0 && _vla_trajectory.current_seq >= 0) {
			// Start from specified sequence
			load_vla_trajectory_item(_vla_trajectory.current_seq);
		}
	}
}

void AutoVLAEndEffector::advance_vla_trajectory()
{
	// Check if we can continue to next item
	if (!_current_trajectory_item.autocontinue) {
		return;
	}

	int next_index = _current_trajectory_index + 1;

	// Check for looping
	if (next_index >= _vla_trajectory.count) {
		if (_vla_trajectory.loop_trajectory) {
			next_index = 0;
		} else {
			// Trajectory complete
			_trajectory_item_reached = true;
			return;
		}
	}

	// Load next trajectory item
	load_vla_trajectory_item(next_index);
}

void AutoVLAEndEffector::generate_vla_setpoint_triplet()
{
	_vla_setpoint_triplet.timestamp = hrt_absolute_time();

	// Fill in previous setpoint
	if (_current_trajectory_index > 0) {
		_vla_setpoint_triplet.previous_valid = true;
		_vla_setpoint_triplet.previous_bucket_x = _previous_trajectory_item.bucket_position_x;
		_vla_setpoint_triplet.previous_bucket_y = _previous_trajectory_item.bucket_position_y;
		_vla_setpoint_triplet.previous_bucket_z = _previous_trajectory_item.bucket_position_z;
		_vla_setpoint_triplet.previous_bucket_yaw = _previous_trajectory_item.bucket_orientation_yaw;
	} else {
		_vla_setpoint_triplet.previous_valid = false;
	}

	// Fill in current setpoint
	_vla_setpoint_triplet.current_valid = true;
	_vla_setpoint_triplet.current_bucket_x = _current_trajectory_item.bucket_position_x;
	_vla_setpoint_triplet.current_bucket_y = _current_trajectory_item.bucket_position_y;
	_vla_setpoint_triplet.current_bucket_z = _current_trajectory_item.bucket_position_z;
	_vla_setpoint_triplet.current_bucket_roll = _current_trajectory_item.bucket_orientation_roll;
	_vla_setpoint_triplet.current_bucket_pitch = _current_trajectory_item.bucket_orientation_pitch;
	_vla_setpoint_triplet.current_bucket_yaw = _current_trajectory_item.bucket_orientation_yaw;

	// Fill in next setpoint
	if (_current_trajectory_index + 1 < _vla_trajectory.count) {
		_vla_setpoint_triplet.next_valid = true;
		_vla_setpoint_triplet.next_bucket_x = _next_trajectory_item.bucket_position_x;
		_vla_setpoint_triplet.next_bucket_y = _next_trajectory_item.bucket_position_y;
		_vla_setpoint_triplet.next_bucket_z = _next_trajectory_item.bucket_position_z;
		_vla_setpoint_triplet.next_bucket_yaw = _next_trajectory_item.bucket_orientation_yaw;
	} else {
		_vla_setpoint_triplet.next_valid = false;
	}

	// Set motion constraints from current trajectory item
	_vla_setpoint_triplet.max_velocity = math::constrain(_current_trajectory_item.max_velocity,
							     0.1f,
							     _param_nav_autovla_ee_vel.get());
	_vla_setpoint_triplet.max_acceleration = math::constrain(_current_trajectory_item.max_acceleration,
								 0.1f,
								 _param_nav_autovla_ee_acc.get());
	_vla_setpoint_triplet.acceptance_radius = _current_trajectory_item.acceptance_radius;

	// Set wheel slip control
	_vla_setpoint_triplet.front_wheel_slip_rate = _current_trajectory_item.front_wheel_slip_rate;
	_vla_setpoint_triplet.rear_wheel_slip_rate = _current_trajectory_item.rear_wheel_slip_rate;

	// Set trajectory item information
	_vla_setpoint_triplet.item_type = _current_trajectory_item.item_type;
	_vla_setpoint_triplet.autocontinue = _current_trajectory_item.autocontinue;
	_vla_setpoint_triplet.loiter_radius = _current_trajectory_item.acceptance_radius;

	// Publish the setpoint triplet
	_vla_setpoint_triplet_pub.publish(_vla_setpoint_triplet);
}

bool AutoVLAEndEffector::is_trajectory_item_reached() const
{
	if (_trajectory_item_reached) {
		return true;
	}

	// Check if we're within acceptance radius
	float distance = get_distance_to_next_waypoint(_navigator->get_global_position()->lat,
							_navigator->get_global_position()->lon,
							_navigator->get_position_setpoint_triplet()->current.lat,
							_navigator->get_position_setpoint_triplet()->current.lon);

	if (distance < _current_trajectory_item.acceptance_radius) {
		// Check if we've waited long enough at the waypoint
		hrt_abstime time_at_waypoint = hrt_absolute_time() - _trajectory_item_start_time;
		if (time_at_waypoint > (hrt_abstime)(_current_trajectory_item.time_inside * 1e6f)) {
			return true;
		}
	}

	return false;
}

bool AutoVLAEndEffector::is_vla_end_effector_trajectory_valid() const
{
	// Check if we have a valid current trajectory item
	if (_current_trajectory_index < 0 || _current_trajectory_index >= _vla_trajectory.count) {
		return false;
	}

	// Check if trajectory item is marked as valid
	if (!_current_trajectory_item.valid_output) {
		return false;
	}

	// Check execution mode
	if (_vla_trajectory.execution_mode != vla_end_effector_trajectory_s::EXECUTION_MODE_ACTIVE) {
		return false;
	}

	return true;
}
