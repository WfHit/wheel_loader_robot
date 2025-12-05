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
 * Receives VLA trajectory items from MAVLink and publishes setpoint triplets
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
	// Reset VLA end effector trajectory item
	_current_trajectory_item = {};
	_last_trajectory_item_update = 0;
}

void AutoVLAEndEffector::on_inactive()
{
	// Nothing to do when inactive
}

void AutoVLAEndEffector::on_activation()
{
	// Reset on activation
	_current_trajectory_item = {};
	_last_trajectory_item_update = 0;

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

void AutoVLAEndEffector::on_active()
{
	// Update VLA end effector trajectory item from MAVLink
	update_vla_trajectory_item();

	// Generate and publish VLA end effector setpoint triplet
	if (is_vla_trajectory_item_valid()) {
		generate_vla_setpoint_triplet();
	} else {
		// No valid trajectory item - publish invalid setpoint triplet
		_vla_setpoint_triplet.timestamp = hrt_absolute_time();
		_vla_setpoint_triplet.current_valid = false;
		_vla_setpoint_triplet.previous_valid = false;
		_vla_setpoint_triplet.next_valid = false;
		_vla_setpoint_triplet_pub.publish(_vla_setpoint_triplet);
	}
}

void AutoVLAEndEffector::update_vla_trajectory_item()
{
	// Update subscription and check for new trajectory items from MAVLink
	if (_vla_trajectory_item_sub.updated()) {
		_vla_trajectory_item_sub.copy(&_current_trajectory_item);
		_last_trajectory_item_update = hrt_absolute_time();
	}
}

void AutoVLAEndEffector::generate_vla_setpoint_triplet()
{
	_vla_setpoint_triplet.timestamp = hrt_absolute_time();

	// For continuous MAVLink updates, we don't have previous/next items
	// Only current setpoint is valid
	_vla_setpoint_triplet.previous_valid = false;
	_vla_setpoint_triplet.next_valid = false;

	// Fill in current setpoint from received trajectory item
	_vla_setpoint_triplet.current_valid = true;
	_vla_setpoint_triplet.current_bucket_x = _current_trajectory_item.bucket_position_x;
	_vla_setpoint_triplet.current_bucket_y = _current_trajectory_item.bucket_position_y;
	_vla_setpoint_triplet.current_bucket_z = _current_trajectory_item.bucket_position_z;
	_vla_setpoint_triplet.current_bucket_roll = _current_trajectory_item.bucket_orientation_roll;
	_vla_setpoint_triplet.current_bucket_pitch = _current_trajectory_item.bucket_orientation_pitch;
	_vla_setpoint_triplet.current_bucket_yaw = _current_trajectory_item.bucket_orientation_yaw;

	// Set motion constraints from trajectory item
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

bool AutoVLAEndEffector::is_vla_trajectory_item_valid() const
{
	// Check if trajectory item is marked as valid
	if (!_current_trajectory_item.valid_output) {
		return false;
	}

	// Check if trajectory item is recent (received from MAVLink recently)
	if (_current_trajectory_item.timestamp > 0 && hrt_elapsed_time(&_current_trajectory_item.timestamp) < VLA_ITEM_TIMEOUT) {
		return true;
	}

	return false;
}
