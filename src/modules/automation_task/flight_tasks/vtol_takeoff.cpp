/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file vtol_takeoff.cpp
 *
 * Helper class to do a VTOL takeoff and transition into a loiter.
 *
 */

#include "vtol_takeoff.h"
#include "../automation_task.hpp"

using matrix::wrap_pi;

VtolTakeoff::VtolTakeoff(Automation *automation) :
	TaskBlock(automation, mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF),
	ModuleParams(automation)
{
}

void
VtolTakeoff::on_activation()
{
	if (hrt_elapsed_time(&_automation->get_global_position()->timestamp) < 1_s) {
		set_takeoff_position();
		_takeoff_state = vtol_takeoff_state::TAKEOFF_HOVER;
		_automation->reset_cruising_speed();
		_automation->set_cruising_throttle();
	}
}

void
VtolTakeoff::on_active()
{
	if (is_current_task_item_reached_or_completed()) {
		reset_current_task_item_reached();

		switch	(_takeoff_state) {
		case vtol_takeoff_state::TAKEOFF_HOVER: {

				position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

				_current_task_item.nav_cmd = NAV_CMD_WAYPOINT;
				_current_task_item.yaw = wrap_pi(get_bearing_to_next_waypoint(_current_task_item.lat,
							    _current_task_item.lon, _loiter_location(0), _loiter_location(1)));
				_current_task_item.force_heading = true;
				task_item_to_position_setpoint(_current_task_item, &pos_sp_triplet->current);
				pos_sp_triplet->current.cruising_speed = -1.f;
				_automation->set_position_setpoint_triplet_updated();

				_takeoff_state = vtol_takeoff_state::ALIGN_HEADING;

				break;
			}

		case vtol_takeoff_state::ALIGN_HEADING: {

				set_vtol_transition_item(&_current_task_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);
				_current_task_item.lat = _loiter_location(0);
				_current_task_item.lon = _loiter_location(1);
				position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();
				pos_sp_triplet->previous = pos_sp_triplet->current;

				_automation->set_position_setpoint_triplet_updated();

				issue_command(_current_task_item);

				_takeoff_state = vtol_takeoff_state::TRANSITION;

				break;
			}

		case vtol_takeoff_state::TRANSITION: {
				position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

				if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
					setLoiterItemFromCurrentPositionSetpoint(&_current_task_item);

				} else {
					setLoiterItemFromCurrentPosition(&_current_task_item);
				}

				_current_task_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;

				// we need the vehicle to loiter indefinitely but also we want this mission item to be reached as soon
				// as the loiter is established. therefore, set a small loiter time so that the mission item will be reached quickly,
				// however it will just continue loitering as there is no next mission item
				_current_task_item.time_inside = 1.f;
				_current_task_item.loiter_radius = _automation->get_loiter_radius();
				_current_task_item.acceptance_radius  = _automation->get_acceptance_radius();
				_current_task_item.altitude = _takeoff_alt_msl + _param_loiter_alt.get();

				task_item_to_position_setpoint(_current_task_item, &pos_sp_triplet->current);
				pos_sp_triplet->current.lat = _loiter_location(0);
				pos_sp_triplet->current.lon = _loiter_location(1);
				pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
				pos_sp_triplet->current.cruising_speed = -1.f;
				pos_sp_triplet->current.cruising_throttle = -1.f;

				_current_task_item.lat = pos_sp_triplet->current.lat;
				_current_task_item.lon = pos_sp_triplet->current.lon;

				_automation->set_position_setpoint_triplet_updated();

				reset_current_task_item_reached();

				_takeoff_state = vtol_takeoff_state::CLIMB;

				break;
			}

		case vtol_takeoff_state::CLIMB: {

				// reset any potentially valid reposition triplet which was not handled
				// we do this to avoid random loiter locations after switching to loiter mode after this
				position_setpoint_triplet_s *reposition_triplet = _automation->get_reposition_triplet();
				_automation->reset_position_setpoint(reposition_triplet->previous);
				_automation->reset_position_setpoint(reposition_triplet->current);
				_automation->reset_position_setpoint(reposition_triplet->next);

				// the VTOL takeoff is done
				_automation->get_task_result()->finished = true;
				_automation->set_task_result_updated();
				_automation->mode_completed(getAutomationStateId());

				break;
			}

		default: {

				break;
			}
		}
	}
}

void
VtolTakeoff::set_takeoff_position()
{
	// set current mission item to takeoff
	set_takeoff_item(&_current_task_item, _transition_alt_amsl);

	_takeoff_alt_msl = _automation->get_global_position()->alt;

	_current_task_item.lat = _automation->get_global_position()->lat;
	_current_task_item.lon = _automation->get_global_position()->lon;

	_automation->get_task_result()->finished = false;
	_automation->set_task_result_updated();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();
	task_item_to_position_setpoint(_current_task_item, &pos_sp_triplet->current);

	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	_automation->set_position_setpoint_triplet_updated();
}
