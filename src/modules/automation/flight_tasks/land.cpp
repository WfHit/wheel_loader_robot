/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include "land.h"
#include "../automation.h"

Land::Land(Automation *automation) :
	TaskBlock(automation, vehicle_status_s::OPERATION_MODE_AUTO_LAND)
{
}

void
Land::on_activation()
{
	/* set current mission item to Land */
	set_land_item(&_current_task_item);
	_automation->get_task_result()->finished = false;
	_automation->set_task_result_updated();
	reset_current_task_item_reached();

	/* convert mission item to current setpoint */
	struct position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

	if (_automation->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && _automation->get_local_position()->xy_global) { // only execute if global position is valid
		_automation->preproject_stop_point(_current_task_item.lat, _current_task_item.lon);
	}

	task_item_to_position_setpoint(_current_task_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	_automation->set_position_setpoint_triplet_updated();

	// reset cruising speed to default
	_automation->reset_cruising_speed();

	// set gimbal to neutral position (level with horizon) to reduce change of damage on landing
	_automation->acquire_gimbal_control();
	_automation->set_gimbal_neutral();
	_automation->release_gimbal_control();

}

void
Land::on_active()
{
	/* for VTOL update landing location during back transition */
	if (_automation->get_vstatus()->is_vtol
	    && _automation->get_vstatus()->in_transition_mode
	    && _automation->get_local_position()->xy_global) {
		struct position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();

		// create a wp in front of the VTOL while in back-transition, based on MPC settings that will apply in MC phase afterwards
		_automation->preproject_stop_point(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);
		_automation->set_position_setpoint_triplet_updated();
	}


	if (_automation->get_land_detected()->landed) {
		_automation->get_task_result()->finished = true;
		_automation->set_task_result_updated();
		_automation->mode_completed(getAutomationStateId());
		set_idle_item(&_current_task_item);

		struct position_setpoint_triplet_s *pos_sp_triplet = _automation->get_position_setpoint_triplet();
		task_item_to_position_setpoint(_current_task_item, &pos_sp_triplet->current);
		_automation->set_position_setpoint_triplet_updated();
	}

	/* check if landing needs to be aborted */
	if (_automation->abort_landing()) {

		// send reposition cmd to get out of land mode (will loiter at current position and altitude)
		vehicle_command_s vehicle_command{};
		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
		vehicle_command.param1 = -1.f; // Default speed
		vehicle_command.param2 = 1.f; // Modes should switch, not setting this is unsupported
		vehicle_command.param5 = _automation->get_global_position()->lat;
		vehicle_command.param6 = _automation->get_global_position()->lon;
		// as we don't know the landing point altitude assume the worst case (abort at 0m above ground),
		// and thus always climb MIS_LND_ABRT_ALT
		vehicle_command.param7 = _automation->get_global_position()->alt + _automation->get_landing_abort_min_alt();

		_automation->publish_vehicle_command(vehicle_command);
	}
}
