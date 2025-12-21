/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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

#include "control_flags.hpp"

static bool stabilization_required(uint8_t vtype)
{
	return vtype == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
}

void fill_control_mode(uint8_t mode, uint8_t vtype,
		       vehicle_control_mode_s &control_mode_out)
{
	switch (mode) {
	case mode_status_s::OPERATION_MODE_MANUAL:
		control_mode_out.flag_control_manual_enabled = true;
		control_mode_out.flag_control_attitude_enabled = stabilization_required(vtype);
		control_mode_out.flag_control_rates_enabled = stabilization_required(vtype);
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_STAB:
		control_mode_out.flag_control_manual_enabled = true;
		control_mode_out.flag_control_attitude_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_ALTCTL:
		control_mode_out.flag_control_manual_enabled = true;
		control_mode_out.flag_control_altitude_enabled = true;
		control_mode_out.flag_control_climb_rate_enabled = true;
		control_mode_out.flag_control_attitude_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_POSCTL:
	case mode_status_s::OPERATION_MODE_POSITION_SLOW:
		control_mode_out.flag_control_manual_enabled = true;
		control_mode_out.flag_control_position_enabled = true;
		control_mode_out.flag_control_velocity_enabled = true;
		control_mode_out.flag_control_altitude_enabled = true;
		control_mode_out.flag_control_climb_rate_enabled = true;
		control_mode_out.flag_control_attitude_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_AUTO_RTL:
	case mode_status_s::OPERATION_MODE_AUTO_LAND:
	case mode_status_s::OPERATION_MODE_AUTO_PRECLAND:
	case mode_status_s::OPERATION_MODE_AUTO_MISSION:
	case mode_status_s::OPERATION_MODE_AUTO_LOITER:
	case mode_status_s::OPERATION_MODE_AUTO_TAKEOFF:
	case mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF:
		control_mode_out.flag_control_auto_enabled = true;
		control_mode_out.flag_control_position_enabled = true;
		control_mode_out.flag_control_velocity_enabled = true;
		control_mode_out.flag_control_altitude_enabled = true;
		control_mode_out.flag_control_climb_rate_enabled = true;
		control_mode_out.flag_control_attitude_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_ACRO:
		control_mode_out.flag_control_manual_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_DESCEND:
		control_mode_out.flag_control_auto_enabled = true;
		control_mode_out.flag_control_climb_rate_enabled = true;
		control_mode_out.flag_control_attitude_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_TERMINATION:
		/* disable all controllers on termination */
		control_mode_out.flag_control_termination_enabled = true;
		break;

	case mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET:
	// Follow Target supports RC adjustment, so disable auto control mode to disable
	// the Flight Task from exiting itself when RC stick movement is detected.
	case mode_status_s::OPERATION_MODE_ORBIT:
		control_mode_out.flag_control_manual_enabled = false;
		control_mode_out.flag_control_auto_enabled = false;
		control_mode_out.flag_control_position_enabled = true;
		control_mode_out.flag_control_velocity_enabled = true;
		control_mode_out.flag_control_altitude_enabled = true;
		control_mode_out.flag_control_climb_rate_enabled = true;
		control_mode_out.flag_control_attitude_enabled = true;
		control_mode_out.flag_control_rates_enabled = true;
		control_mode_out.flag_control_allocation_enabled = true;
		break;

	default:
		break;
	}
}
