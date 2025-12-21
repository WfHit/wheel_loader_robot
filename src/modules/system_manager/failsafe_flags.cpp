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

#include "failsafe_flags.hpp"
#include <uORB/topics/mode_status.h>

static inline void set_flag(uint8_t mode, uint32_t &flag_bits)
{
	flag_bits |= 1u << mode;
}

void fill_failsafe_flags(uint8_t vtype, failsafe_flags_s &flags_out)
{
	(void)vtype; // Currently unused, reserved for vehicle-specific requirements

	flags_out.mode_req_angular_velocity = 0;
	flags_out.mode_req_attitude = 0;
	flags_out.mode_req_local_position = 0;
	flags_out.mode_req_local_position_relaxed = 0;
	flags_out.mode_req_global_position = 0;
	flags_out.mode_req_local_alt = 0;
	flags_out.mode_req_current_auto_mission = 0;
	flags_out.mode_req_offboard_signal = 0;
	flags_out.mode_req_home_position = 0;
	flags_out.mode_req_wind_and_flight_time_compliance = 0;
	flags_out.mode_req_prevent_arming = 0;
	flags_out.mode_req_manual_control = 0;
	flags_out.mode_req_other = 0;

	// mode_req_manual_control: modes requiring manual RC input
	set_flag(mode_status_s::OPERATION_MODE_MANUAL, flags_out.mode_req_manual_control);
	set_flag(mode_status_s::OPERATION_MODE_ALTCTL, flags_out.mode_req_manual_control);
	set_flag(mode_status_s::OPERATION_MODE_POSCTL, flags_out.mode_req_manual_control);
	set_flag(mode_status_s::OPERATION_MODE_POSITION_SLOW, flags_out.mode_req_manual_control);
	set_flag(mode_status_s::OPERATION_MODE_ACRO, flags_out.mode_req_manual_control);
	set_flag(mode_status_s::OPERATION_MODE_STAB, flags_out.mode_req_manual_control);

	// mode_req_angular_velocity: modes requiring angular velocity estimate
	set_flag(mode_status_s::OPERATION_MODE_ALTCTL, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_POSCTL, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_POSITION_SLOW, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LOITER, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_ACRO, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_DESCEND, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_OFFBOARD, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_STAB, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LAND, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_PRECLAND, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_ORBIT, flags_out.mode_req_angular_velocity);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags_out.mode_req_angular_velocity);

	// mode_req_attitude: modes requiring attitude estimate
	set_flag(mode_status_s::OPERATION_MODE_ALTCTL, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_POSCTL, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_POSITION_SLOW, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LOITER, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_DESCEND, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_OFFBOARD, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_STAB, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LAND, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_PRECLAND, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_ORBIT, flags_out.mode_req_attitude);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags_out.mode_req_attitude);

	// mode_req_local_alt: modes requiring altitude estimate
	set_flag(mode_status_s::OPERATION_MODE_ALTCTL, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_POSCTL, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_POSITION_SLOW, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LOITER, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LAND, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_PRECLAND, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_ORBIT, flags_out.mode_req_local_alt);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags_out.mode_req_local_alt);

	// mode_req_local_position_relaxed: modes requiring relaxed position estimate
	set_flag(mode_status_s::OPERATION_MODE_POSCTL, flags_out.mode_req_local_position_relaxed);
	set_flag(mode_status_s::OPERATION_MODE_POSITION_SLOW, flags_out.mode_req_local_position_relaxed);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LAND, flags_out.mode_req_local_position_relaxed);

	// mode_req_local_position: modes requiring full local position estimate
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LOITER, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_PRECLAND, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_ORBIT, flags_out.mode_req_local_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags_out.mode_req_local_position);

	// mode_req_global_position: modes requiring global position estimate
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_global_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LOITER, flags_out.mode_req_global_position);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_global_position);

	// mode_req_current_auto_mission: modes requiring valid mission
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_current_auto_mission);

	// mode_req_home_position: modes requiring valid home position
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_home_position);

	// mode_req_offboard_signal: modes requiring offboard control signal
	set_flag(mode_status_s::OPERATION_MODE_OFFBOARD, flags_out.mode_req_offboard_signal);

	// mode_req_wind_and_flight_time_compliance: modes with wind/time constraints
	set_flag(mode_status_s::OPERATION_MODE_AUTO_MISSION, flags_out.mode_req_wind_and_flight_time_compliance);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LOITER, flags_out.mode_req_wind_and_flight_time_compliance);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags_out.mode_req_wind_and_flight_time_compliance);
	set_flag(mode_status_s::OPERATION_MODE_ORBIT, flags_out.mode_req_wind_and_flight_time_compliance);

	// mode_req_prevent_arming: modes that cannot be armed into directly
	set_flag(mode_status_s::OPERATION_MODE_AUTO_RTL, flags_out.mode_req_prevent_arming);
	set_flag(mode_status_s::OPERATION_MODE_DESCEND, flags_out.mode_req_prevent_arming);
	set_flag(mode_status_s::OPERATION_MODE_TERMINATION, flags_out.mode_req_prevent_arming);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_LAND, flags_out.mode_req_prevent_arming);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags_out.mode_req_prevent_arming);
	set_flag(mode_status_s::OPERATION_MODE_AUTO_PRECLAND, flags_out.mode_req_prevent_arming);
	set_flag(mode_status_s::OPERATION_MODE_ORBIT, flags_out.mode_req_prevent_arming);

	// EXTERNALx: handled outside

	static_assert(mode_status_s::OPERATION_MODE_MAX == 32, "update failsafe flags");
}
