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

#include "mode_requirements.hpp"
#include <uORB/topics/vehicle_status.h>
#include <lib/vehicle_type/VehicleTypeRegistry.hpp>

namespace mode_util
{

static inline void setRequirement(uint8_t nav_state, uint32_t &mode_requirement)
{
	mode_requirement |= 1u << nav_state;
}


void getModeRequirements(uint8_t vehicle_type, failsafe_flags_s &flags)
{
	flags.mode_req_angular_velocity = 0;
	flags.mode_req_attitude = 0;
	flags.mode_req_local_position = 0;
	flags.mode_req_local_position_relaxed = 0;
	flags.mode_req_global_position = 0;
	flags.mode_req_local_alt = 0;
	flags.mode_req_current_auto_mission = 0;
	flags.mode_req_offboard_signal = 0;
	flags.mode_req_home_position = 0;
	flags.mode_req_wind_and_flight_time_compliance = 0;
	flags.mode_req_prevent_arming = 0;
	flags.mode_req_manual_control = 0;
	flags.mode_req_other = 0;

	// Use the vehicle type strategy to set mode requirements
	const vehicle_type::VehicleTypeStrategy *strategy = vehicle_type::VehicleTypeRegistry::getStrategy(vehicle_type);

	if (strategy) {
		strategy->setModeRequirements(flags);
		return;
	}

	// Fallback to default requirements for unsupported vehicle types
	// OPERATION_MODE_MANUAL
	setRequirement(vehicle_status_s::OPERATION_MODE_MANUAL, flags.mode_req_manual_control);

	// OPERATION_MODE_ALTCTL
	setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_manual_control);

	// OPERATION_MODE_POSCTL
	setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_local_position_relaxed);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_manual_control);

	// OPERATION_MODE_POSITION_SLOW
	setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_local_position_relaxed);
	setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_manual_control);

	// OPERATION_MODE_AUTO_MISSION
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_global_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_current_auto_mission);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_wind_and_flight_time_compliance);

	// OPERATION_MODE_AUTO_LOITER
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_global_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_wind_and_flight_time_compliance);

	// OPERATION_MODE_AUTO_RTL
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_global_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_home_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_prevent_arming);

	// OPERATION_MODE_ACRO
	setRequirement(vehicle_status_s::OPERATION_MODE_ACRO, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_ACRO, flags.mode_req_manual_control);

	// OPERATION_MODE_DESCEND
	setRequirement(vehicle_status_s::OPERATION_MODE_DESCEND, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_DESCEND, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_DESCEND, flags.mode_req_prevent_arming);

	// OPERATION_MODE_TERMINATION
	setRequirement(vehicle_status_s::OPERATION_MODE_TERMINATION, flags.mode_req_prevent_arming);

	// OPERATION_MODE_OFFBOARD
	setRequirement(vehicle_status_s::OPERATION_MODE_OFFBOARD, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_OFFBOARD, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_OFFBOARD, flags.mode_req_offboard_signal);

	// OPERATION_MODE_STAB
	setRequirement(vehicle_status_s::OPERATION_MODE_STAB, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_STAB, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_STAB, flags.mode_req_manual_control);

	// OPERATION_MODE_AUTO_TAKEOFF
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_local_alt);

	// OPERATION_MODE_AUTO_LAND
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_local_position_relaxed);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_prevent_arming);

	// OPERATION_MODE_AUTO_FOLLOW_TARGET
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_prevent_arming);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_wind_and_flight_time_compliance);

	// OPERATION_MODE_AUTO_PRECLAND
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_prevent_arming);

	// OPERATION_MODE_ORBIT
	setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_local_alt);
	setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_prevent_arming);
	setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_wind_and_flight_time_compliance);

	// OPERATION_MODE_AUTO_VTOL_TAKEOFF
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_angular_velocity);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_attitude);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_local_position);
	setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_local_alt);

	// OPERATION_MODE_EXTERNALx: handled outside

	static_assert(vehicle_status_s::OPERATION_MODE_MAX == 31, "update mode requirements");
}

} // namespace mode_util
