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

/**
 * @file event_mode.hpp
 *
 * Event mode conversion utilities.
 * Converts operation mode values to event enums for logging/reporting.
 */

#pragma once

#include <uORB/topics/mode_status.h>
#include <px4_platform_common/events.h>

#include <stdint.h>

using operation_mode_t = events::px4::enums::operation_mode_t;

/**
 * Convert operation mode to operation_mode_t for event reporting
 *
 * Maps operation mode constants to the operation_mode_t enum used
 * in event/logging system.
 *
 * @param mode Current operation mode (mode_status_s::OPERATION_MODE_*)
 * @return Corresponding operation_mode_t enum value
 */
static inline operation_mode_t operation_mode_to_event_mode(uint8_t mode)
{
	switch (mode) {
	case mode_status_s::OPERATION_MODE_MANUAL:
		return operation_mode_t::manual;

	case mode_status_s::OPERATION_MODE_ALTCTL:
		return operation_mode_t::altctl;

	case mode_status_s::OPERATION_MODE_POSCTL:
		return operation_mode_t::posctl;

	case mode_status_s::OPERATION_MODE_AUTO_MISSION:
		return operation_mode_t::auto_mission;

	case mode_status_s::OPERATION_MODE_AUTO_LOITER:
		return operation_mode_t::auto_loiter;

	case mode_status_s::OPERATION_MODE_AUTO_RTL:
		return operation_mode_t::auto_rtl;

	case mode_status_s::OPERATION_MODE_POSITION_SLOW:
		return operation_mode_t::position_slow;

	case mode_status_s::OPERATION_MODE_ACRO:
		return operation_mode_t::acro;

	case mode_status_s::OPERATION_MODE_OFFBOARD:
		return operation_mode_t::offboard;

	case mode_status_s::OPERATION_MODE_STAB:
		return operation_mode_t::stab;

	case mode_status_s::OPERATION_MODE_AUTO_TAKEOFF:
		return operation_mode_t::auto_takeoff;

	case mode_status_s::OPERATION_MODE_AUTO_LAND:
		return operation_mode_t::auto_land;

	case mode_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET:
		return operation_mode_t::auto_follow_target;

	case mode_status_s::OPERATION_MODE_AUTO_PRECLAND:
		return operation_mode_t::auto_precland;

	case mode_status_s::OPERATION_MODE_ORBIT:
		return operation_mode_t::orbit;

	case mode_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF:
		return operation_mode_t::auto_vtol_takeoff;

	case mode_status_s::OPERATION_MODE_EXTERNAL1:
		return operation_mode_t::external1;

	case mode_status_s::OPERATION_MODE_EXTERNAL2:
		return operation_mode_t::external2;

	case mode_status_s::OPERATION_MODE_EXTERNAL3:
		return operation_mode_t::external3;

	case mode_status_s::OPERATION_MODE_EXTERNAL4:
		return operation_mode_t::external4;

	case mode_status_s::OPERATION_MODE_EXTERNAL5:
		return operation_mode_t::external5;

	case mode_status_s::OPERATION_MODE_EXTERNAL6:
		return operation_mode_t::external6;

	case mode_status_s::OPERATION_MODE_EXTERNAL7:
		return operation_mode_t::external7;

	case mode_status_s::OPERATION_MODE_EXTERNAL8:
		return operation_mode_t::external8;
	}

	static_assert(mode_status_s::OPERATION_MODE_MAX == 32, "update operation mode conversion map");

	return operation_mode_t::unknown;
}
