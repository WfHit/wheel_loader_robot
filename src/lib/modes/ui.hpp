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

#pragma once

#include <uORB/topics/vehicle_status.h>

#include <stdint.h>

namespace mode_util
{

/**
 * @return Bitmask with all valid modes
 */
static inline uint32_t getValidOperationModes()
{
	return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
	       (1u << vehicle_status_s::OPERATION_MODE_ALTCTL) |
	       (1u << vehicle_status_s::OPERATION_MODE_POSCTL) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
	       (1u << vehicle_status_s::OPERATION_MODE_POSITION_SLOW) |
	       (1u << vehicle_status_s::OPERATION_MODE_ACRO) |
	       (1u << vehicle_status_s::OPERATION_MODE_TERMINATION) |
	       (1u << vehicle_status_s::OPERATION_MODE_OFFBOARD) |
	       (1u << vehicle_status_s::OPERATION_MODE_STAB) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND) |
	       (1u << vehicle_status_s::OPERATION_MODE_ORBIT) |
	       (1u << vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF);

	static_assert(vehicle_status_s::OPERATION_MODE_MAX  == 31, "update valid nav states");
}

const char *const operation_mode_names[vehicle_status_s::OPERATION_MODE_MAX] = {
	"Manual",
	"Altitude",
	"Position",
	"Mission",
	"Hold",
	"Return",
	"Position Slow",
	"7: unallocated",
	"8: unallocated",
	"9: unallocated",
	"Acro",
	"11: UNUSED",
	"Descend",
	"Termination",
	"Offboard",
	"Stabilized",
	"16: UNUSED2",
	"Takeoff",
	"Land",
	"Follow Target",
	"Precision Landing",
	"Orbit",
	"VTOL Takeoff",
	"External 1",
	"External 2",
	"External 3",
	"External 4",
	"External 5",
	"External 6",
	"External 7",
	"External 8",
};

/**
 * @return returns true for advanced modes
 */
static inline bool isAdvanced(uint8_t nav_state)
{
	switch (nav_state) {
	case vehicle_status_s::OPERATION_MODE_ALTCTL: return false;

	case vehicle_status_s::OPERATION_MODE_POSCTL: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL1: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL2: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL3: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL4: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL5: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL6: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL7: return false;

	case vehicle_status_s::OPERATION_MODE_EXTERNAL8: return false;

	}

	return true;
}

} // namespace mode_util
