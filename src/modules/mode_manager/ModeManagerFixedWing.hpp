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
 * @file ModeManagerFixedWing.hpp
 *
 * ModeManager subclass for fixed wing vehicles.
 */

#pragma once

#include "ModeManagerBase.hpp"

/**
 * @class ModeManagerFixedWing
 *
 * Fixed wing specific ModeManager implementation.
 */
class ModeManagerFixedWing : public ModeManagerBase
{
public:
	ModeManagerFixedWing() : ModeManagerBase() {}
	~ModeManagerFixedWing() override = default;

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	}

	const char* getVehicleTypeName() const override
	{
		return "Fixed Wing";
	}

	void selectVehicleSpecificMode() override
	{
		const vehicle_status_s &vstatus = _vehicle_status_sub.get();
		const uint8_t operation_mode = vstatus.operation_mode;

		ModeError error = ModeError::NoError;
		bool mode_activated = false;

		// Orbit (circle loiter)
		if (operation_mode == vehicle_status_s::OPERATION_MODE_ORBIT) {
#if !defined(CONSTRAINED_FLASH)
			error = switchMode(ModeIndex::Orbit);
			if (error == ModeError::NoError) { mode_activated = true; }
#endif
		}

		// Auto modes (mission, loiter, RTL, takeoff, land)
		if (!mode_activated && (
		    operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_MISSION ||
		    operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_LOITER ||
		    operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_RTL ||
		    operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF ||
		    operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_LAND)) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Position control
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_POSCTL) {
			error = switchMode(ModeIndex::ManualPosition);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Altitude control
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) {
			error = switchMode(ModeIndex::ManualAltitude);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Acro/Manual - no mode manager control
		if (!mode_activated && (operation_mode == vehicle_status_s::OPERATION_MODE_ACRO ||
					operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL)) {
			switchMode(ModeIndex::None);
			mode_activated = true;
		}

		// Failsafe
		if (!mode_activated) {
			error = switchMode(ModeIndex::Failsafe);
			if (error != ModeError::NoError) {
				switchMode(ModeIndex::None);
			}
		}
	}

	uint64_t getAvailableModesMask() const override
	{
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ALTCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ORBIT) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
	}

	ModeIndex getDefaultModeIndex() const override
	{
		return ModeIndex::ManualAltitude;
	}

	ModeIndex getFailsafeModeIndex() const override
	{
		return ModeIndex::Failsafe;
	}
};
