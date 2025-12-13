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
 * @file ModeManagerRotaryWing.hpp
 *
 * ModeManager subclass for rotary wing (multicopter) vehicles.
 */

#pragma once

#include "ModeManagerBase.hpp"

/**
 * @class ModeManagerRotaryWing
 *
 * Rotary wing (multicopter) specific ModeManager implementation.
 */
class ModeManagerRotaryWing : public ModeManagerBase
{
public:
	ModeManagerRotaryWing() : ModeManagerBase() {}
	~ModeManagerRotaryWing() override = default;

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	const char* getVehicleTypeName() const override
	{
		return "Rotary Wing";
	}

	void selectVehicleSpecificMode() override
	{
		const vehicle_status_s &vstatus = _vehicle_status_sub.get();
		const vehicle_control_mode_s &vcm = _vehicle_control_mode_sub.get();
		const uint8_t operation_mode = vstatus.operation_mode;

		ModeError error = ModeError::NoError;
		bool mode_activated = false;

		// Transition mode for VTOL
		if (vstatus.in_transition_mode && vcm.flag_control_altitude_enabled) {
			error = switchMode(ModeIndex::Transition);
			if (error == ModeError::NoError) return;
		}

		// Follow target
		if (operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) {
#if !defined(CONSTRAINED_FLASH)
			error = switchMode(ModeIndex::AutoFollowTarget);
			if (error == ModeError::NoError) { mode_activated = true; }
#endif
		}

		// Orbit
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_ORBIT) {
#if !defined(CONSTRAINED_FLASH)
			error = switchMode(ModeIndex::Orbit);
			if (error == ModeError::NoError) { mode_activated = true; }
#endif
		}

		// Takeoff
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Land
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_LAND) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// RTL
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_RTL) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Loiter
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_LOITER) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Mission
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_MISSION) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Descend
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_DESCEND) {
			error = switchMode(ModeIndex::Descend);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Position control
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_POSCTL) {
			error = switchMode(ModeIndex::ManualPosition);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Position slow
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_POSITION_SLOW) {
			error = switchMode(ModeIndex::ManualAccelerationSlow);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Altitude control
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) {
			error = switchMode(ModeIndex::ManualAltitude);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Acro/Manual
		if (!mode_activated && (operation_mode == vehicle_status_s::OPERATION_MODE_ACRO ||
					operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL)) {
			// No mode manager control for acro/manual
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
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSITION_SLOW) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ORBIT) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_DESCEND);
	}

	ModeIndex getDefaultModeIndex() const override
	{
		return ModeIndex::ManualPosition;
	}

	ModeIndex getFailsafeModeIndex() const override
	{
		return ModeIndex::Failsafe;
	}
};
