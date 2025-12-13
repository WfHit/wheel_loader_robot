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
 * @file ModeManagerRover.hpp
 *
 * ModeManager subclass for rover (ground) vehicles.
 */

#pragma once

#include "ModeManagerBase.hpp"

/**
 * @class ModeManagerRover
 *
 * Rover specific ModeManager implementation.
 */
class ModeManagerRover : public ModeManagerBase
{
public:
	ModeManagerRover() : ModeManagerBase() {}
	~ModeManagerRover() override = default;

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROVER;
	}

	const char* getVehicleTypeName() const override
	{
		return "Rover";
	}

	void selectVehicleSpecificMode() override
	{
		const uint8_t operation_mode = _vehicle_status_sub.get().operation_mode;
		const vehicle_control_mode_s &vcm = _vehicle_control_mode_sub.get();

		ModeError error = ModeError::NoError;
		bool mode_activated = false;

		// Manual mode
		if (operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL) {
			error = switchMode(ModeIndex::ManualPosition);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Position control mode
		if (!mode_activated && operation_mode == vehicle_status_s::OPERATION_MODE_POSCTL) {
			error = switchMode(ModeIndex::ManualAcceleration);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Auto modes (mission, loiter, RTL)
		if (!mode_activated && vcm.flag_control_auto_enabled) {
			error = switchMode(ModeIndex::Auto);
			if (error == ModeError::NoError) { mode_activated = true; }
		}

		// Failsafe - default to manual position control
		if (!mode_activated) {
			error = switchMode(ModeIndex::ManualPosition);
			if (error != ModeError::NoError) {
				switchMode(ModeIndex::None);
			}
		}
	}

	uint64_t getAvailableModesMask() const override
	{
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
	}

	ModeIndex getDefaultModeIndex() const override
	{
		return ModeIndex::ManualPosition;
	}

	ModeIndex getFailsafeModeIndex() const override
	{
		return ModeIndex::ManualPosition;  // Rovers stop in place
	}
};
