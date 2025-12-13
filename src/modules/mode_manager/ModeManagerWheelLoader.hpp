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
 * @file ModeManagerWheelLoader.hpp
 *
 * ModeManager subclass for wheel loader vehicles.
 * Handles wheel loader specific mode selection including VLA autonomous mode.
 */

#pragma once

#include "ModeManagerBase.hpp"

/**
 * @class ModeManagerWheelLoader
 *
 * Wheel loader specific ModeManager implementation.
 *
 * Key behaviors:
 * - Supports only Manual and VLA modes
 * - VLA (Vision-Language-Action) for autonomous operation
 * - Manual mode for direct operator control
 * - No aerial flight modes
 */
class ModeManagerWheelLoader : public ModeManagerBase
{
public:
	ModeManagerWheelLoader() : ModeManagerBase() {}
	~ModeManagerWheelLoader() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char* getVehicleTypeName() const override
	{
		return "Wheel Loader";
	}

	//========================================================================
	// Mode Selection
	//========================================================================

	void selectVehicleSpecificMode() override
	{
		const uint8_t operation_mode = _vehicle_status_sub.get().operation_mode;
		ModeError error = ModeError::NoError;
		bool mode_activated = false;

		// Check if requested mode is available
		if (!isModeAvailable(operation_mode)) {
			PX4_WARN("Mode %d not available for wheel loader", operation_mode);
		}

		// VLA 7-DOF Trajectory Following (chassis + boom + tilt) - autonomous mode
		if (operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA) {
			error = switchMode(ModeIndex::VLA);

			if (error == ModeError::NoError) {
				mode_activated = true;
				logModeSelection("VLA", true);

			} else {
				logModeSelection("VLA", false);
			}
		}

		// Manual mode for wheel loader
		if (!mode_activated &&
		    (operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL ||
		     !mode_activated)) {  // Fallback to manual
			error = switchMode(ModeIndex::ManualWheelLoader);

			if (error == ModeError::NoError) {
				mode_activated = true;
				logModeSelection("ManualWheelLoader", true);

			} else {
				logModeSelection("ManualWheelLoader", false);
			}
		}

		// Failsafe if no mode activated
		if (!mode_activated) {
			PX4_WARN("Wheel loader entering failsafe mode");
			error = switchMode(getFailsafeModeIndex());

			if (error != ModeError::NoError) {
				PX4_ERR("No valid mode available for wheel loader");
				switchMode(ModeIndex::None);
			}
		}
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
		// Wheel loaders only support manual and VLA modes
		return (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_VLA) |
		       (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
	}

	ModeIndex getDefaultModeIndex() const override
	{
		return ModeIndex::ManualWheelLoader;
	}

	ModeIndex getFailsafeModeIndex() const override
	{
		// Wheel loader failsafe is manual mode (stop and wait for operator)
		return ModeIndex::ManualWheelLoader;
	}

	ModeIndex getModeIndexForOperationMode(uint8_t operation_mode) const override
	{
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_MANUAL:
			return ModeIndex::ManualWheelLoader;

		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			return ModeIndex::VLA;

		default:
			return ModeIndex::ManualWheelLoader;  // Default fallback
		}
	}

	//========================================================================
	// Mode Transition
	//========================================================================

	bool isTransitionAllowed(uint8_t from_mode, uint8_t to_mode) const override
	{
		// Wheel loader allows transition between any supported modes
		return isModeAvailable(from_mode) && isModeAvailable(to_mode);
	}
};
