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
 * @brief ModeManager subclass for wheel loader vehicles
 *
 * This class manages mode selection and transitions for articulated wheel loaders.
 * Wheel loaders support a limited set of operation modes focused on ground-based
 * construction operations.
 *
 * Supported modes:
 * - Manual: Direct operator control of chassis, boom, and bucket
 * - VLA (Vision-Language-Action): Autonomous operation using AI-guided control
 * - Offboard: External control via MAVLink
 *
 * Mode transitions:
 * - Any supported mode can transition to any other supported mode
 * - Unsupported modes automatically fall back to Manual
 * - Failsafe always returns to Manual mode (emergency stop)
 */

#pragma once

#include "ModeManagerBase.hpp"

/**
 * @class ModeManagerWheelLoader
 * @brief Wheel loader specific ModeManager implementation
 *
 * Key behaviors:
 * - Limited mode set (Manual, VLA, Offboard only)
 * - No aerial flight modes
 * - Manual mode as both default and failsafe
 * - VLA mode for autonomous operations
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

		// Validate requested mode availability
		if (!isModeAvailable(operation_mode)) {
			PX4_WARN("Wheel Loader: Mode %d not available, falling back to manual",
				 operation_mode);
		}

		// Try to activate the requested mode
		ModeError error = ModeError::NoError;

		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			error = switchMode(ModeIndex::VLA);

			if (error == ModeError::NoError) {
				logModeSelection("VLA", true);
				return;
			}

			logModeSelection("VLA", false);
			break;

		case vehicle_status_s::OPERATION_MODE_OFFBOARD:
			error = switchMode(ModeIndex::Offboard);

			if (error == ModeError::NoError) {
				logModeSelection("Offboard", true);
				return;
			}

			logModeSelection("Offboard", false);
			break;

		default:
			// Fall through to manual mode
			break;
		}

		// Default/fallback: Manual mode
		error = switchMode(ModeIndex::ManualWheelLoader);

		if (error == ModeError::NoError) {
			logModeSelection("ManualWheelLoader", true);
			return;
		}

		// Last resort: failsafe
		logModeSelection("ManualWheelLoader", false);
		PX4_ERR("Wheel Loader: No valid mode available, entering failsafe");
		switchMode(ModeIndex::None);
	}

	//========================================================================
	// Mode Availability
	//========================================================================

	uint64_t getAvailableModesMask() const override
	{
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
		return ModeIndex::ManualWheelLoader;
	}

	ModeIndex getModeIndexForOperationMode(uint8_t operation_mode) const override
	{
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			return ModeIndex::VLA;

		case vehicle_status_s::OPERATION_MODE_OFFBOARD:
			return ModeIndex::Offboard;

		case vehicle_status_s::OPERATION_MODE_MANUAL:
		default:
			return ModeIndex::ManualWheelLoader;
		}
	}

	//========================================================================
	// Mode Transition
	//========================================================================

	bool isTransitionAllowed(uint8_t from_mode, uint8_t to_mode) const override
	{
		// Allow transition only between supported modes
		return isModeAvailable(from_mode) && isModeAvailable(to_mode);
	}
};
