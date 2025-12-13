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
 * @file ModeManagerStrategyWheelLoader.hpp
 *
 * ModeManager strategy for wheel loader vehicles
 */

#pragma once

#include "ModeManagerStrategyBase.hpp"

namespace mode_manager_strategy
{

// ModeIndex values (must match Modes_generated.hpp)
// These are placeholder values - actual values come from generated code
namespace ModeIndexValues {
	constexpr int None = -1;
	constexpr int VLA = 6;              // VLA mode index
	constexpr int ManualWheelLoader = 7; // Manual wheel loader mode index
	constexpr int Failsafe = 8;          // Failsafe mode index
}

class ModeManagerStrategyWheelLoader : public ModeManagerStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::WheelLoader;
	}

	const char* getName() const override
	{
		return "WheelLoader ModeManager Strategy";
	}

	//========================================================================
	// Mode Selection
	//========================================================================

	ModeSelectionResult selectMode(const ModeSelectionContext& context) const override
	{
		ModeSelectionResult result{};
		result.requested_mode_index = ModeIndexValues::None;
		result.found_mode = false;
		result.fallback_used = false;
		result.result = ModeResult::ModeUnavailable;

		// Check if requested mode is available
		if (!isModeAvailable(context.operation_mode)) {
			// Mode not available, use fallback
			result.requested_mode_index = getDefaultModeIndex();
			result.fallback_used = true;
			result.found_mode = true;
			result.result = ModeResult::FallbackUsed;
			return result;
		}

		// VLA 7-DOF Trajectory Following (autonomous mode)
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA) {
			result.requested_mode_index = ModeIndexValues::VLA;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Manual mode (default for wheel loader)
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL) {
			result.requested_mode_index = ModeIndexValues::ManualWheelLoader;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Default to manual mode
		result.requested_mode_index = getDefaultModeIndex();
		result.found_mode = true;
		result.fallback_used = true;
		result.result = ModeResult::FallbackUsed;
		return result;
	}

	bool isModeAvailable(uint8_t operation_mode) const override
	{
		return (getAvailableModesMask() & (1u << operation_mode)) != 0;
	}

	int getDefaultModeIndex() const override
	{
		return ModeIndexValues::ManualWheelLoader;
	}

	int getFailsafeModeIndex() const override
	{
		return ModeIndexValues::Failsafe;
	}

	uint32_t getAvailableModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_VLA);
	}

	//========================================================================
	// Mode Transition
	//========================================================================

	bool shouldPreserveSetpoints(int from_mode, int to_mode) const override
	{
		// Preserve setpoints when switching between VLA and Manual
		(void)from_mode;
		(void)to_mode;
		return true;
	}

	bool shouldResetTriplets(int from_mode, int to_mode) const override
	{
		// Don't reset triplets for wheel loader - we want smooth transitions
		(void)from_mode;
		(void)to_mode;
		return false;
	}

	//========================================================================
	// Setpoint Generation
	//========================================================================

	float getSetpointUpdateRate() const override
	{
		return 50.0f;  // 50Hz for wheel loader
	}
};

} // namespace mode_manager_strategy
