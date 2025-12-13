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
 * @file ModeManagerStrategyRotaryWing.hpp
 *
 * ModeManager strategy for rotary wing (multicopter) vehicles
 */

#pragma once

#include "ModeManagerStrategyBase.hpp"

namespace mode_manager_strategy
{

// ModeIndex values (must match Modes_generated.hpp)
namespace RotaryWingModeIndex {
	constexpr int None = -1;
	constexpr int Transition = 0;
	constexpr int AutoFollowTarget = 1;
	constexpr int Orbit = 2;
	constexpr int Auto = 3;
	constexpr int ManualAccelerationSlow = 4;
	constexpr int ManualPosition = 5;
	constexpr int ManualAcceleration = 6;
	constexpr int ManualAltitude = 7;
	constexpr int ManualAltitudeSmoothVel = 8;
	constexpr int Descend = 9;
	constexpr int Failsafe = 10;
}

class ModeManagerStrategyRotaryWing : public ModeManagerStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::RotaryWing;
	}

	const char* getName() const override
	{
		return "RotaryWing ModeManager Strategy";
	}

	//========================================================================
	// Mode Selection
	//========================================================================

	ModeSelectionResult selectMode(const ModeSelectionContext& context) const override
	{
		ModeSelectionResult result{};
		result.requested_mode_index = RotaryWingModeIndex::None;
		result.found_mode = false;
		result.fallback_used = false;
		result.result = ModeResult::ModeUnavailable;

		const bool operation_mode_descend =
			(context.operation_mode == vehicle_status_s::OPERATION_MODE_DESCEND);

		// Transition mode for VTOLs
		if (context.in_transition_mode && context.flag_control_altitude_enabled) {
			result.requested_mode_index = RotaryWingModeIndex::Transition;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Follow me
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) {
			result.requested_mode_index = RotaryWingModeIndex::AutoFollowTarget;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Orbit
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_ORBIT &&
		    !context.command_failed) {
			result.requested_mode_index = RotaryWingModeIndex::Orbit;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Auto modes (mission, loiter, RTL, etc.)
		if (context.flag_control_auto_enabled && !operation_mode_descend) {
			result.requested_mode_index = RotaryWingModeIndex::Auto;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Position slow mode
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_POSITION_SLOW) {
			result.requested_mode_index = RotaryWingModeIndex::ManualAccelerationSlow;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Position control
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_POSCTL) {
			result.requested_mode_index = RotaryWingModeIndex::ManualAcceleration;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Altitude control
		if (context.operation_mode == vehicle_status_s::OPERATION_MODE_ALTCTL) {
			result.requested_mode_index = RotaryWingModeIndex::ManualAltitudeSmoothVel;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// Emergency descend
		if (operation_mode_descend) {
			result.requested_mode_index = RotaryWingModeIndex::Descend;
			result.found_mode = true;
			result.result = ModeResult::Success;
			return result;
		}

		// No matching mode - use failsafe
		result.requested_mode_index = RotaryWingModeIndex::Failsafe;
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
		return RotaryWingModeIndex::ManualAcceleration;
	}

	int getFailsafeModeIndex() const override
	{
		return RotaryWingModeIndex::Failsafe;
	}

	uint32_t getAvailableModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_ACRO) |
		       (1u << vehicle_status_s::OPERATION_MODE_STAB) |
		       (1u << vehicle_status_s::OPERATION_MODE_ALTCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_POSITION_SLOW) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LAND) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET) |
		       (1u << vehicle_status_s::OPERATION_MODE_ORBIT) |
		       (1u << vehicle_status_s::OPERATION_MODE_OFFBOARD) |
		       (1u << vehicle_status_s::OPERATION_MODE_DESCEND);
	}

	//========================================================================
	// Mode Transition
	//========================================================================

	bool shouldPreserveSetpoints(int from_mode, int to_mode) const override
	{
		// Preserve setpoints when going from takeoff to loiter
		(void)from_mode;
		(void)to_mode;
		return false;
	}

	bool shouldResetTriplets(int from_mode, int to_mode) const override
	{
		// Check for takeoff to loiter transition (don't reset)
		// This would need actual ModeIndex values to implement properly
		(void)from_mode;
		(void)to_mode;
		return true;
	}
};

} // namespace mode_manager_strategy
