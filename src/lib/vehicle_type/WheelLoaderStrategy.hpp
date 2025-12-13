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
 * @file WheelLoaderStrategy.hpp
 *
 * Vehicle Type Strategy for Articulated Wheel Loaders
 *
 * Wheel loaders are ground vehicles with:
 * - Articulated steering (front/rear chassis sections)
 * - Boom and bucket end effector
 * - Manual and VLA (Vision-Language-Action) autonomous modes
 * - No flight capabilities (2D position + heading)
 */

#pragma once

#include "VehicleTypeStrategy.hpp"

namespace vehicle_type
{

class WheelLoaderStrategy : public VehicleTypeStrategy
{
public:
	WheelLoaderStrategy() = default;
	~WheelLoaderStrategy() override = default;

	//========================================================================
	// Vehicle Identification
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char *getVehicleTypeName() const override
	{
		return "Wheel Loader";
	}

	//========================================================================
	// Operation Modes
	//========================================================================

	int getNumSupportedModes() const override
	{
		return NUM_SUPPORTED_MODES;
	}

	const OperationModeInfo *getSupportedMode(int index) const override
	{
		if (index >= 0 && index < NUM_SUPPORTED_MODES) {
			return &_supported_modes[index];
		}

		return nullptr;
	}

	bool isModeSupported(uint8_t operation_mode) const override
	{
		return operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL
		       || operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_VLA
		       || operation_mode == vehicle_status_s::OPERATION_MODE_TERMINATION;
	}

	uint32_t getValidOperationModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL)
		       | (1u << vehicle_status_s::OPERATION_MODE_AUTO_VLA)
		       | (1u << vehicle_status_s::OPERATION_MODE_TERMINATION);
	}

	uint32_t getSelectableOperationModesMask() const override
	{
		// Users can select manual and VLA, but not termination
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL)
		       | (1u << vehicle_status_s::OPERATION_MODE_AUTO_VLA);
	}

	//========================================================================
	// Mode Selection Logic
	//========================================================================

	ModeChangeConfig getModeChangeConfig() const override
	{
		ModeChangeConfig config{};
		config.allow_stick_override = true;           // Allow RC override in VLA mode
		config.require_disarm_for_mode_change = false; // Can change modes while armed
		config.default_mode = vehicle_status_s::OPERATION_MODE_MANUAL;
		config.failsafe_mode = vehicle_status_s::OPERATION_MODE_MANUAL; // Failsafe to manual control
		config.manual_fallback_mode = vehicle_status_s::OPERATION_MODE_MANUAL;
		return config;
	}

	uint8_t selectMode(uint8_t requested_mode, uint8_t current_mode,
			   bool is_armed, bool has_position, bool has_manual_control) const override
	{
		// Handle specific mode requests
		switch (requested_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			// VLA requires position estimate
			if (has_position) {
				return vehicle_status_s::OPERATION_MODE_AUTO_VLA;
			}

			// Fall back to manual if no position
			return vehicle_status_s::OPERATION_MODE_MANUAL;

		case vehicle_status_s::OPERATION_MODE_MANUAL:
			// Manual mode requires manual control input
			if (has_manual_control) {
				return vehicle_status_s::OPERATION_MODE_MANUAL;
			}

			// If no manual control, stay in current mode or go to termination
			if (is_armed) {
				return current_mode;
			}

			return vehicle_status_s::OPERATION_MODE_MANUAL;

		case vehicle_status_s::OPERATION_MODE_TERMINATION:
			// Termination always allowed
			return vehicle_status_s::OPERATION_MODE_TERMINATION;

		default:
			// Unsupported mode - fall back to manual
			return vehicle_status_s::OPERATION_MODE_MANUAL;
		}
	}

	uint8_t getFallbackMode(uint8_t failed_mode, bool is_armed) const override
	{
		// For wheel loaders, always fall back to manual control
		// (unlike aircraft which might need to descend/land)
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	//========================================================================
	// Control Mode Flags
	//========================================================================

	void getControlModeFlags(uint8_t operation_mode,
				 const offboard_control_mode_s &offboard_control_mode,
				 vehicle_control_mode_s &control_mode) const override
	{
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_MANUAL:
			// Manual mode: direct RC control to actuators
			control_mode.flag_control_manual_enabled = true;
			// No attitude/rate control needed for ground vehicle
			control_mode.flag_control_attitude_enabled = false;
			control_mode.flag_control_rates_enabled = false;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			// VLA autonomous mode: position/velocity control
			control_mode.flag_control_auto_enabled = true;
			control_mode.flag_control_position_enabled = true;
			control_mode.flag_control_velocity_enabled = true;
			// No altitude control for ground vehicle
			control_mode.flag_control_altitude_enabled = false;
			control_mode.flag_control_climb_rate_enabled = false;
			// No attitude/rate control for ground vehicle
			control_mode.flag_control_attitude_enabled = false;
			control_mode.flag_control_rates_enabled = false;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_TERMINATION:
			// Termination: disable all control
			control_mode.flag_control_termination_enabled = true;
			break;

		default:
			// Default to manual mode flags
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;
		}
	}

	bool requiresStabilization() const override
	{
		// Ground vehicles don't need attitude stabilization
		return false;
	}

	//========================================================================
	// Mode Requirements
	//========================================================================

	void setModeRequirements(failsafe_flags_s &flags) const override
	{
		// OPERATION_MODE_MANUAL
		setRequirement(vehicle_status_s::OPERATION_MODE_MANUAL, flags.mode_req_manual_control);

		// OPERATION_MODE_AUTO_VLA
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VLA, flags.mode_req_local_position_relaxed);

		// OPERATION_MODE_TERMINATION
		setRequirement(vehicle_status_s::OPERATION_MODE_TERMINATION, flags.mode_req_prevent_arming);
	}

	//========================================================================
	// Failsafe Behavior
	//========================================================================

	uint8_t getFailsafeAction(uint8_t failure_type, uint8_t current_mode, bool is_armed) const override
	{
		// For wheel loaders, most failures should result in stopping (manual mode)
		// This is safer than aircraft which need active control to land
		if (!is_armed) {
			return vehicle_status_s::OPERATION_MODE_MANUAL;
		}

		// For critical failures, enter manual mode (operator takes over)
		// In future, could add a "hold" mode that stops the vehicle
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	bool isSafeStopMode(uint8_t operation_mode) const override
	{
		// Manual mode is safe (operator in control)
		// Termination is also safe (all outputs disabled)
		return operation_mode == vehicle_status_s::OPERATION_MODE_MANUAL
		       || operation_mode == vehicle_status_s::OPERATION_MODE_TERMINATION;
	}

	//========================================================================
	// Automation Task Support
	//========================================================================

	bool supportsMissions() const override
	{
		// Wheel loaders use VLA trajectories instead of waypoint missions
		return false;
	}

	bool supportsRTL() const override
	{
		// No return-to-launch for wheel loaders (not critical like aircraft)
		return false;
	}

	const char *getAutomationTaskForMode(uint8_t operation_mode) const override
	{
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			return "vla_trajectory";

		default:
			return nullptr;
		}
	}

private:
	static constexpr int NUM_SUPPORTED_MODES = 3;

	static void setRequirement(uint8_t nav_state, uint32_t &mode_requirement)
	{
		mode_requirement |= 1u << nav_state;
	}

	static constexpr OperationModeInfo _supported_modes[NUM_SUPPORTED_MODES] = {
		{
			.mode_id = vehicle_status_s::OPERATION_MODE_MANUAL,
			.name = "Manual",
			.is_auto_mode = false,
			.requires_position = false,
			.requires_manual_input = true,
			.fallback_mode = vehicle_status_s::OPERATION_MODE_MANUAL
		},
		{
			.mode_id = vehicle_status_s::OPERATION_MODE_AUTO_VLA,
			.name = "VLA Auto",
			.is_auto_mode = true,
			.requires_position = true,
			.requires_manual_input = false,
			.fallback_mode = vehicle_status_s::OPERATION_MODE_MANUAL
		},
		{
			.mode_id = vehicle_status_s::OPERATION_MODE_TERMINATION,
			.name = "Termination",
			.is_auto_mode = false,
			.requires_position = false,
			.requires_manual_input = false,
			.fallback_mode = vehicle_status_s::OPERATION_MODE_TERMINATION
		}
	};
};

} // namespace vehicle_type
