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
 * @file RotaryWingStrategy.hpp
 *
 * Vehicle Type Strategy for Rotary Wing (Multicopter/Helicopter) Aircraft
 *
 * Rotary wing aircraft have:
 * - 3D position + attitude control
 * - Manual, stabilized, altitude, position, and auto modes
 * - Mission, RTL, takeoff, land capabilities
 * - Attitude stabilization required even in manual mode
 */

#pragma once

#include "VehicleTypeStrategy.hpp"

namespace vehicle_type
{

class RotaryWingStrategy : public VehicleTypeStrategy
{
public:
	RotaryWingStrategy() = default;
	~RotaryWingStrategy() override = default;

	//========================================================================
	// Vehicle Identification
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	const char *getVehicleTypeName() const override
	{
		return "Rotary Wing";
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
		for (int i = 0; i < NUM_SUPPORTED_MODES; ++i) {
			if (_supported_modes[i].mode_id == operation_mode) {
				return true;
			}
		}

		return false;
	}

	uint32_t getValidOperationModesMask() const override
	{
		uint32_t mask = 0;

		for (int i = 0; i < NUM_SUPPORTED_MODES; ++i) {
			mask |= (1u << _supported_modes[i].mode_id);
		}

		return mask;
	}

	uint32_t getSelectableOperationModesMask() const override
	{
		// All modes except descend and termination can be selected
		return getValidOperationModesMask()
		       & ~(1u << vehicle_status_s::OPERATION_MODE_DESCEND)
		       & ~(1u << vehicle_status_s::OPERATION_MODE_TERMINATION);
	}

	//========================================================================
	// Mode Selection Logic
	//========================================================================

	ModeChangeConfig getModeChangeConfig() const override
	{
		ModeChangeConfig config{};
		config.allow_stick_override = true;
		config.require_disarm_for_mode_change = false;
		config.default_mode = vehicle_status_s::OPERATION_MODE_MANUAL;
		config.failsafe_mode = vehicle_status_s::OPERATION_MODE_DESCEND;
		config.manual_fallback_mode = vehicle_status_s::OPERATION_MODE_MANUAL;
		return config;
	}

	uint8_t selectMode(uint8_t requested_mode, uint8_t current_mode,
			   bool is_armed, bool has_position, bool has_manual_control) const override
	{
		// For rotary wing, check mode requirements
		switch (requested_mode) {
		case vehicle_status_s::OPERATION_MODE_POSCTL:
		case vehicle_status_s::OPERATION_MODE_POSITION_SLOW:
			if (!has_position) {
				return has_manual_control ? vehicle_status_s::OPERATION_MODE_ALTCTL
				       : vehicle_status_s::OPERATION_MODE_MANUAL;
			}

			if (!has_manual_control) {
				return current_mode;
			}

			return requested_mode;

		case vehicle_status_s::OPERATION_MODE_ALTCTL:
			if (!has_manual_control) {
				return current_mode;
			}

			return requested_mode;

		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
		case vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF:
		case vehicle_status_s::OPERATION_MODE_AUTO_LAND:
			if (!has_position) {
				return vehicle_status_s::OPERATION_MODE_DESCEND;
			}

			return requested_mode;

		case vehicle_status_s::OPERATION_MODE_MANUAL:
		case vehicle_status_s::OPERATION_MODE_ACRO:
		case vehicle_status_s::OPERATION_MODE_STAB:
			if (!has_manual_control && is_armed) {
				return current_mode;
			}

			return requested_mode;

		default:
			if (isModeSupported(requested_mode)) {
				return requested_mode;
			}

			return vehicle_status_s::OPERATION_MODE_MANUAL;
		}
	}

	uint8_t getFallbackMode(uint8_t failed_mode, bool is_armed) const override
	{
		if (!is_armed) {
			return vehicle_status_s::OPERATION_MODE_MANUAL;
		}

		// For rotary wing, need to maintain flight - descend is safest fallback
		switch (failed_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
			return vehicle_status_s::OPERATION_MODE_DESCEND;

		case vehicle_status_s::OPERATION_MODE_POSCTL:
			return vehicle_status_s::OPERATION_MODE_ALTCTL;

		case vehicle_status_s::OPERATION_MODE_ALTCTL:
			return vehicle_status_s::OPERATION_MODE_MANUAL;

		default:
			return vehicle_status_s::OPERATION_MODE_DESCEND;
		}
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
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_STAB:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_ALTCTL:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_POSCTL:
		case vehicle_status_s::OPERATION_MODE_POSITION_SLOW:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_position_enabled = true;
			control_mode.flag_control_velocity_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
		case vehicle_status_s::OPERATION_MODE_AUTO_LAND:
		case vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND:
		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
		case vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF:
		case vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF:
			control_mode.flag_control_auto_enabled = true;
			control_mode.flag_control_position_enabled = true;
			control_mode.flag_control_velocity_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_ACRO:
			control_mode.flag_control_manual_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_DESCEND:
			control_mode.flag_control_auto_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_TERMINATION:
			control_mode.flag_control_termination_enabled = true;
			break;

		case vehicle_status_s::OPERATION_MODE_ORBIT:
		case vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET:
			control_mode.flag_control_manual_enabled = false;
			control_mode.flag_control_auto_enabled = false;
			control_mode.flag_control_position_enabled = true;
			control_mode.flag_control_velocity_enabled = true;
			control_mode.flag_control_altitude_enabled = true;
			control_mode.flag_control_climb_rate_enabled = true;
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;

		default:
			// Default to stabilized mode
			control_mode.flag_control_attitude_enabled = true;
			control_mode.flag_control_rates_enabled = true;
			control_mode.flag_control_allocation_enabled = true;
			break;
		}
	}

	bool requiresStabilization() const override
	{
		// Rotary wing always needs stabilization
		return true;
	}

	//========================================================================
	// Mode Requirements
	//========================================================================

	void setModeRequirements(failsafe_flags_s &flags) const override
	{
		// OPERATION_MODE_MANUAL
		setRequirement(vehicle_status_s::OPERATION_MODE_MANUAL, flags.mode_req_manual_control);

		// OPERATION_MODE_ALTCTL
		setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_ALTCTL, flags.mode_req_manual_control);

		// OPERATION_MODE_POSCTL
		setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_local_position_relaxed);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSCTL, flags.mode_req_manual_control);

		// OPERATION_MODE_POSITION_SLOW
		setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_local_position_relaxed);
		setRequirement(vehicle_status_s::OPERATION_MODE_POSITION_SLOW, flags.mode_req_manual_control);

		// OPERATION_MODE_AUTO_MISSION
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_global_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_mission);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_MISSION, flags.mode_req_wind_and_flight_time_compliance);

		// OPERATION_MODE_AUTO_LOITER
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_global_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LOITER, flags.mode_req_wind_and_flight_time_compliance);

		// OPERATION_MODE_AUTO_RTL
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_global_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_home_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_RTL, flags.mode_req_prevent_arming);

		// OPERATION_MODE_ACRO
		setRequirement(vehicle_status_s::OPERATION_MODE_ACRO, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_ACRO, flags.mode_req_manual_control);

		// OPERATION_MODE_DESCEND
		setRequirement(vehicle_status_s::OPERATION_MODE_DESCEND, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_DESCEND, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_DESCEND, flags.mode_req_prevent_arming);

		// OPERATION_MODE_TERMINATION
		setRequirement(vehicle_status_s::OPERATION_MODE_TERMINATION, flags.mode_req_prevent_arming);

		// OPERATION_MODE_STAB
		setRequirement(vehicle_status_s::OPERATION_MODE_STAB, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_STAB, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_STAB, flags.mode_req_manual_control);

		// OPERATION_MODE_AUTO_TAKEOFF
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, flags.mode_req_local_alt);

		// OPERATION_MODE_AUTO_LAND
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_local_position_relaxed);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_LAND, flags.mode_req_prevent_arming);

		// OPERATION_MODE_AUTO_FOLLOW_TARGET
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_prevent_arming);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, flags.mode_req_wind_and_flight_time_compliance);

		// OPERATION_MODE_AUTO_PRECLAND
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, flags.mode_req_prevent_arming);

		// OPERATION_MODE_ORBIT
		setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_local_alt);
		setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_prevent_arming);
		setRequirement(vehicle_status_s::OPERATION_MODE_ORBIT, flags.mode_req_wind_and_flight_time_compliance);

		// OPERATION_MODE_AUTO_VTOL_TAKEOFF
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_angular_velocity);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_attitude);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_local_position);
		setRequirement(vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, flags.mode_req_local_alt);
	}

	//========================================================================
	// Failsafe Behavior
	//========================================================================

	uint8_t getFailsafeAction(uint8_t failure_type, uint8_t current_mode, bool is_armed) const override
	{
		if (!is_armed) {
			return vehicle_status_s::OPERATION_MODE_MANUAL;
		}

		// For rotary wing, need controlled descent as failsafe
		return vehicle_status_s::OPERATION_MODE_DESCEND;
	}

	bool isSafeStopMode(uint8_t operation_mode) const override
	{
		return operation_mode == vehicle_status_s::OPERATION_MODE_AUTO_LAND
		       || operation_mode == vehicle_status_s::OPERATION_MODE_DESCEND
		       || operation_mode == vehicle_status_s::OPERATION_MODE_TERMINATION;
	}

	//========================================================================
	// Automation Task Support
	//========================================================================

	bool supportsMissions() const override
	{
		return true;
	}

	bool supportsRTL() const override
	{
		return true;
	}

	const char *getAutomationTaskForMode(uint8_t operation_mode) const override
	{
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
			return "mission";

		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
			return "rtl";

		case vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF:
			return "takeoff";

		case vehicle_status_s::OPERATION_MODE_AUTO_LAND:
			return "land";

		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
			return "loiter";

		case vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND:
			return "precland";

		default:
			return nullptr;
		}
	}

private:
	static constexpr int NUM_SUPPORTED_MODES = 18;

	static void setRequirement(uint8_t nav_state, uint32_t &mode_requirement)
	{
		mode_requirement |= 1u << nav_state;
	}

	static constexpr OperationModeInfo _supported_modes[NUM_SUPPORTED_MODES] = {
		{vehicle_status_s::OPERATION_MODE_MANUAL, "Manual", false, false, true, vehicle_status_s::OPERATION_MODE_MANUAL},
		{vehicle_status_s::OPERATION_MODE_STAB, "Stabilized", false, false, true, vehicle_status_s::OPERATION_MODE_MANUAL},
		{vehicle_status_s::OPERATION_MODE_ALTCTL, "Altitude", false, false, true, vehicle_status_s::OPERATION_MODE_MANUAL},
		{vehicle_status_s::OPERATION_MODE_POSCTL, "Position", false, true, true, vehicle_status_s::OPERATION_MODE_ALTCTL},
		{vehicle_status_s::OPERATION_MODE_POSITION_SLOW, "Position Slow", false, true, true, vehicle_status_s::OPERATION_MODE_ALTCTL},
		{vehicle_status_s::OPERATION_MODE_ACRO, "Acro", false, false, true, vehicle_status_s::OPERATION_MODE_MANUAL},
		{vehicle_status_s::OPERATION_MODE_AUTO_MISSION, "Mission", true, true, false, vehicle_status_s::OPERATION_MODE_AUTO_LOITER},
		{vehicle_status_s::OPERATION_MODE_AUTO_LOITER, "Loiter", true, true, false, vehicle_status_s::OPERATION_MODE_DESCEND},
		{vehicle_status_s::OPERATION_MODE_AUTO_RTL, "RTL", true, true, false, vehicle_status_s::OPERATION_MODE_DESCEND},
		{vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF, "Takeoff", true, true, false, vehicle_status_s::OPERATION_MODE_AUTO_LOITER},
		{vehicle_status_s::OPERATION_MODE_AUTO_LAND, "Land", true, true, false, vehicle_status_s::OPERATION_MODE_DESCEND},
		{vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND, "Precision Land", true, true, false, vehicle_status_s::OPERATION_MODE_AUTO_LAND},
		{vehicle_status_s::OPERATION_MODE_AUTO_FOLLOW_TARGET, "Follow Target", true, true, false, vehicle_status_s::OPERATION_MODE_AUTO_LOITER},
		{vehicle_status_s::OPERATION_MODE_ORBIT, "Orbit", true, true, false, vehicle_status_s::OPERATION_MODE_AUTO_LOITER},
		{vehicle_status_s::OPERATION_MODE_AUTO_VTOL_TAKEOFF, "VTOL Takeoff", true, true, false, vehicle_status_s::OPERATION_MODE_AUTO_LOITER},
		{vehicle_status_s::OPERATION_MODE_DESCEND, "Descend", true, false, false, vehicle_status_s::OPERATION_MODE_TERMINATION},
		{vehicle_status_s::OPERATION_MODE_TERMINATION, "Termination", false, false, false, vehicle_status_s::OPERATION_MODE_TERMINATION},
		{vehicle_status_s::OPERATION_MODE_OFFBOARD, "Offboard", true, false, false, vehicle_status_s::OPERATION_MODE_DESCEND},
	};
};

} // namespace vehicle_type
