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
 * @file FixedWingStrategy.hpp
 *
 * Fixed Wing Vehicle Type Strategy
 */

#pragma once

#include "VehicleTypeStrategy.hpp"

namespace vehicle_type
{

/**
 * @brief Strategy implementation for fixed wing vehicles
 *
 * Fixed wing aircraft support altitude and position control modes,
 * along with standard mission-based autonomous flight.
 *
 * Command Set:
 * - Full aircraft command set
 * - Takeoff/land commands
 * - Navigation commands
 * - Mission commands
 * - Gimbal/camera commands
 *
 * Event Reactions:
 * - RC loss: RTL
 * - Datalink loss: RTL
 * - Low battery: RTL
 * - Geofence breach: RTL
 */
class FixedWingStrategy : public VehicleTypeStrategy
{
public:
	//========================================================================
	// Basic Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	}

	const char *getName() const override
	{
		return "Fixed Wing";
	}

	//========================================================================
	// Mode Configuration
	//========================================================================

	uint32_t getAvailableModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_ALTCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LOITER) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LAND);
	}

	uint32_t getAvailableAutomationTasksMask() const override
	{
		return (1u << vehicle_type_config_s::AUTOMATION_TASK_MISSION) |
		       (1u << vehicle_type_config_s::AUTOMATION_TASK_LOITER) |
		       (1u << vehicle_type_config_s::AUTOMATION_TASK_RTL) |
		       (1u << vehicle_type_config_s::AUTOMATION_TASK_TAKEOFF) |
		       (1u << vehicle_type_config_s::AUTOMATION_TASK_LAND);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_POSCTL;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_AUTO_RTL;
	}

	uint8_t getModeChangeLogic() const override
	{
		return vehicle_type_config_s::MODE_CHANGE_LOGIC_STANDARD;
	}

	//========================================================================
	// Control Capabilities and Safety
	//========================================================================

	ControlCapabilities getControlCapabilities() const override
	{
		ControlCapabilities caps{};
		caps.altitude_control = true;
		caps.position_control = true;
		caps.velocity_control = true;
		caps.attitude_control = true;
		caps.manual_control = true;
		caps.autonomous_control = true;
		caps.boom_control = false;
		caps.tilt_control = false;
		caps.articulated_steering = false;
		return caps;
	}

	SafetyLimits getSafetyLimits() const override
	{
		SafetyLimits limits{};
		limits.emergency_stop_decel = 5.0f;   // m/s^2 (limited for fixed wing)
		limits.max_velocity = 50.0f;           // m/s
		limits.max_steering_rate = 1.0f;       // rad/s
		return limits;
	}

	//========================================================================
	// Command Set Configuration
	//========================================================================

	uint32_t getSupportedCommandsMask() const override
	{
		return (1u << vehicle_type_config_s::CMD_CATEGORY_ARM_DISARM) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_TAKEOFF_LAND) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_NAVIGATION) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_MISSION) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_GEOFENCE) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_GIMBAL) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_ACTUATOR) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_PAYLOAD);
	}

	bool isCommandSupported(uint16_t command) const override
	{
		switch (command) {
		// Arm/disarm
		case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
			return true;

		// Takeoff/land
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
			return true;

		// Navigation
		case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
		case vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT:
		case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM:
			return true;

		// Mission
		case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		case vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE:
			return true;

		// Mode change
		case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
			return true;

		// Gimbal/Camera
		case vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL:
		case vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE:
		case vehicle_command_s::VEHICLE_CMD_IMAGE_STOP_CAPTURE:
			return true;

		// Actuator
		case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
			return true;

		// VTOL commands not supported for pure fixed wing
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
			return false;

		default:
			// For fixed wing, delegate unknown commands to default handler
			// rather than rejecting them outright
			return false;
		}
	}

	bool shouldRejectCommand(const vehicle_command_s &command) const override
	{
		switch (command.command) {
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
			return true;  // Not a VTOL

		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
			return true;  // Not a VTOL

		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
			return true;  // No precision landing for fixed wing

		default:
			return false;  // Accept most commands
		}
	}

	uint8_t getTargetModeForCommand(uint16_t command) const override
	{
		switch (command) {
		case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		case vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE:
			return vehicle_status_s::OPERATION_MODE_AUTO_LOITER;

		case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
			return vehicle_status_s::OPERATION_MODE_AUTO_RTL;

		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
			return vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF;

		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
			return vehicle_status_s::OPERATION_MODE_AUTO_LAND;

		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
			// Fixed wing uses loiter for orbit-like behavior
			return vehicle_status_s::OPERATION_MODE_AUTO_LOITER;

		case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
			// Figure 8 uses loiter mode
			return vehicle_status_s::OPERATION_MODE_AUTO_LOITER;

		case vehicle_command_s::VEHICLE_CMD_MISSION_START:
			return vehicle_status_s::OPERATION_MODE_AUTO_MISSION;

		case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:
			return vehicle_status_s::OPERATION_MODE_TERMINATION;

		case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
		case vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE:
			// Mode specified in params
			return vehicle_status_s::OPERATION_MODE_MAX;

		default:
			return vehicle_status_s::OPERATION_MODE_MAX;  // No mode change
		}
	}

	//========================================================================
	// Event Reaction Configuration
	//========================================================================

	uint32_t getEventReactionsMask() const override
	{
		return (1u << vehicle_type_config_s::EVENT_REACT_RC_LOSS_RTL) |
		       (1u << vehicle_type_config_s::EVENT_REACT_DATALINK_LOSS_RTL) |
		       (1u << vehicle_type_config_s::EVENT_REACT_LOW_BATTERY_RTL) |
		       (1u << vehicle_type_config_s::EVENT_REACT_GEOFENCE_RTL);
	}

	EventAction getEventReaction(EventType event) const override
	{
		switch (event) {
		case EventType::RcLoss:
			return EventAction::SwitchToRtl;

		case EventType::DatalinkLoss:
			return EventAction::SwitchToRtl;

		case EventType::LowBattery:
			return EventAction::SwitchToRtl;

		case EventType::CriticalBattery:
			return EventAction::SwitchToLand;

		case EventType::GeofenceBreach:
			return EventAction::SwitchToRtl;

		case EventType::PositionLoss:
			return EventAction::SwitchToManual;

		case EventType::EmergencyStop:
			return EventAction::Disarm;

		default:
			return EventAction::Warn;
		}
	}

	//========================================================================
	// RC Input Configuration
	//========================================================================

	uint8_t getRcInputMode() const override
	{
		return vehicle_type_config_s::RC_INPUT_MODE_STANDARD;
	}
};

} // namespace vehicle_type
