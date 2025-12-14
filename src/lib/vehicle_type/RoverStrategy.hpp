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
 * @file RoverStrategy.hpp
 *
 * Rover (Ground Vehicle) Vehicle Type Strategy
 */

#pragma once

#include "VehicleTypeStrategy.hpp"

namespace vehicle_type
{

/**
 * @brief Strategy implementation for rover/ground vehicles
 *
 * Rovers are wheeled ground vehicles that support mission-based navigation,
 * position control, and return-to-launch capabilities.
 *
 * Command Set:
 * - Arm/disarm commands
 * - Navigation commands (waypoints, RTL)
 * - Mission commands
 * - Geofence commands
 *
 * Event Reactions:
 * - RC loss: RTL or hold
 * - Datalink loss: Continue mission
 * - Geofence breach: Hold position
 * - Obstacle detection: Stop
 */
class RoverStrategy : public VehicleTypeStrategy
{
public:
	//========================================================================
	// Basic Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROVER;
	}

	const char *getName() const override
	{
		return "Rover";
	}

	//========================================================================
	// Mode Configuration
	//========================================================================

	uint32_t getAvailableModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_POSCTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_MISSION) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_RTL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_LOITER);
	}

	uint32_t getAvailableAutomationTasksMask() const override
	{
		return (1u << vehicle_type_config_s::AUTOMATION_TASK_MISSION) |
		       (1u << vehicle_type_config_s::AUTOMATION_TASK_LOITER) |
		       (1u << vehicle_type_config_s::AUTOMATION_TASK_RTL);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_AUTO_RTL;
	}

	uint8_t getModeChangeLogic() const override
	{
		return vehicle_type_config_s::MODE_CHANGE_LOGIC_ROVER;
	}

	//========================================================================
	// Control Capabilities and Safety
	//========================================================================

	ControlCapabilities getControlCapabilities() const override
	{
		ControlCapabilities caps{};
		caps.altitude_control = false;
		caps.position_control = true;
		caps.velocity_control = true;
		caps.attitude_control = false;
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
		limits.emergency_stop_decel = 3.0f;  // m/s^2
		limits.max_velocity = 10.0f;          // m/s
		limits.max_steering_rate = 1.0f;      // rad/s
		return limits;
	}

	//========================================================================
	// Command Set Configuration
	//========================================================================

	uint32_t getSupportedCommandsMask() const override
	{
		return (1u << vehicle_type_config_s::CMD_CATEGORY_ARM_DISARM) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_NAVIGATION) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_MISSION) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_GEOFENCE) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_ACTUATOR);
	}

	bool isCommandSupported(uint16_t command) const override
	{
		switch (command) {
		// Arm/disarm commands
		case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
			return true;

		// Navigation commands
		case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		case vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT:
		case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
		case vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM:
			return true;

		// Mission commands
		case vehicle_command_s::VEHICLE_CMD_MISSION_START:
		case vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE:
			return true;

		// Mode change
		case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
			return true;

		// Actuator test
		case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
			return true;

		// Takeoff/land not supported for ground vehicles
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
			return false;

		default:
			return false;
		}
	}

	bool shouldRejectCommand(const vehicle_command_s &command) const override
	{
		// Rovers don't support aerial commands
		switch (command.command) {
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_PRECLAND:
		case vehicle_command_s::VEHICLE_CMD_DO_ORBIT:
		case vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT:
		case vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION:
			return true;  // Reject aerial commands

		default:
			return false;  // Allow all other commands
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

		case vehicle_command_s::VEHICLE_CMD_MISSION_START:
			return vehicle_status_s::OPERATION_MODE_AUTO_MISSION;

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
		       (1u << vehicle_type_config_s::EVENT_REACT_DATALINK_LOSS_CONTINUE) |
		       (1u << vehicle_type_config_s::EVENT_REACT_LOW_BATTERY_RTL) |
		       (1u << vehicle_type_config_s::EVENT_REACT_GEOFENCE_HOLD) |
		       (1u << vehicle_type_config_s::EVENT_REACT_OBSTACLE_STOP);
	}

	EventAction getEventReaction(EventType event) const override
	{
		switch (event) {
		case EventType::RcLoss:
			return EventAction::SwitchToRtl;

		case EventType::DatalinkLoss:
			return EventAction::ContinueMission;

		case EventType::LowBattery:
			return EventAction::SwitchToRtl;

		case EventType::CriticalBattery:
			return EventAction::EmergencyStop;

		case EventType::GeofenceBreach:
			return EventAction::SwitchToHold;

		case EventType::ObstacleDetected:
			return EventAction::EmergencyStop;

		case EventType::PositionLoss:
			return EventAction::SwitchToManual;

		case EventType::EmergencyStop:
			return EventAction::EmergencyStop;

		default:
			return EventAction::Warn;
		}
	}

	//========================================================================
	// RC Input Configuration
	//========================================================================

	uint8_t getRcInputMode() const override
	{
		return vehicle_type_config_s::RC_INPUT_MODE_ROVER;
	}
};

} // namespace vehicle_type
