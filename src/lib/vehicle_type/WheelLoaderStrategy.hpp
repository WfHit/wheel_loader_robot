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
 * Wheel Loader Vehicle Type Strategy
 */

#pragma once

#include "VehicleTypeStrategy.hpp"

namespace vehicle_type
{

/**
 * @brief Strategy implementation for wheel loader vehicles
 *
 * Wheel loaders are articulated ground vehicles with boom and bucket (tilt) control.
 * They support manual control and VLA (Vision-Language-Action) autonomous mode.
 *
 * Command Set:
 * - Arm/disarm commands
 * - Boom/bucket control commands
 * - VLA trajectory commands
 * - Basic navigation (no takeoff/land)
 *
 * Event Reactions:
 * - RC loss: Emergency stop (ground vehicle safety)
 * - Datalink loss: Continue current task
 * - Obstacle detection: Stop
 * - VLA timeout: Switch to manual
 */
class WheelLoaderStrategy : public VehicleTypeStrategy
{
public:
	//========================================================================
	// Basic Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char *getName() const override
	{
		return "Wheel Loader";
	}

	//========================================================================
	// Mode Configuration
	//========================================================================

	uint32_t getAvailableModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_VLA);
	}

	uint32_t getAvailableAutomationTasksMask() const override
	{
		return (1u << vehicle_type_config_s::AUTOMATION_TASK_VLA_TRAJECTORY);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getModeChangeLogic() const override
	{
		return vehicle_type_config_s::MODE_CHANGE_LOGIC_WHEEL_LOADER;
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
		caps.boom_control = true;
		caps.tilt_control = true;
		caps.articulated_steering = true;
		return caps;
	}

	SafetyLimits getSafetyLimits() const override
	{
		SafetyLimits limits{};
		limits.emergency_stop_decel = 5.0f;  // m/s^2
		limits.max_velocity = 3.0f;           // m/s (slow for safety)
		limits.max_steering_rate = 0.5f;      // rad/s
		return limits;
	}

	//========================================================================
	// Command Set Configuration
	//========================================================================

	uint32_t getSupportedCommandsMask() const override
	{
		return (1u << vehicle_type_config_s::CMD_CATEGORY_ARM_DISARM) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_ACTUATOR) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_BOOM_BUCKET) |
		       (1u << vehicle_type_config_s::CMD_CATEGORY_VLA);
	}

	bool isCommandSupported(uint16_t command) const override
	{
		switch (command) {
		// Arm/disarm commands
		case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
			return true;

		// Actuator test
		case vehicle_command_s::VEHICLE_CMD_ACTUATOR_TEST:
			return true;

		// Mode change (filtered by available modes)
		case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
			return true;

		// TODO: Add VLA-specific MAVLink commands when defined
		// VLA trajectory commands would be added here once the
		// vehicle_command_s enum includes them, for example:
		// case vehicle_command_s::VEHICLE_CMD_VLA_START_TRAJECTORY:
		// case vehicle_command_s::VEHICLE_CMD_VLA_STOP:
		//     return true;

		// Takeoff/land not supported for ground vehicles
		case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
		case vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF:
			return false;

		default:
			return false;
		}
	}

	CommandResult handleCommand(const vehicle_command_s &command) const override
	{
		switch (command.command) {
		// Wheel loader specific command handling would go here
		// For now, delegate to default handler
		default:
			return CommandResult::Delegated;
		}
	}

	//========================================================================
	// Event Reaction Configuration
	//========================================================================

	uint32_t getEventReactionsMask() const override
	{
		return (1u << vehicle_type_config_s::EVENT_REACT_RC_LOSS_ESTOP) |
		       (1u << vehicle_type_config_s::EVENT_REACT_DATALINK_LOSS_CONTINUE) |
		       (1u << vehicle_type_config_s::EVENT_REACT_OBSTACLE_STOP);
	}

	EventAction getEventReaction(EventType event) const override
	{
		switch (event) {
		case EventType::RcLoss:
			// Ground vehicle safety: E-stop on RC loss
			return EventAction::EmergencyStop;

		case EventType::DatalinkLoss:
			// Continue current task on datalink loss
			return EventAction::ContinueMission;

		case EventType::LowBattery:
			// Switch to manual on low battery
			return EventAction::SwitchToManual;

		case EventType::CriticalBattery:
			// Emergency stop on critical battery
			return EventAction::EmergencyStop;

		case EventType::ObstacleDetected:
			// Stop immediately on obstacle detection
			return EventAction::EmergencyStop;

		case EventType::VlaTimeout:
			// Fall back to manual on VLA timeout
			return EventAction::SwitchToManual;

		case EventType::BoomLimitReached:
			// Just warn, operator handles it
			return EventAction::Warn;

		case EventType::EmergencyStop:
			// Acknowledge E-stop
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
		return vehicle_type_config_s::RC_INPUT_MODE_WHEEL_LOADER;
	}
};

} // namespace vehicle_type
