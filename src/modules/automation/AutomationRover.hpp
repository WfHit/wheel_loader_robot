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
 * @file AutomationRover.hpp
 * @brief Automation subclass for rover/ground vehicles
 *
 * This class implements the vehicle-specific automation behavior for ground-based
 * rovers. It supports ground navigation tasks while rejecting aerial operations.
 *
 * Supported operations:
 * - RTL (Return to Launch) - drives back to launch position
 * - Mission - follows ground waypoints
 * - Loiter/Hold - stops and holds current position
 *
 * Unsupported operations:
 * - Takeoff/Land - not applicable for ground vehicles
 * - Precision Landing - aerial only
 * - VLA - wheel loader specific
 */

#pragma once

#include "AutomationBase.hpp"

/**
 * @class AutomationRover
 * @brief Rover/ground vehicle specific Automation implementation
 *
 * Key characteristics:
 * - Ground-only navigation
 * - Hold position as default/failsafe behavior
 * - No altitude-based operations
 */
class AutomationRover : public AutomationBase
{
public:
	AutomationRover() : AutomationBase() {}
	~AutomationRover() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROVER;
	}

	const char* getVehicleTypeName() const override
	{
		return "Rover";
	}

	//========================================================================
	// Task Selection
	//========================================================================

	TaskBase* selectVehicleSpecificTask(uint8_t operation_mode) override
	{
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
			logTaskSelection("RTL", true);
			return &_rtl;

		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
			logTaskSelection("Mission", true);
			return &_mission_task;

		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
			logTaskSelection("Hold", true);
			return &_loiter;

		case vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF:
		case vehicle_status_s::OPERATION_MODE_AUTO_LAND:
			logTaskSelection("Aerial task (not supported)", false);
			return nullptr;

		default:
			return nullptr;
		}
	}

	//========================================================================
	// Task Availability
	//========================================================================

	bool isTaskAvailable(uint8_t task_type) const override
	{
		switch (task_type) {
		// Ground-based tasks - supported
		case vehicle_type_config_s::AUTOMATION_TASK_RTL:
		case vehicle_type_config_s::AUTOMATION_TASK_LOITER:
		case vehicle_type_config_s::AUTOMATION_TASK_MISSION:
			return true;

		// Aerial and vehicle-specific tasks - not supported
		case vehicle_type_config_s::AUTOMATION_TASK_TAKEOFF:
		case vehicle_type_config_s::AUTOMATION_TASK_LAND:
		case vehicle_type_config_s::AUTOMATION_TASK_PRECLAND:
		case vehicle_type_config_s::AUTOMATION_TASK_VLA:
			return false;

		default:
			return true;
		}
	}

	TaskBase* getDefaultTask() override
	{
		return &_loiter;
	}

	TaskBase* getFailsafeTask() override
	{
		return &_loiter;
	}

	//========================================================================
	// Vehicle-specific Parameters
	//========================================================================

	float getAcceptanceRadius() const override
	{
		return _param_nav_acc_rad.get();
	}

	float getCruisingSpeed() const override
	{
		return _param_mpc_xy_cruise.get();
	}
};
