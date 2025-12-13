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
 *
 * Automation subclass for rover/ground vehicles.
 */

#pragma once

#include "AutomationBase.hpp"

/**
 * @class AutomationRover
 *
 * Rover/ground vehicle specific Automation implementation.
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
		TaskBase* selected_task = nullptr;

		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
			selected_task = &_rtl;
			logTaskSelection("RTL", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
			selected_task = &_mission_task;
			logTaskSelection("Mission", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
			// Rover loiter = hold position
			selected_task = &_loiter;
			logTaskSelection("Hold", true);
			break;

		// Aerial operations not available for rovers
		case vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF:
			logTaskSelection("Takeoff (ground vehicle)", false);
			selected_task = nullptr;
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_LAND:
			logTaskSelection("Land (ground vehicle)", false);
			selected_task = nullptr;
			break;

		default:
			selected_task = nullptr;
			break;
		}

		return selected_task;
	}

	//========================================================================
	// Task Availability
	//========================================================================

	bool isTaskAvailable(uint8_t task_type) const override
	{
		switch (task_type) {
		// Ground-based tasks
		case vehicle_type_config_s::AUTOMATION_TASK_RTL:
		case vehicle_type_config_s::AUTOMATION_TASK_LOITER:
		case vehicle_type_config_s::AUTOMATION_TASK_MISSION:
			return true;

		// Aerial tasks not supported
		case vehicle_type_config_s::AUTOMATION_TASK_TAKEOFF:
		case vehicle_type_config_s::AUTOMATION_TASK_LAND:
		case vehicle_type_config_s::AUTOMATION_TASK_PRECLAND:
			return false;

		// VLA is wheel loader specific
		case vehicle_type_config_s::AUTOMATION_TASK_VLA:
			return false;

		default:
			return true;
		}
	}

	TaskBase* getDefaultTask() override
	{
		return &_loiter;  // Hold position
	}

	TaskBase* getFailsafeTask() override
	{
		return &_loiter;  // Hold position (rovers stop)
	}

	//========================================================================
	// Vehicle-specific parameters
	//========================================================================

	float getAcceptanceRadius() const override
	{
		// Rover waypoint acceptance
		return _param_nav_acc_rad.get();
	}

	float getCruiseSpeed() const override
	{
		// Rover cruise speed
		return _param_mpc_xy_cruise.get();
	}
};
