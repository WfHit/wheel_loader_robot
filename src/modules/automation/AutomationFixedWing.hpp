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
 * @file AutomationFixedWing.hpp
 *
 * Automation subclass for fixed wing vehicles.
 */

#pragma once

#include "AutomationBase.hpp"

/**
 * @class AutomationFixedWing
 *
 * Fixed wing specific Automation implementation.
 */
class AutomationFixedWing : public AutomationBase
{
public:
	AutomationFixedWing() : AutomationBase() {}
	~AutomationFixedWing() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	}

	const char* getVehicleTypeName() const override
	{
		return "Fixed Wing";
	}

	//========================================================================
	// Task Selection
	//========================================================================

	TaskBase* selectVehicleSpecificTask(uint8_t operation_mode) override
	{
		TaskBase* selected_task = nullptr;

		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF:
			selected_task = &_takeoff;
			logTaskSelection("Takeoff", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_LAND:
			selected_task = &_land;
			logTaskSelection("Land", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
			selected_task = &_rtl;
			logTaskSelection("RTL", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
			selected_task = &_mission_task;
			logTaskSelection("Mission", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
			selected_task = &_loiter;
			logTaskSelection("Loiter", true);
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
		case vehicle_type_config_s::AUTOMATION_TASK_TAKEOFF:
		case vehicle_type_config_s::AUTOMATION_TASK_LAND:
		case vehicle_type_config_s::AUTOMATION_TASK_RTL:
		case vehicle_type_config_s::AUTOMATION_TASK_LOITER:
		case vehicle_type_config_s::AUTOMATION_TASK_MISSION:
			return true;

		// Precision landing typically not supported
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
		return &_loiter;  // Circle loiter
	}

	TaskBase* getFailsafeTask() override
	{
		return &_rtl;  // RTL with circle
	}

	//========================================================================
	// Vehicle-specific parameters
	//========================================================================

	float getAcceptanceRadius() const override
	{
		// Fixed wing needs larger acceptance radius
		return _param_nav_acc_rad.get();
	}
};
