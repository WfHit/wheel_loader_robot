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
 * @file AutomationRotaryWing.hpp
 *
 * Automation subclass for rotary wing (multicopter) vehicles.
 */

#pragma once

#include "AutomationBase.hpp"

/**
 * @class AutomationRotaryWing
 *
 * Rotary wing (multicopter) specific Automation implementation.
 */
class AutomationRotaryWing : public AutomationBase
{
public:
	AutomationRotaryWing() : AutomationBase() {}
	~AutomationRotaryWing() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	const char* getVehicleTypeName() const override
	{
		return "Rotary Wing";
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

		case vehicle_status_s::OPERATION_MODE_AUTO_PRECLAND:
			selected_task = &_precland;
			logTaskSelection("Precision Land", true);
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

	uint64_t getAvailableTasksMask() const override
	{
		return UINT64_MAX;  // All tasks available
	}

	bool isTaskAvailable(uint8_t task_type) const override
	{
		// Rotary wing supports all aerial tasks
		switch (task_type) {
		case vehicle_type_config_s::AUTOMATION_TASK_TAKEOFF:
		case vehicle_type_config_s::AUTOMATION_TASK_LAND:
		case vehicle_type_config_s::AUTOMATION_TASK_RTL:
		case vehicle_type_config_s::AUTOMATION_TASK_LOITER:
		case vehicle_type_config_s::AUTOMATION_TASK_MISSION:
		case vehicle_type_config_s::AUTOMATION_TASK_PRECLAND:
			return true;

		// VLA is wheel loader specific
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
		return &_rtl;
	}

	//========================================================================
	// Geofence
	//========================================================================

	uint8_t getGeofenceAction() const override
	{
		// Use configured geofence action
		return _geofence.getGeofenceAction();
	}
};
