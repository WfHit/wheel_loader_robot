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
 * @file AutomationWheelLoader.hpp
 *
 * Automation subclass for wheel loader vehicles.
 * Handles VLA trajectory following and wheel loader specific tasks.
 */

#pragma once

#include "AutomationBase.hpp"

/**
 * @class AutomationWheelLoader
 *
 * Wheel loader specific Automation implementation.
 *
 * Key behaviors:
 * - Supports VLA trajectory following for autonomous operation
 * - No aerial flight tasks (takeoff, land, RTL)
 * - Ground-based navigation only
 */
class AutomationWheelLoader : public AutomationBase
{
public:
	AutomationWheelLoader() : AutomationBase() {}
	~AutomationWheelLoader() override = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char* getVehicleTypeName() const override
	{
		return "Wheel Loader";
	}

	//========================================================================
	// Task Selection
	//========================================================================

	TaskBase* selectVehicleSpecificTask(uint8_t operation_mode) override
	{
		TaskBase* selected_task = nullptr;

		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			// VLA 7-DOF Trajectory Following (chassis + boom + tilt)
			selected_task = &_vla_trajectory_task;
			logTaskSelection("VLA Trajectory", true);
			break;

		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
			// Mission execution for wheel loader
			selected_task = &_mission_task;
			logTaskSelection("Mission", true);
			break;

		case vehicle_status_s::OPERATION_MODE_MANUAL:
		default:
			// No automation task for manual mode
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
		// Wheel loaders support limited task set
		// Note: These would be task type enum values
		return (1ULL << 0) |  // None/Idle
		       (1ULL << 1) |  // VLA trajectory
		       (1ULL << 2);   // Mission (ground waypoints only)
	}

	bool isTaskAvailable(uint8_t task_type) const override
	{
		// Wheel loaders don't support aerial tasks
		switch (task_type) {
		case vehicle_type_config_s::AUTOMATION_TASK_VLA:
		case vehicle_type_config_s::AUTOMATION_TASK_MISSION:
			return true;

		case vehicle_type_config_s::AUTOMATION_TASK_TAKEOFF:
		case vehicle_type_config_s::AUTOMATION_TASK_LAND:
		case vehicle_type_config_s::AUTOMATION_TASK_RTL:
		case vehicle_type_config_s::AUTOMATION_TASK_LOITER:
		case vehicle_type_config_s::AUTOMATION_TASK_PRECLAND:
		default:
			return false;
		}
	}

	TaskBase* getDefaultTask() override
	{
		// Wheel loader default is no automation (manual control)
		return nullptr;
	}

	TaskBase* getFailsafeTask() override
	{
		// Wheel loader failsafe is to stop (no automation task)
		return nullptr;
	}

	//========================================================================
	// Geofence
	//========================================================================

	uint8_t getGeofenceAction() const override
	{
		// Wheel loaders should stop on geofence breach
		return 0;  // Geofence action: None (handled by SystemManager failsafe)
	}

	//========================================================================
	// Vehicle-specific parameters
	//========================================================================

	float getAcceptanceRadius() const override
	{
		// Wheel loaders need larger acceptance radius for ground navigation
		return 2.0f;  // 2 meters
	}

	float getCruisingSpeed() const override
	{
		// Wheel loader cruising speed (m/s)
		return 2.0f;  // 2 m/s typical for wheel loader
	}
};
