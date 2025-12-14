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
 * @brief Automation subclass for wheel loader vehicles
 *
 * This class implements the vehicle-specific automation behavior for articulated
 * wheel loaders. It supports VLA (Vision-Language-Action) trajectory following
 * for autonomous construction operations.
 *
 * Supported operations:
 * - VLA Trajectory Following: AI-guided 7-DOF control (chassis + boom + bucket)
 * - Mission: Ground waypoint following
 *
 * Unsupported operations:
 * - All aerial operations (takeoff, land, RTL, loiter, precision landing)
 *
 * The wheel loader uses a unique control paradigm with coordinated control of:
 * - Chassis: steering and propulsion
 * - Boom: lift arm position
 * - Bucket: tilt angle
 */

#pragma once

#include "AutomationBase.hpp"

/**
 * @class AutomationWheelLoader
 * @brief Wheel loader specific Automation implementation
 *
 * Key characteristics:
 * - Ground-only operations
 * - 7-DOF trajectory following (VLA mode)
 * - No automation in failsafe (stop and wait)
 * - Larger acceptance radius for ground navigation
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
		switch (operation_mode) {
		case vehicle_status_s::OPERATION_MODE_AUTO_VLA:
			logTaskSelection("VLA Trajectory", true);
			return &_vla_trajectory_task;

		case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
			logTaskSelection("Mission", true);
			return &_mission_task;

		case vehicle_status_s::OPERATION_MODE_MANUAL:
		default:
			// No automation task for manual mode
			return nullptr;
		}
	}

	//========================================================================
	// Task Availability
	//========================================================================

	uint64_t getAvailableTasksMask() const override
	{
		// Bit 0: None/Idle, Bit 1: VLA trajectory, Bit 2: Mission
		return (1ULL << 0) | (1ULL << 1) | (1ULL << 2);
	}

	bool isTaskAvailable(uint8_t task_type) const override
	{
		switch (task_type) {
		// Wheel loader specific tasks
		case vehicle_type_config_s::AUTOMATION_TASK_VLA:
		case vehicle_type_config_s::AUTOMATION_TASK_MISSION:
			return true;

		// All aerial tasks - not supported
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
		return nullptr;  // Manual control, no automation task
	}

	TaskBase* getFailsafeTask() override
	{
		return nullptr;  // Stop and wait for operator
	}

	//========================================================================
	// Geofence
	//========================================================================

	uint8_t getGeofenceAction() const override
	{
		// Geofence breach handled by SystemManager failsafe (emergency stop)
		return 0;
	}

	//========================================================================
	// Vehicle-specific Parameters
	//========================================================================

	float getAcceptanceRadius() const override
	{
		return 2.0f;  // 2 meters for ground navigation
	}

	float getCruisingSpeed() const override
	{
		return 2.0f;  // 2 m/s typical for wheel loader
	}
};
