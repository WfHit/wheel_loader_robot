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
 * @file AutomationStrategyFixedWing.hpp
 *
 * Automation strategy for fixed wing vehicles.
 * Handles aerial autonomous operations with fixed wing constraints.
 */

#pragma once

#include "AutomationStrategyBase.hpp"

namespace automation_strategy
{

class AutomationStrategyFixedWing : public AutomationStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::FixedWing;
	}

	const char* getName() const override
	{
		return "Fixed Wing Automation Strategy";
	}

	//========================================================================
	// Task Selection
	//========================================================================

	TaskSelectionResult selectTask(const TaskSelectionContext& context) const override
	{
		TaskSelectionResult result{};
		result.selected_task = TaskType::None;
		result.task_changed = false;
		result.rejection_reason = nullptr;
		result.result = TaskResult::Rejected;

		TaskType requested = static_cast<TaskType>(context.requested_task);

		// Check if task is available for fixed wing
		if (!isTaskAvailable(requested)) {
			result.rejection_reason = "Task not available for fixed wing";
			result.result = TaskResult::NotSupported;
			return result;
		}

		// Check prerequisites
		if (!checkFixedWingPrerequisites(context, requested)) {
			result.rejection_reason = "Prerequisites not met";
			result.result = TaskResult::PrerequisitesNotMet;
			return result;
		}

		// Check transition validity
		TaskType current = static_cast<TaskType>(context.current_task);
		TaskResult transition_result = canTransition(current, requested, context);

		if (transition_result != TaskResult::Success) {
			result.rejection_reason = "Invalid task transition";
			result.result = transition_result;
			return result;
		}

		result.selected_task = requested;
		result.task_changed = (requested != current);
		result.result = TaskResult::Success;
		return result;
	}

	bool isTaskAvailable(TaskType task) const override
	{
		switch (task) {
		// Fixed wing specific tasks (no hover capability)
		case TaskType::Takeoff:
		case TaskType::Land:
		case TaskType::RTL:
		case TaskType::Loiter:      // Circle loiter
		case TaskType::Waypoint:
		case TaskType::Mission:
		// Common tasks
		case TaskType::None:
		case TaskType::Idle:
		case TaskType::Initialize:
		case TaskType::Calibrate:
		case TaskType::Hold:
		case TaskType::Manual:
		case TaskType::Offboard:
		case TaskType::EmergencyStop:
		case TaskType::Failsafe:
			return true;

		// Orbit (point-centered) is more complex for fixed wing
		case TaskType::Orbit:
			return true;  // Supported but with different behavior than multicopter

		// Not supported for fixed wing
		case TaskType::FollowTarget:  // Requires hover or very slow flight
		case TaskType::LoadCycle:
		case TaskType::Transport:
		case TaskType::Dump:
		case TaskType::Approach:
		case TaskType::Dig:
		case TaskType::ReturnToStart:
		default:
			return false;
		}
	}

	TaskType getDefaultTask() const override
	{
		return TaskType::Idle;
	}

	TaskType getFailsafeTask() const override
	{
		return TaskType::RTL;  // Return to launch and circle
	}

	//========================================================================
	// Task Transition Rules
	//========================================================================

	TaskResult canTransition(TaskType from_task, TaskType to_task,
				 const TaskSelectionContext& context) const override
	{
		// Emergency tasks can always be entered
		if (to_task == TaskType::EmergencyStop || to_task == TaskType::Failsafe ||
		    to_task == TaskType::RTL || to_task == TaskType::Land) {
			return TaskResult::Success;
		}

		// Cannot transition from failsafe without explicit action
		if (from_task == TaskType::Failsafe && to_task != TaskType::Idle && to_task != TaskType::Land) {
			return TaskResult::InvalidTransition;
		}

		// Can always go to idle or manual
		if (to_task == TaskType::Idle || to_task == TaskType::Manual) {
			return TaskResult::Success;
		}

		// Must be armed for active flight tasks
		if (!context.is_armed) {
			if (to_task != TaskType::Initialize && to_task != TaskType::Calibrate) {
				return TaskResult::PrerequisitesNotMet;
			}
		}

		// Fixed wing specific: Takeoff requires being on ground
		if (to_task == TaskType::Takeoff && !context.is_on_ground) {
			return TaskResult::InvalidTransition;
		}

		// Mission requires valid mission
		if (to_task == TaskType::Mission && !context.has_valid_mission) {
			return TaskResult::PrerequisitesNotMet;
		}

		// Fixed wing cannot immediately go to Loiter from takeoff (need airspeed)
		if (from_task == TaskType::Takeoff && to_task == TaskType::Loiter) {
			// Check if we have sufficient altitude
			if (context.aerial.altitude_agl < MIN_LOITER_ALTITUDE) {
				return TaskResult::PrerequisitesNotMet;
			}
		}

		return TaskResult::Success;
	}

	uint32_t getTaskPrerequisites(TaskType task) const override
	{
		uint32_t prereqs = 0;

		switch (task) {
		case TaskType::Takeoff:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID | PREREQ_ON_GROUND;
			break;

		case TaskType::Land:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Mission:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID | PREREQ_MISSION_VALID;
			break;

		case TaskType::RTL:
		case TaskType::Loiter:
		case TaskType::Orbit:
		case TaskType::Waypoint:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Offboard:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Manual:
		case TaskType::Idle:
			// No prerequisites
			break;

		default:
			break;
		}

		return prereqs;
	}

	//========================================================================
	// Autonomous Behavior
	//========================================================================

	bool canEnterAutonomous(const TaskSelectionContext& context) const override
	{
		return context.is_armed &&
		       context.has_valid_position &&
		       (context.has_valid_mission || !context.is_on_ground);
	}

	TaskType getNextAutonomousTask(TaskType current_task, bool task_complete) const override
	{
		if (!task_complete) {
			return current_task;  // Continue current task
		}

		// Fixed wing autonomous state machine
		switch (current_task) {
		case TaskType::Idle:
		case TaskType::Initialize:
			return TaskType::Takeoff;

		case TaskType::Takeoff:
			return TaskType::Mission;

		case TaskType::Mission:
			return TaskType::RTL;

		case TaskType::RTL:
			return TaskType::Land;

		case TaskType::Land:
			return TaskType::Idle;

		case TaskType::Loiter:
		case TaskType::Orbit:
			return current_task;  // Stay in loiter/orbit until commanded

		case TaskType::EmergencyStop:
		case TaskType::Failsafe:
			return TaskType::RTL;  // Try to RTL and land

		default:
			return TaskType::Loiter;  // Fixed wing can loiter (circle)
		}
	}

	uint64_t getAvailableTasksMask() const override
	{
		return (1ULL << static_cast<uint8_t>(TaskType::None)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Idle)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Initialize)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Calibrate)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Takeoff)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Land)) |
		       (1ULL << static_cast<uint8_t>(TaskType::RTL)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Loiter)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Orbit)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Waypoint)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Mission)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Hold)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Manual)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Offboard)) |
		       (1ULL << static_cast<uint8_t>(TaskType::EmergencyStop)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Failsafe));
	}

private:
	// Prerequisite bit flags
	static constexpr uint32_t PREREQ_ARMED = (1u << 0);
	static constexpr uint32_t PREREQ_POSITION_VALID = (1u << 1);
	static constexpr uint32_t PREREQ_MISSION_VALID = (1u << 2);
	static constexpr uint32_t PREREQ_ON_GROUND = (1u << 3);

	// Fixed wing specific constants
	static constexpr float MIN_LOITER_ALTITUDE = 30.0f;  // meters AGL

	bool checkFixedWingPrerequisites(const TaskSelectionContext& context, TaskType task) const
	{
		uint32_t required = getTaskPrerequisites(task);

		if ((required & PREREQ_ARMED) && !context.is_armed) {
			return false;
		}

		if ((required & PREREQ_POSITION_VALID) && !context.has_valid_position) {
			return false;
		}

		if ((required & PREREQ_MISSION_VALID) && !context.has_valid_mission) {
			return false;
		}

		if ((required & PREREQ_ON_GROUND) && !context.is_on_ground) {
			return false;
		}

		return true;
	}
};

} // namespace automation_strategy
