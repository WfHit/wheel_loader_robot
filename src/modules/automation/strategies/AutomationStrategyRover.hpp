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
 * @file AutomationStrategyRover.hpp
 *
 * Automation strategy for rover (ground) vehicles.
 * Handles ground-based autonomous operations.
 */

#pragma once

#include "AutomationStrategyBase.hpp"

namespace automation_strategy
{

class AutomationStrategyRover : public AutomationStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::Rover;
	}

	const char* getName() const override
	{
		return "Rover Automation Strategy";
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

		// Check if task is available for rover
		if (!isTaskAvailable(requested)) {
			result.rejection_reason = "Task not available for rover";
			result.result = TaskResult::NotSupported;
			return result;
		}

		// Check prerequisites
		if (!checkRoverPrerequisites(context, requested)) {
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
		// Rover specific tasks
		case TaskType::Waypoint:
		case TaskType::Mission:
		case TaskType::FollowTarget:
		case TaskType::RTL:          // Return to launch point
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

		// Aerial tasks NOT available
		case TaskType::Takeoff:
		case TaskType::Land:
		case TaskType::Loiter:       // Rovers don't loiter in air
		case TaskType::Orbit:        // Rovers could orbit but typically don't
		// Wheel loader specific tasks NOT available
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
		return TaskType::Hold;  // Rovers stop in place on failsafe
	}

	//========================================================================
	// Task Transition Rules
	//========================================================================

	TaskResult canTransition(TaskType from_task, TaskType to_task,
				 const TaskSelectionContext& context) const override
	{
		// Emergency tasks can always be entered
		if (to_task == TaskType::EmergencyStop || to_task == TaskType::Failsafe ||
		    to_task == TaskType::Hold) {
			return TaskResult::Success;
		}

		// Cannot transition from emergency stop without explicit release
		if (from_task == TaskType::EmergencyStop && to_task != TaskType::Idle) {
			return TaskResult::InvalidTransition;
		}

		// Can always go to idle or manual
		if (to_task == TaskType::Idle || to_task == TaskType::Manual) {
			return TaskResult::Success;
		}

		// Must be armed for active tasks
		if (!context.is_armed) {
			if (to_task != TaskType::Initialize && to_task != TaskType::Calibrate) {
				return TaskResult::PrerequisitesNotMet;
			}
		}

		// Mission requires valid mission
		if (to_task == TaskType::Mission && !context.has_valid_mission) {
			return TaskResult::PrerequisitesNotMet;
		}

		// Position tasks require valid position
		if ((to_task == TaskType::Waypoint || to_task == TaskType::RTL ||
		     to_task == TaskType::FollowTarget) && !context.has_valid_position) {
			return TaskResult::PrerequisitesNotMet;
		}

		return TaskResult::Success;
	}

	uint32_t getTaskPrerequisites(TaskType task) const override
	{
		uint32_t prereqs = 0;

		switch (task) {
		case TaskType::Mission:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID | PREREQ_MISSION_VALID;
			break;

		case TaskType::Waypoint:
		case TaskType::RTL:
		case TaskType::FollowTarget:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Offboard:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Manual:
		case TaskType::Idle:
		case TaskType::Hold:
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
		       context.has_valid_mission;
	}

	TaskType getNextAutonomousTask(TaskType current_task, bool task_complete) const override
	{
		if (!task_complete) {
			return current_task;  // Continue current task
		}

		// Rover autonomous state machine
		switch (current_task) {
		case TaskType::Idle:
		case TaskType::Initialize:
			return TaskType::Mission;  // Start mission directly (no takeoff needed)

		case TaskType::Mission:
			return TaskType::RTL;

		case TaskType::RTL:
			return TaskType::Idle;

		case TaskType::Waypoint:
			return TaskType::Hold;  // Hold after reaching waypoint

		case TaskType::FollowTarget:
			return current_task;  // Continue following

		case TaskType::EmergencyStop:
		case TaskType::Failsafe:
			return TaskType::Hold;  // Stop and hold

		default:
			return TaskType::Idle;
		}
	}

	uint64_t getAvailableTasksMask() const override
	{
		return (1ULL << static_cast<uint8_t>(TaskType::None)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Idle)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Initialize)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Calibrate)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Waypoint)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Mission)) |
		       (1ULL << static_cast<uint8_t>(TaskType::RTL)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Hold)) |
		       (1ULL << static_cast<uint8_t>(TaskType::FollowTarget)) |
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

	bool checkRoverPrerequisites(const TaskSelectionContext& context, TaskType task) const
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

		return true;
	}
};

} // namespace automation_strategy
