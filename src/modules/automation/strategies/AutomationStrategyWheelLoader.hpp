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
 * @file AutomationStrategyWheelLoader.hpp
 *
 * Automation strategy for wheel loader vehicles.
 * Handles load cycle, transport, and ground-based autonomous operations.
 */

#pragma once

#include "AutomationStrategyBase.hpp"

namespace automation_strategy
{

/**
 * Wheel loader specific task state machine
 */
enum class WheelLoaderTaskState : uint8_t {
	Idle = 0,
	Approaching,        // Moving to dig location
	Digging,            // Engaging with material
	Lifting,            // Lifting bucket
	Transporting,       // Moving to dump location
	Dumping,            // Dumping material
	Returning,          // Returning to start position
	EmergencyStop
};

class AutomationStrategyWheelLoader : public AutomationStrategyBase
{
public:
	//========================================================================
	// Vehicle Type Information
	//========================================================================

	VehicleType getVehicleType() const override
	{
		return VehicleType::WheelLoader;
	}

	const char* getName() const override
	{
		return "Wheel Loader Automation Strategy";
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

		// Check if task is available for wheel loader
		if (!isTaskAvailable(requested)) {
			result.rejection_reason = "Task not available for wheel loader";
			result.result = TaskResult::NotSupported;
			return result;
		}

		// Check prerequisites
		if (!checkWheelLoaderPrerequisites(context, requested)) {
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
		// Wheel loader specific tasks
		case TaskType::LoadCycle:
		case TaskType::Transport:
		case TaskType::Dump:
		case TaskType::Approach:
		case TaskType::Dig:
		case TaskType::ReturnToStart:
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
		case TaskType::RTL:
		case TaskType::Loiter:
		case TaskType::Orbit:
		case TaskType::Waypoint:
		case TaskType::Mission:
		case TaskType::FollowTarget:
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
		return TaskType::EmergencyStop;
	}

	//========================================================================
	// Task Transition Rules
	//========================================================================

	TaskResult canTransition(TaskType from_task, TaskType to_task,
				 const TaskSelectionContext& context) const override
	{
		// Emergency stop can always be entered
		if (to_task == TaskType::EmergencyStop || to_task == TaskType::Failsafe) {
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

		// Check if armed for active tasks
		if (!context.is_armed) {
			if (to_task != TaskType::Initialize && to_task != TaskType::Calibrate) {
				return TaskResult::PrerequisitesNotMet;
			}
		}

		// Load cycle specific transitions
		if (to_task == TaskType::LoadCycle) {
			// Need valid position for autonomous load cycle
			if (!context.has_valid_position) {
				return TaskResult::PrerequisitesNotMet;
			}
		}

		return TaskResult::Success;
	}

	uint32_t getTaskPrerequisites(TaskType task) const override
	{
		uint32_t prereqs = 0;

		switch (task) {
		case TaskType::LoadCycle:
		case TaskType::Approach:
		case TaskType::Transport:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Dig:
		case TaskType::Dump:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
			break;

		case TaskType::Manual:
		case TaskType::Idle:
			// No prerequisites
			break;

		case TaskType::Offboard:
			prereqs |= PREREQ_ARMED | PREREQ_POSITION_VALID;
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
		// Wheel loader autonomous requires:
		// - Armed
		// - Valid position
		// - Bucket attached (if applicable)
		return context.is_armed &&
		       context.has_valid_position &&
		       context.wheel_loader.bucket_attached;
	}

	TaskType getNextAutonomousTask(TaskType current_task, bool task_complete) const override
	{
		if (!task_complete) {
			return current_task;  // Continue current task
		}

		// Wheel loader load cycle state machine
		switch (current_task) {
		case TaskType::Idle:
		case TaskType::Initialize:
			return TaskType::Approach;

		case TaskType::Approach:
			return TaskType::Dig;

		case TaskType::Dig:
			return TaskType::Transport;

		case TaskType::Transport:
			return TaskType::Dump;

		case TaskType::Dump:
			return TaskType::ReturnToStart;

		case TaskType::ReturnToStart:
			return TaskType::Approach;  // Loop back for next cycle

		case TaskType::EmergencyStop:
		case TaskType::Failsafe:
			return current_task;  // Stay in emergency state

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
		       (1ULL << static_cast<uint8_t>(TaskType::LoadCycle)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Transport)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Dump)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Approach)) |
		       (1ULL << static_cast<uint8_t>(TaskType::Dig)) |
		       (1ULL << static_cast<uint8_t>(TaskType::ReturnToStart)) |
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
	static constexpr uint32_t PREREQ_BUCKET_ATTACHED = (1u << 2);
	static constexpr uint32_t PREREQ_ON_GROUND = (1u << 3);

	bool checkWheelLoaderPrerequisites(const TaskSelectionContext& context, TaskType task) const
	{
		uint32_t required = getTaskPrerequisites(task);

		if ((required & PREREQ_ARMED) && !context.is_armed) {
			return false;
		}

		if ((required & PREREQ_POSITION_VALID) && !context.has_valid_position) {
			return false;
		}

		if ((required & PREREQ_BUCKET_ATTACHED) && !context.wheel_loader.bucket_attached) {
			return false;
		}

		return true;
	}
};

} // namespace automation_strategy
