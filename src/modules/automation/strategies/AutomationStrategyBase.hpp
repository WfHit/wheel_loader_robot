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
 * @file AutomationStrategyBase.hpp
 *
 * Base class for vehicle-type-specific Automation strategies.
 * Each vehicle type implements task selection and autonomous behavior logic.
 */

#pragma once

#include <lib/vehicle_strategy/VehicleStrategyTypes.hpp>
#include <uORB/topics/vehicle_status.h>

using namespace vehicle_strategy;

namespace automation_strategy
{

/**
 * Task types supported by automation
 */
enum class TaskType : uint8_t {
	None = 0,
	Idle,
	Initialize,
	Calibrate,

	// Wheel Loader specific
	LoadCycle,
	Transport,
	Dump,
	Approach,
	Dig,
	ReturnToStart,

	// Aerial vehicle specific
	Takeoff,
	Land,
	RTL,
	Loiter,
	Orbit,
	Waypoint,
	Mission,

	// Common
	Hold,
	FollowTarget,
	Offboard,
	Manual,

	// Safety
	EmergencyStop,
	Failsafe
};

/**
 * Context for task selection
 */
struct TaskSelectionContext {
	uint8_t current_task;              // Current active task
	uint8_t requested_task;            // Requested new task
	bool is_armed;                     // Armed state
	bool is_in_transition;             // In transition between tasks
	bool has_valid_position;           // Position estimate valid
	bool has_valid_mission;            // Mission loaded and valid
	bool is_on_ground;                 // On ground vs in air/motion
	float battery_level;               // Battery state of charge (0-1)

	// Vehicle-specific context (can be extended)
	union {
		struct {
			bool bucket_attached;
			bool load_detected;
			float bucket_angle;
		} wheel_loader;

		struct {
			float altitude_agl;
			bool vtol_in_fw_mode;
		} aerial;
	};
};

/**
 * Result of task selection
 */
struct TaskSelectionResult {
	TaskType selected_task;
	TaskResult result;
	bool task_changed;
	const char* rejection_reason;      // Human-readable reason if rejected
};

/**
 * @class AutomationStrategyBase
 *
 * Abstract base class for vehicle-type-specific Automation strategies.
 * Handles task selection, autonomous behavior logic, and task execution rules.
 */
class AutomationStrategyBase
{
public:
	virtual ~AutomationStrategyBase() = default;

	//========================================================================
	// Vehicle Type Information
	//========================================================================

	/**
	 * Get the vehicle type this strategy handles
	 */
	virtual VehicleType getVehicleType() const = 0;

	/**
	 * Get human-readable name of this strategy
	 */
	virtual const char* getName() const = 0;

	//========================================================================
	// Task Selection
	//========================================================================

	/**
	 * Select the appropriate task based on context
	 * @param context Current automation context
	 * @return Task selection result
	 */
	virtual TaskSelectionResult selectTask(const TaskSelectionContext& context) const = 0;

	/**
	 * Check if a task is available for this vehicle type
	 * @param task The task to check
	 * @return true if the task can be executed
	 */
	virtual bool isTaskAvailable(TaskType task) const = 0;

	/**
	 * Get the default task for this vehicle type
	 * @return Default task type
	 */
	virtual TaskType getDefaultTask() const = 0;

	/**
	 * Get the failsafe task for this vehicle type
	 * @return Failsafe task type
	 */
	virtual TaskType getFailsafeTask() const = 0;

	//========================================================================
	// Task Transition Rules
	//========================================================================

	/**
	 * Check if transition between tasks is allowed
	 * @param from_task Current task
	 * @param to_task Requested task
	 * @param context Current context
	 * @return TaskResult indicating if transition is allowed
	 */
	virtual TaskResult canTransition(TaskType from_task, TaskType to_task,
					 const TaskSelectionContext& context) const = 0;

	/**
	 * Get required conditions for starting a task
	 * @param task The task to check
	 * @return Bitmask of required conditions
	 */
	virtual uint32_t getTaskPrerequisites(TaskType task) const = 0;

	//========================================================================
	// Autonomous Behavior
	//========================================================================

	/**
	 * Check if autonomous operation is allowed
	 * @param context Current context
	 * @return true if autonomous mode can be entered
	 */
	virtual bool canEnterAutonomous(const TaskSelectionContext& context) const = 0;

	/**
	 * Get the next task in an autonomous sequence
	 * @param current_task Current task
	 * @param task_complete Whether current task completed
	 * @return Next task in sequence
	 */
	virtual TaskType getNextAutonomousTask(TaskType current_task, bool task_complete) const = 0;

	//========================================================================
	// Utility
	//========================================================================

	/**
	 * Get mask of all available tasks for this vehicle type
	 */
	virtual uint64_t getAvailableTasksMask() const = 0;

protected:
	// Helper to check common prerequisites
	bool checkCommonPrerequisites(const TaskSelectionContext& context) const
	{
		// Must be armed for most tasks
		return context.is_armed;
	}
};

} // namespace automation_strategy
