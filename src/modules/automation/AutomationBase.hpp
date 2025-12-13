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
 * @file AutomationBase.hpp
 *
 * Base class for vehicle-type-specific Automation implementations.
 * REPLACES the original Automation class - does not extend it.
 */

#pragma once

#include "automation.h"

/**
 * @class AutomationBase
 *
 * Base class that REPLACES Automation with virtual hooks for vehicle-specific
 * task handling. Each vehicle type should create a subclass.
 *
 * Virtual methods to override:
 * - selectVehicleSpecificTask(): Main task selection logic
 * - isTaskAvailable(): Vehicle-specific task availability
 * - getDefaultTask(): Default task for this vehicle
 * - getFailsafeTask(): Failsafe task for this vehicle
 * - getVehicleTypeName(): Human-readable vehicle type name
 */
class AutomationBase : public Automation
{
public:
	AutomationBase();
	~AutomationBase() override = default;

	//========================================================================
	// Vehicle Type Information (pure virtual - must override)
	//========================================================================

	/**
	 * Get the vehicle type this subclass handles
	 * @return vehicle_status_s::VEHICLE_TYPE_* constant
	 */
	virtual uint8_t getVehicleType() const = 0;

	/**
	 * Get human-readable name of this vehicle type
	 * @return Vehicle type name string
	 */
	virtual const char* getVehicleTypeName() const = 0;

	//========================================================================
	// Task Selection (pure virtual - must override)
	//========================================================================

	/**
	 * Vehicle-specific task selection implementation.
	 * Called to select the appropriate navigation task.
	 *
	 * @param operation_mode Current operation mode
	 * @return Pointer to the selected task, or nullptr
	 */
	virtual TaskBase* selectVehicleSpecificTask(uint8_t operation_mode) = 0;

	//========================================================================
	// Task Availability (virtual with default behavior)
	//========================================================================

	/**
	 * Check if a specific task is available for this vehicle type.
	 *
	 * @param task_type The task type to check
	 * @return true if task is available
	 */
	virtual bool isTaskAvailable(uint8_t task_type) const;

	/**
	 * Get mask of available tasks for this vehicle type.
	 *
	 * @return Bitmask of available tasks
	 */
	virtual uint64_t getAvailableTasksMask() const;

	/**
	 * Get the default task for this vehicle type.
	 *
	 * @return Pointer to default task
	 */
	virtual TaskBase* getDefaultTask();

	/**
	 * Get the failsafe task for this vehicle type.
	 *
	 * @return Pointer to failsafe task
	 */
	virtual TaskBase* getFailsafeTask();

	//========================================================================
	// Geofence (virtual with default behavior)
	//========================================================================

	/**
	 * Get geofence behavior for this vehicle type
	 * Different vehicles may respond differently to geofence breaches
	 */
	virtual uint8_t getGeofenceAction() const;

	//========================================================================
	// Vehicle-specific parameters (virtual with default behavior)
	//========================================================================

	/**
	 * Get vehicle-specific acceptance radius
	 */
	virtual float getAcceptanceRadius() const;

	/**
	 * Get vehicle-specific cruising speed
	 */
	virtual float getCruisingSpeed() const;

protected:
	/**
	 * Log task selection for debugging
	 */
	void logTaskSelection(const char* task_name, bool success) const;
};

/**
 * @class AutomationFactory
 *
 * Factory for creating vehicle-type-specific Automation instances.
 */
class AutomationFactory
{
public:
	/**
	 * Create an Automation instance based on vehicle type parameter.
	 *
	 * @param vehicle_type_param Value from SYS_VEHICLE_TYPE parameter
	 * @return Pointer to newly allocated Automation subclass, or nullptr on failure
	 */
	static AutomationBase* create(int vehicle_type_param);

	/**
	 * Create using current parameter value
	 */
	static AutomationBase* createFromParam();

private:
	AutomationFactory() = delete;
};
