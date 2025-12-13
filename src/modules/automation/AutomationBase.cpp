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
 * @file AutomationBase.cpp
 *
 * Base class implementation for vehicle-type-specific Automation
 */

#include "AutomationBase.hpp"
#include <px4_platform_common/log.h>
#include <parameters/param.h>

AutomationBase::AutomationBase()
	: Automation()
{
}

//============================================================================
// Task Availability - Default Implementation
//============================================================================

bool AutomationBase::isTaskAvailable(uint8_t task_type) const
{
	return (getAvailableTasksMask() & (1ULL << task_type)) != 0;
}

uint64_t AutomationBase::getAvailableTasksMask() const
{
	// Default: all tasks available
	return UINT64_MAX;
}

TaskBase* AutomationBase::getDefaultTask()
{
	// Default: loiter
	return &_loiter;
}

TaskBase* AutomationBase::getFailsafeTask()
{
	// Default: RTL
	return &_rtl;
}

//============================================================================
// Geofence - Default Implementation
//============================================================================

uint8_t AutomationBase::getGeofenceAction() const
{
	// Default geofence action - use parameter
	return _geofence.getGeofenceAction();
}

//============================================================================
// Vehicle-specific parameters - Default Implementation
//============================================================================

float AutomationBase::getAcceptanceRadius() const
{
	return get_acceptance_radius();
}

float AutomationBase::getCruisingSpeed() const
{
	return get_cruising_speed();
}

//============================================================================
// Protected Helpers
//============================================================================

void AutomationBase::logTaskSelection(const char* task_name, bool success) const
{
	if (success) {
		PX4_DEBUG("%s: switched to task %s", getVehicleTypeName(), task_name);

	} else {
		PX4_WARN("%s: failed to switch to task %s", getVehicleTypeName(), task_name);
	}
}

//============================================================================
// Factory Implementation
//============================================================================

// Forward declarations
class AutomationWheelLoader;
class AutomationRotaryWing;
class AutomationFixedWing;
class AutomationRover;

#include "AutomationWheelLoader.hpp"
#include "AutomationRotaryWing.hpp"
#include "AutomationFixedWing.hpp"
#include "AutomationRover.hpp"

AutomationBase* AutomationFactory::create(int vehicle_type_param)
{
	switch (vehicle_type_param) {
	case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
		return new AutomationWheelLoader();

	case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
		return new AutomationRotaryWing();

	case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
		return new AutomationFixedWing();

	case vehicle_status_s::VEHICLE_TYPE_ROVER:
		return new AutomationRover();

	default:
		PX4_WARN("Unknown vehicle type %d, defaulting to rotary wing", vehicle_type_param);
		return new AutomationRotaryWing();
	}
}

AutomationBase* AutomationFactory::createFromParam()
{
	int32_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	param_t param = param_find("SYS_VEHICLE_TYPE");

	if (param != PARAM_INVALID) {
		param_get(param, &vehicle_type);
	}

	return create(vehicle_type);
}
