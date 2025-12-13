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
 * @file ModeManagerBase.cpp
 *
 * Base class implementation for vehicle-type-specific ModeManager
 */

#include "ModeManagerBase.hpp"
#include <px4_platform_common/log.h>
#include <parameters/param.h>

ModeManagerBase::ModeManagerBase()
	: ModeManager()
{
}

//============================================================================
// Mode Availability - Default Implementation
//============================================================================

bool ModeManagerBase::isModeAvailable(uint8_t operation_mode) const
{
	return (getAvailableModesMask() & (1ULL << operation_mode)) != 0;
}

uint64_t ModeManagerBase::getAvailableModesMask() const
{
	// Default: all modes available
	return UINT64_MAX;
}

ModeIndex ModeManagerBase::getDefaultModeIndex() const
{
	return ModeIndex::ManualPosition;
}

ModeIndex ModeManagerBase::getFailsafeModeIndex() const
{
	return ModeIndex::Failsafe;
}

//============================================================================
// Mode Transition - Default Implementation
//============================================================================

bool ModeManagerBase::isTransitionAllowed(uint8_t from_mode, uint8_t to_mode) const
{
	(void)from_mode;
	(void)to_mode;
	// Default: all transitions allowed
	return true;
}

ModeIndex ModeManagerBase::getModeIndexForOperationMode(uint8_t operation_mode) const
{
	// Default mapping - subclasses should override for vehicle-specific mapping
	switch (operation_mode) {
	case vehicle_status_s::OPERATION_MODE_MANUAL:
		return ModeIndex::ManualPosition;

	case vehicle_status_s::OPERATION_MODE_ALTCTL:
		return ModeIndex::ManualAltitude;

	case vehicle_status_s::OPERATION_MODE_POSCTL:
		return ModeIndex::ManualPosition;

	case vehicle_status_s::OPERATION_MODE_AUTO_LOITER:
	case vehicle_status_s::OPERATION_MODE_AUTO_MISSION:
	case vehicle_status_s::OPERATION_MODE_AUTO_RTL:
		return ModeIndex::Auto;

	case vehicle_status_s::OPERATION_MODE_DESCEND:
		return ModeIndex::Descend;

	default:
		return ModeIndex::None;
	}
}

//============================================================================
// Protected Helpers
//============================================================================

bool ModeManagerBase::switchModeWithFallback(ModeIndex mode_index, ModeIndex fallback_index)
{
	ModeError error = switchMode(mode_index);

	if (error == ModeError::NoError) {
		return true;
	}

	PX4_WARN("Mode %d failed, trying fallback", (int)mode_index);
	error = switchMode(fallback_index);

	return (error == ModeError::NoError);
}

void ModeManagerBase::logModeSelection(const char* mode_name, bool success) const
{
	if (success) {
		PX4_DEBUG("%s: switched to %s", getVehicleTypeName(), mode_name);

	} else {
		PX4_WARN("%s: failed to switch to %s", getVehicleTypeName(), mode_name);
	}
}

//============================================================================
// Factory Implementation
//============================================================================

// Forward declarations
class ModeManagerWheelLoader;
class ModeManagerRotaryWing;
class ModeManagerFixedWing;
class ModeManagerRover;

#include "ModeManagerWheelLoader.hpp"
#include "ModeManagerRotaryWing.hpp"
#include "ModeManagerFixedWing.hpp"
#include "ModeManagerRover.hpp"

ModeManagerBase* ModeManagerFactory::create(int vehicle_type_param)
{
	switch (vehicle_type_param) {
	case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
		return new ModeManagerWheelLoader();

	case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
		return new ModeManagerRotaryWing();

	case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
		return new ModeManagerFixedWing();

	case vehicle_status_s::VEHICLE_TYPE_ROVER:
		return new ModeManagerRover();

	default:
		PX4_WARN("Unknown vehicle type %d, defaulting to rotary wing", vehicle_type_param);
		return new ModeManagerRotaryWing();
	}
}

ModeManagerBase* ModeManagerFactory::createFromParam()
{
	int32_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	param_t param = param_find("SYS_VEHICLE_TYPE");

	if (param != PARAM_INVALID) {
		param_get(param, &vehicle_type);
	}

	return create(vehicle_type);
}
