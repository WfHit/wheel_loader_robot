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
 * @file SystemManagerBase.cpp
 *
 * Base class implementation for vehicle-type-specific SystemManager
 */

#include "SystemManagerBase.hpp"
#include <px4_platform_common/log.h>
#include <parameters/param.h>

SystemManagerBase::SystemManagerBase()
	: SystemManager()
{
}

//============================================================================
// Command Handling - Default Implementation
//============================================================================

bool SystemManagerBase::shouldRejectCommand(const vehicle_command_s &cmd) const
{
	// Default: check against unsupported commands list
	uint16_t unsupported[32];
	int count = getUnsupportedCommands(unsupported, 32);

	return isCommandInList(cmd.command, unsupported, count);
}

int SystemManagerBase::getUnsupportedCommands(uint16_t* commands, int max_commands) const
{
	// Default: no commands are rejected
	(void)commands;
	(void)max_commands;
	return 0;
}

//============================================================================
// Failsafe Behavior - Default Implementation
//============================================================================

SystemManagerBase::FailsafeAction
SystemManagerBase::getFailsafeAction(uint8_t event, const vehicle_status_s& current_state) const
{
	(void)event;
	(void)current_state;
	// Default: return to launch for aerial, hold for ground
	return FailsafeAction::RTL;
}

bool SystemManagerBase::supportsFailsafeAction(FailsafeAction action) const
{
	// Default: support common actions
	switch (action) {
	case FailsafeAction::None:
	case FailsafeAction::Warn:
	case FailsafeAction::Hold:
	case FailsafeAction::ManualTakeover:
		return true;

	default:
		return false;
	}
}

//============================================================================
// Arming Logic - Default Implementation
//============================================================================

bool SystemManagerBase::vehicleSpecificPrearmCheck(const char** reason) const
{
	// Default: no additional checks
	(void)reason;
	return true;
}

uint32_t SystemManagerBase::getRequiredArmingChecksMask() const
{
	// Default: standard checks
	return 0xFFFFFFFF;  // All checks enabled
}

//============================================================================
// State Management - Default Implementation
//============================================================================

void SystemManagerBase::updateVehicleSpecificState()
{
	// Default: no additional state updates
}

void SystemManagerBase::initVehicleSpecific()
{
	// Default: no additional initialization
}

//============================================================================
// Mode Availability - Default Implementation
//============================================================================

uint64_t SystemManagerBase::getAvailableModesMask() const
{
	// Default: all modes available
	return UINT64_MAX;
}

bool SystemManagerBase::isModeAvailable(uint8_t operation_mode) const
{
	return (getAvailableModesMask() & (1ULL << operation_mode)) != 0;
}

uint8_t SystemManagerBase::getDefaultMode() const
{
	return vehicle_status_s::OPERATION_MODE_MANUAL;
}

uint8_t SystemManagerBase::getFailsafeMode() const
{
	return vehicle_status_s::OPERATION_MODE_AUTO_RTL;
}

//============================================================================
// Protected Helpers
//============================================================================

bool SystemManagerBase::isCommandInList(uint16_t cmd, const uint16_t* list, int list_size) const
{
	for (int i = 0; i < list_size; i++) {
		if (list[i] == cmd) {
			return true;
		}
	}

	return false;
}

void SystemManagerBase::logCommandRejection(const vehicle_command_s &cmd, const char* reason) const
{
	PX4_INFO("Command %d rejected for %s: %s",
		 cmd.command, getVehicleTypeName(), reason);
}

//============================================================================
// Factory Implementation
//============================================================================

// Forward declarations for vehicle-specific subclasses
class SystemManagerWheelLoader;
class SystemManagerRotaryWing;
class SystemManagerFixedWing;
class SystemManagerRover;

#include "SystemManagerWheelLoader.hpp"
#include "SystemManagerRotaryWing.hpp"
#include "SystemManagerFixedWing.hpp"
#include "SystemManagerRover.hpp"

SystemManagerBase* SystemManagerFactory::create(int vehicle_type_param)
{
	switch (vehicle_type_param) {
	case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
		return new SystemManagerWheelLoader();

	case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
		return new SystemManagerRotaryWing();

	case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
		return new SystemManagerFixedWing();

	case vehicle_status_s::VEHICLE_TYPE_ROVER:
		return new SystemManagerRover();

	default:
		PX4_WARN("Unknown vehicle type %d, defaulting to rotary wing", vehicle_type_param);
		return new SystemManagerRotaryWing();
	}
}

SystemManagerBase* SystemManagerFactory::createFromParam()
{
	int32_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	param_t param = param_find("SYS_VEHICLE_TYPE");

	if (param != PARAM_INVALID) {
		param_get(param, &vehicle_type);
	}

	return create(vehicle_type);
}
