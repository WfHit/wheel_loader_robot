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
 * @file StandardCommandHandler.hpp
 *
 * Standard command handler for generic/aerial vehicles.
 *
 * This provides the default command handling logic for multicopters,
 * fixed wings, and other aerial vehicles. Vehicle-specific handlers
 * can inherit from this to reuse common logic.
 */

#pragma once

#include "VehicleCommandHandler.hpp"
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command_ack.h>

namespace command_processor
{

/**
 * @brief Standard command handler for aerial vehicles
 *
 * Provides default handling for common commands like mode changes,
 * navigation commands, and automation tasks.
 */
class StandardCommandHandler : public VehicleCommandHandler
{
public:
	StandardCommandHandler() = default;
	virtual ~StandardCommandHandler() = default;

	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	const char *getName() const override
	{
		return "Standard";
	}

	bool supportsCommand(uint16_t command) const override;

	bool shouldRejectCommand(const vehicle_command_s &cmd) const override
	{
		(void)cmd;
		return false;  // Standard handler accepts all commands by default
	}

	CommandResult processCommand(const vehicle_command_s &cmd,
				     CommandContext &ctx) const override;

	uint8_t getTargetModeForCommand(uint16_t command) const override;

	uint8_t getAutomationTaskForCommand(uint16_t command) const override;

protected:
	// Mode change command handlers
	virtual CommandResult handleDoSetMode(const vehicle_command_s &cmd,
					      CommandContext &ctx) const;

	virtual CommandResult handleSetNavState(const vehicle_command_s &cmd,
						CommandContext &ctx) const;

	virtual CommandResult handleDoSetStandardMode(const vehicle_command_s &cmd,
						      CommandContext &ctx) const;

	// Navigation command handlers
	virtual CommandResult handleNavTakeoff(const vehicle_command_s &cmd,
					       CommandContext &ctx) const;

	virtual CommandResult handleNavLand(const vehicle_command_s &cmd,
					    CommandContext &ctx) const;

	virtual CommandResult handleNavRtl(const vehicle_command_s &cmd,
					   CommandContext &ctx) const;

	virtual CommandResult handleMissionStart(const vehicle_command_s &cmd,
						 CommandContext &ctx) const;

	virtual CommandResult handleDoReposition(const vehicle_command_s &cmd,
						 CommandContext &ctx) const;

	virtual CommandResult handleDoChangeAltitude(const vehicle_command_s &cmd,
						     CommandContext &ctx) const;

	virtual CommandResult handleDoOrbit(const vehicle_command_s &cmd,
					    CommandContext &ctx) const;

	virtual CommandResult handleNavPrecland(const vehicle_command_s &cmd,
						CommandContext &ctx) const;

	virtual CommandResult handleNavVtolTakeoff(const vehicle_command_s &cmd,
						   CommandContext &ctx) const;

	virtual CommandResult handleDoFigureEight(const vehicle_command_s &cmd,
						  CommandContext &ctx) const;

	// Safety command handlers
	virtual CommandResult handleDoFlighttermination(const vehicle_command_s &cmd,
							CommandContext &ctx) const;

	// Home position handler
	virtual CommandResult handleDoSetHome(const vehicle_command_s &cmd,
					      CommandContext &ctx) const;

	// System command handlers
	virtual CommandResult handleComponentArmDisarm(const vehicle_command_s &cmd,
						       CommandContext &ctx) const;

	virtual CommandResult handlePreflightCalibration(const vehicle_command_s &cmd,
							 CommandContext &ctx) const;

	virtual CommandResult handlePreflightRebootShutdown(const vehicle_command_s &cmd,
							    CommandContext &ctx) const;

	virtual CommandResult handlePreflightStorage(const vehicle_command_s &cmd,
						     CommandContext &ctx) const;

	virtual CommandResult handleFixedMagCalYaw(const vehicle_command_s &cmd,
						   CommandContext &ctx) const;

	virtual CommandResult handleActuatorTest(const vehicle_command_s &cmd,
						 CommandContext &ctx) const;

	virtual CommandResult handleRunPrearmChecks(const vehicle_command_s &cmd,
						    CommandContext &ctx) const;

	// Helper to parse DO_SET_MODE parameters
	uint8_t parseDoSetModeParams(const vehicle_command_s &cmd) const;
};

} // namespace command_processor
