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
 * @file wheel_loader_command_handler.hpp
 *
 * Wheel loader specific command handler.
 *
 * This provides command handling specific to wheel loader vehicles,
 * including VLA trajectory control, boom/tilt operations, and
 * ground vehicle specific behavior.
 */

#pragma once

#include "airplane_command_handler.hpp"

namespace command_processor
{

/**
 * @brief Wheel loader specific command handler
 *
 * Handles commands for wheel loader vehicles with:
 * - VLA 7-DOF trajectory control
 * - Boom and tilt control
 * - Ground vehicle specific behaviors (no takeoff/land/RTL)
 * - Articulated steering
 */
class WheelLoaderCommandHandler : public AirPlaneCommandHandler
{
public:
	WheelLoaderCommandHandler() = default;
	virtual ~WheelLoaderCommandHandler() = default;

	uint8_t getVehicleType() const override
	{
		return vehicle_identity_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char *getName() const override
	{
		return "WheelLoader";
	}

	bool supportsCommand(uint16_t command) const override;

	bool shouldRejectCommand(const vehicle_command_s &cmd) const override;

	CommandResult processCommand(const vehicle_command_s &cmd,
				     CommandContext &ctx) const override;

	uint8_t getTargetModeForCommand(uint16_t command) const override;

	uint8_t getAutomationTaskForCommand(uint16_t command) const override;

protected:
	// Override aerial navigation commands to reject them
	CommandResult handleNavTakeoff(const vehicle_command_s &cmd,
				       CommandContext &ctx) const override;

	CommandResult handleNavLand(const vehicle_command_s &cmd,
				    CommandContext &ctx) const override;

	CommandResult handleNavRtl(const vehicle_command_s &cmd,
				   CommandContext &ctx) const override;

	CommandResult handleDoOrbit(const vehicle_command_s &cmd,
				    CommandContext &ctx) const override;

	CommandResult handleNavPrecland(const vehicle_command_s &cmd,
					CommandContext &ctx) const override;

	CommandResult handleNavVtolTakeoff(const vehicle_command_s &cmd,
					   CommandContext &ctx) const override;

	// Wheel loader specific commands
	CommandResult handleDoReposition(const vehicle_command_s &cmd,
					 CommandContext &ctx) const override;

	// VLA trajectory command
	CommandResult handleVlaTrajectory(const vehicle_command_s &cmd,
					  CommandContext &ctx) const;
};

} // namespace command_processor
