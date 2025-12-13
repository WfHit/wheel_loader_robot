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
 * @file WheelLoaderStrategy.hpp
 *
 * Wheel Loader Vehicle Type Strategy
 */

#pragma once

#include "VehicleTypeStrategy.hpp"

namespace vehicle_type
{

/**
 * @brief Strategy implementation for wheel loader vehicles
 *
 * Wheel loaders are articulated ground vehicles with boom and bucket (tilt) control.
 * They support manual control and VLA (Vision-Language-Action) autonomous mode.
 */
class WheelLoaderStrategy : public VehicleTypeStrategy
{
public:
	uint8_t getVehicleType() const override
	{
		return vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

	const char *getName() const override
	{
		return "Wheel Loader";
	}

	uint32_t getAvailableModesMask() const override
	{
		return (1u << vehicle_status_s::OPERATION_MODE_MANUAL) |
		       (1u << vehicle_status_s::OPERATION_MODE_AUTO_VLA);
	}

	uint32_t getAvailableAutomationTasksMask() const override
	{
		return (1u << vehicle_type_config_s::AUTOMATION_TASK_VLA_TRAJECTORY);
	}

	uint8_t getDefaultMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getFailsafeMode() const override
	{
		return vehicle_status_s::OPERATION_MODE_MANUAL;
	}

	uint8_t getModeChangeLogic() const override
	{
		return vehicle_type_config_s::MODE_CHANGE_LOGIC_WHEEL_LOADER;
	}

	ControlCapabilities getControlCapabilities() const override
	{
		ControlCapabilities caps{};
		caps.altitude_control = false;
		caps.position_control = true;
		caps.velocity_control = true;
		caps.attitude_control = false;
		caps.manual_control = true;
		caps.autonomous_control = true;
		caps.boom_control = true;
		caps.tilt_control = true;
		caps.articulated_steering = true;
		return caps;
	}

	SafetyLimits getSafetyLimits() const override
	{
		SafetyLimits limits{};
		limits.emergency_stop_decel = 5.0f;  // m/s^2
		limits.max_velocity = 3.0f;           // m/s (slow for safety)
		limits.max_steering_rate = 0.5f;      // rad/s
		return limits;
	}
};

} // namespace vehicle_type
