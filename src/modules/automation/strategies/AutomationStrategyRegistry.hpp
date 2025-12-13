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
 * @file AutomationStrategyRegistry.hpp
 *
 * Registry and factory for Automation vehicle strategies
 */

#pragma once

#include "AutomationStrategyBase.hpp"
#include "AutomationStrategyWheelLoader.hpp"
#include "AutomationStrategyRotaryWing.hpp"
#include "AutomationStrategyFixedWing.hpp"
#include "AutomationStrategyRover.hpp"

namespace automation_strategy
{

/**
 * @class AutomationStrategyRegistry
 *
 * Factory class for obtaining vehicle-type-specific Automation strategies.
 * Uses static instances for efficiency (singleton pattern per strategy).
 */
class AutomationStrategyRegistry
{
public:
	/**
	 * Get strategy for a specific vehicle type
	 * @param type The vehicle type
	 * @return Pointer to the strategy, or nullptr if unsupported
	 */
	static const AutomationStrategyBase* getStrategy(VehicleType type)
	{
		switch (type) {
		case VehicleType::WheelLoader:
			return &_wheel_loader_strategy;

		case VehicleType::RotaryWing:
			return &_rotary_wing_strategy;

		case VehicleType::FixedWing:
			return &_fixed_wing_strategy;

		case VehicleType::Rover:
			return &_rover_strategy;

		default:
			return nullptr;
		}
	}

	/**
	 * Get strategy from vehicle_type integer parameter
	 * @param vehicle_type_param Value from SYS_VEHICLE_TYPE parameter
	 * @return Pointer to the strategy, or nullptr if unsupported
	 */
	static const AutomationStrategyBase* getStrategyFromParam(int vehicle_type_param)
	{
		VehicleType type = vehicleTypeFromParam(vehicle_type_param);
		return getStrategy(type);
	}

	/**
	 * Get default strategy (rotary wing as the most common)
	 */
	static const AutomationStrategyBase* getDefaultStrategy()
	{
		return &_rotary_wing_strategy;
	}

	/**
	 * Check if a vehicle type is supported
	 */
	static bool isSupported(VehicleType type)
	{
		return getStrategy(type) != nullptr;
	}

	/**
	 * Get the wheel loader strategy directly
	 */
	static const AutomationStrategyWheelLoader& wheelLoader()
	{
		return _wheel_loader_strategy;
	}

	/**
	 * Get the rotary wing strategy directly
	 */
	static const AutomationStrategyRotaryWing& rotaryWing()
	{
		return _rotary_wing_strategy;
	}

	/**
	 * Get the fixed wing strategy directly
	 */
	static const AutomationStrategyFixedWing& fixedWing()
	{
		return _fixed_wing_strategy;
	}

	/**
	 * Get the rover strategy directly
	 */
	static const AutomationStrategyRover& rover()
	{
		return _rover_strategy;
	}

private:
	// Static strategy instances (singleton per type)
	static AutomationStrategyWheelLoader _wheel_loader_strategy;
	static AutomationStrategyRotaryWing _rotary_wing_strategy;
	static AutomationStrategyFixedWing _fixed_wing_strategy;
	static AutomationStrategyRover _rover_strategy;

	// Prevent instantiation
	AutomationStrategyRegistry() = delete;
	~AutomationStrategyRegistry() = delete;
};

} // namespace automation_strategy
