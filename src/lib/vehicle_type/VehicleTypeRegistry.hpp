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
 * @file VehicleTypeRegistry.hpp
 *
 * Vehicle Type Strategy Registry
 *
 * This provides a singleton registry that maps vehicle type constants
 * to their corresponding strategy implementations.
 */

#pragma once

#include "VehicleTypeStrategy.hpp"
#include "WheelLoaderStrategy.hpp"
#include "RoverStrategy.hpp"
#include "RotaryWingStrategy.hpp"
#include "FixedWingStrategy.hpp"

namespace vehicle_type
{

/**
 * @brief Registry for vehicle type strategies
 *
 * This class provides access to vehicle type strategy instances
 * based on vehicle type constants. Strategies are allocated statically
 * to avoid dynamic memory allocation.
 */
class VehicleTypeRegistry
{
public:
	/**
	 * @brief Get the strategy for a specific vehicle type
	 * @param vehicle_type Vehicle type constant from vehicle_status_s
	 * @return Pointer to the appropriate strategy, or FixedWingStrategy as default
	 */
	static const VehicleTypeStrategy *getStrategy(uint8_t vehicle_type)
	{
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
			return &_wheel_loader_strategy;

		case vehicle_status_s::VEHICLE_TYPE_ROVER:
			return &_rover_strategy;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			return &_rotary_wing_strategy;

		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
		default:
			return &_fixed_wing_strategy;
		}
	}

	/**
	 * @brief Get all registered strategies
	 * @param strategies Output array of strategy pointers
	 * @param max_count Maximum number of strategies to return
	 * @return Number of strategies returned
	 */
	static size_t getAllStrategies(const VehicleTypeStrategy **strategies, size_t max_count)
	{
		const VehicleTypeStrategy *all_strategies[] = {
			&_wheel_loader_strategy,
			&_rover_strategy,
			&_rotary_wing_strategy,
			&_fixed_wing_strategy
		};

		size_t count = sizeof(all_strategies) / sizeof(all_strategies[0]);

		if (count > max_count) {
			count = max_count;
		}

		for (size_t i = 0; i < count; i++) {
			strategies[i] = all_strategies[i];
		}

		return count;
	}

	/**
	 * @brief Fill a vehicle_type_config_s message for a given vehicle type
	 * @param config Reference to the config message to fill
	 * @param vehicle_type Vehicle type constant
	 */
	static void fillConfig(vehicle_type_config_s &config, uint8_t vehicle_type)
	{
		const VehicleTypeStrategy *strategy = getStrategy(vehicle_type);
		strategy->fillConfig(config);
	}

	/**
	 * @brief Check if a mode is available for a vehicle type
	 * @param vehicle_type Vehicle type constant
	 * @param mode Operation mode to check
	 * @return true if mode is available
	 */
	static bool isModeAvailable(uint8_t vehicle_type, uint8_t mode)
	{
		return getStrategy(vehicle_type)->isModeAvailable(mode);
	}

	/**
	 * @brief Check if an automation task is available for a vehicle type
	 * @param vehicle_type Vehicle type constant
	 * @param task Automation task to check
	 * @return true if task is available
	 */
	static bool isAutomationTaskAvailable(uint8_t vehicle_type, uint8_t task)
	{
		return getStrategy(vehicle_type)->isAutomationTaskAvailable(task);
	}

private:
	// Static strategy instances (avoid dynamic allocation)
	static WheelLoaderStrategy _wheel_loader_strategy;
	static RoverStrategy _rover_strategy;
	static RotaryWingStrategy _rotary_wing_strategy;
	static FixedWingStrategy _fixed_wing_strategy;
};

} // namespace vehicle_type
