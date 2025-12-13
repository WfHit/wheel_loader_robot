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
 * @file SystemManagerStrategyRegistry.hpp
 *
 * Registry for SystemManager vehicle-type-specific strategies
 */

#pragma once

#include "SystemManagerStrategyBase.hpp"
#include "SystemManagerStrategyWheelLoader.hpp"
#include "SystemManagerStrategyRotaryWing.hpp"
#include "SystemManagerStrategyFixedWing.hpp"
#include "SystemManagerStrategyRover.hpp"

#include <uORB/topics/vehicle_status.h>

namespace system_manager_strategy
{

/**
 * @brief Registry for SystemManager strategies
 *
 * Provides factory method to get the appropriate strategy based on vehicle type.
 * Strategies are static singletons.
 */
class SystemManagerStrategyRegistry
{
public:
	/**
	 * @brief Get the strategy for a specific vehicle type
	 * @param vehicle_type Vehicle type from vehicle_status_s
	 * @return Pointer to the strategy (never null, returns RotaryWing as default)
	 */
	static const SystemManagerStrategyBase* getStrategy(uint8_t vehicle_type)
	{
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
			return &_wheel_loader_strategy;

		case vehicle_status_s::VEHICLE_TYPE_ROVER:
			return &_rover_strategy;

		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			return &_fixed_wing_strategy;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
		default:
			return &_rotary_wing_strategy;
		}
	}

	/**
	 * @brief Get all registered strategies
	 * @param strategies Output array of strategy pointers
	 * @param max_count Maximum number of strategies
	 * @return Number of strategies returned
	 */
	static size_t getAllStrategies(const SystemManagerStrategyBase** strategies, size_t max_count)
	{
		const SystemManagerStrategyBase* all[] = {
			&_wheel_loader_strategy,
			&_rover_strategy,
			&_rotary_wing_strategy,
			&_fixed_wing_strategy
		};

		size_t count = sizeof(all) / sizeof(all[0]);
		if (count > max_count) {
			count = max_count;
		}

		for (size_t i = 0; i < count; i++) {
			strategies[i] = all[i];
		}

		return count;
	}

private:
	static SystemManagerStrategyWheelLoader _wheel_loader_strategy;
	static SystemManagerStrategyRover _rover_strategy;
	static SystemManagerStrategyRotaryWing _rotary_wing_strategy;
	static SystemManagerStrategyFixedWing _fixed_wing_strategy;
};

} // namespace system_manager_strategy
