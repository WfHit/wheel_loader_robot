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
 * @file VehicleStrategyRegistry.hpp
 *
 * Central registry for all module-specific vehicle strategies
 *
 * This registry provides access to strategy instances for each module
 * (ModeManager, Automation) based on vehicle type.
 * Strategies are allocated statically as singletons.
 */

#pragma once

#include "VehicleStrategyTypes.hpp"

// Forward declarations - actual implementations in module folders
namespace mode_manager_strategy { class ModeManagerStrategyBase; }
namespace automation_strategy { class AutomationStrategyBase; }

namespace vehicle_strategy
{

/**
 * @brief Central registry for vehicle-type-specific strategies
 *
 * Provides factory methods to get the appropriate strategy instance
 * for each module based on vehicle type. All strategies are singletons.
 */
class VehicleStrategyRegistry
{
public:
	/**
	 * @brief Get the ModeManager strategy for a vehicle type
	 * @param vehicle_type Vehicle type from vehicle_status_s
	 * @return Pointer to the strategy (never null)
	 */
	static const mode_manager_strategy::ModeManagerStrategyBase* getModeManagerStrategy(uint8_t vehicle_type);

	/**
	 * @brief Get the Automation strategy for a vehicle type
	 * @param vehicle_type Vehicle type from vehicle_status_s
	 * @return Pointer to the strategy (never null)
	 */
	static const automation_strategy::AutomationStrategyBase* getAutomationStrategy(uint8_t vehicle_type);

	/**
	 * @brief Get all registered vehicle types
	 * @param types Output array of vehicle types
	 * @param max_count Maximum number of types to return
	 * @return Number of types returned
	 */
	static size_t getAllVehicleTypes(VehicleType* types, size_t max_count);

	/**
	 * @brief Check if a vehicle type is registered
	 * @param vehicle_type Vehicle type to check
	 * @return true if vehicle type has registered strategies
	 */
	static bool isVehicleTypeRegistered(uint8_t vehicle_type);
};

} // namespace vehicle_strategy
