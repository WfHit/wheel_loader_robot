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
 * Vehicle Type Registry - Factory and singleton access to vehicle type strategies
 *
 * This class provides:
 * - Singleton instances of each vehicle type strategy
 * - Factory method to get strategy by vehicle type
 * - Default strategy for unknown vehicle types
 */

#pragma once

#include "VehicleTypeStrategy.hpp"
#include "RotaryWingStrategy.hpp"
#include "WheelLoaderStrategy.hpp"

namespace vehicle_type
{

/**
 * @brief Registry for vehicle type strategies
 *
 * Provides access to singleton strategy instances for each vehicle type.
 */
class VehicleTypeRegistry
{
public:
	/**
	 * @brief Get the strategy for a specific vehicle type
	 * @param vehicle_type One of vehicle_status_s::VEHICLE_TYPE_* constants
	 * @return Pointer to the appropriate strategy, or default rotary wing if unknown
	 */
	static const VehicleTypeStrategy *getStrategy(uint8_t vehicle_type)
	{
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER:
			return &_wheel_loader_strategy;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
		default:
			// Default to rotary wing for compatibility
			return &_rotary_wing_strategy;
		}
	}

	/**
	 * @brief Get the rotary wing strategy
	 * @return Reference to the rotary wing strategy
	 */
	static const RotaryWingStrategy &rotaryWing()
	{
		return _rotary_wing_strategy;
	}

	/**
	 * @brief Get the wheel loader strategy
	 * @return Reference to the wheel loader strategy
	 */
	static const WheelLoaderStrategy &wheelLoader()
	{
		return _wheel_loader_strategy;
	}

	/**
	 * @brief Check if a vehicle type is supported
	 * @param vehicle_type Vehicle type constant
	 * @return true if a specific strategy exists for this type
	 */
	static bool isSupported(uint8_t vehicle_type)
	{
		return vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		       || vehicle_type == vehicle_status_s::VEHICLE_TYPE_WHEEL_LOADER;
	}

private:
	// Singleton strategy instances
	static RotaryWingStrategy _rotary_wing_strategy;
	static WheelLoaderStrategy _wheel_loader_strategy;
};

// Static member definitions are in VehicleTypeRegistry.cpp

} // namespace vehicle_type
