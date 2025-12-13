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
 * @file VehicleTypeStrategy.hpp
 *
 * Vehicle Type Strategy Pattern Interface
 *
 * This provides an abstract interface for vehicle-type-specific configurations.
 * Each vehicle type (wheel loader, rover, multicopter, fixed wing) implements
 * this interface to provide its specific modes, tasks, and capabilities.
 */

#pragma once

#include <uORB/topics/vehicle_type_config.h>
#include <uORB/topics/vehicle_status.h>
#include <stdint.h>

namespace vehicle_type
{

/**
 * @brief Control capabilities structure for a vehicle type
 */
struct ControlCapabilities {
	bool altitude_control{false};
	bool position_control{false};
	bool velocity_control{false};
	bool attitude_control{false};
	bool manual_control{true};
	bool autonomous_control{false};
	bool boom_control{false};
	bool tilt_control{false};
	bool articulated_steering{false};
};

/**
 * @brief Safety limits structure for a vehicle type
 */
struct SafetyLimits {
	float emergency_stop_decel{5.0f};  // m/s^2
	float max_velocity{10.0f};          // m/s
	float max_steering_rate{1.0f};      // rad/s
};

/**
 * @brief Abstract base class for vehicle type strategies
 *
 * Each vehicle type should implement this interface to define its
 * specific modes, automation tasks, and capabilities.
 */
class VehicleTypeStrategy
{
public:
	virtual ~VehicleTypeStrategy() = default;

	/**
	 * @brief Get the vehicle type constant
	 * @return Vehicle type from vehicle_status_s
	 */
	virtual uint8_t getVehicleType() const = 0;

	/**
	 * @brief Get the human-readable name of the vehicle type
	 * @return Vehicle type name string
	 */
	virtual const char *getName() const = 0;

	/**
	 * @brief Get the bitmask of available operation modes
	 * @return Bitmask where each bit corresponds to OPERATION_MODE_*
	 */
	virtual uint32_t getAvailableModesMask() const = 0;

	/**
	 * @brief Get the bitmask of available automation tasks
	 * @return Bitmask where each bit corresponds to AUTOMATION_TASK_*
	 */
	virtual uint32_t getAvailableAutomationTasksMask() const = 0;

	/**
	 * @brief Get the default operation mode
	 * @return Default OPERATION_MODE_* value
	 */
	virtual uint8_t getDefaultMode() const = 0;

	/**
	 * @brief Get the failsafe operation mode
	 * @return Failsafe OPERATION_MODE_* value
	 */
	virtual uint8_t getFailsafeMode() const = 0;

	/**
	 * @brief Get the mode change logic type
	 * @return MODE_CHANGE_LOGIC_* constant
	 */
	virtual uint8_t getModeChangeLogic() const = 0;

	/**
	 * @brief Get the control capabilities
	 * @return ControlCapabilities struct
	 */
	virtual ControlCapabilities getControlCapabilities() const = 0;

	/**
	 * @brief Get the safety limits
	 * @return SafetyLimits struct
	 */
	virtual SafetyLimits getSafetyLimits() const = 0;

	/**
	 * @brief Check if a specific operation mode is available
	 * @param mode OPERATION_MODE_* value to check
	 * @return true if mode is available
	 */
	bool isModeAvailable(uint8_t mode) const
	{
		if (mode >= 32) { return false; }

		return (getAvailableModesMask() & (1u << mode)) != 0;
	}

	/**
	 * @brief Check if a specific automation task is available
	 * @param task AUTOMATION_TASK_* value to check
	 * @return true if task is available
	 */
	bool isAutomationTaskAvailable(uint8_t task) const
	{
		if (task >= 32) { return false; }

		return (getAvailableAutomationTasksMask() & (1u << task)) != 0;
	}

	/**
	 * @brief Fill a vehicle_type_config_s message with this strategy's configuration
	 * @param config Reference to the config message to fill
	 */
	void fillConfig(vehicle_type_config_s &config) const
	{
		config.vehicle_type = getVehicleType();
		config.available_modes_mask = getAvailableModesMask();
		config.available_automation_tasks_mask = getAvailableAutomationTasksMask();
		config.default_mode = getDefaultMode();
		config.failsafe_mode = getFailsafeMode();
		config.mode_change_logic = getModeChangeLogic();

		ControlCapabilities caps = getControlCapabilities();
		config.supports_altitude_control = caps.altitude_control;
		config.supports_position_control = caps.position_control;
		config.supports_velocity_control = caps.velocity_control;
		config.supports_attitude_control = caps.attitude_control;
		config.supports_manual_control = caps.manual_control;
		config.supports_autonomous_control = caps.autonomous_control;
		config.supports_boom_control = caps.boom_control;
		config.supports_tilt_control = caps.tilt_control;
		config.supports_articulated_steering = caps.articulated_steering;

		SafetyLimits limits = getSafetyLimits();
		config.emergency_stop_decel = limits.emergency_stop_decel;
		config.max_velocity = limits.max_velocity;
		config.max_steering_rate = limits.max_steering_rate;

		config.config_valid = true;
	}
};

} // namespace vehicle_type
