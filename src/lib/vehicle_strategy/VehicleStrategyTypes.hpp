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
 * @file VehicleStrategyTypes.hpp
 *
 * Common types and enums used across all vehicle strategy modules
 */

#pragma once

#include <stdint.h>

namespace vehicle_strategy
{

/**
 * @brief Vehicle type enumeration (mirrors vehicle_status_s values)
 */
enum class VehicleType : uint8_t {
	Unknown = 0,
	RotaryWing = 1,
	FixedWing = 2,
	Rover = 3,
	WheelLoader = 4,
	Count
};

/**
 * @brief Result of command handling
 */
enum class CommandResult : uint8_t {
	Accepted,            // Command accepted and will be executed
	Rejected,            // Command rejected (not allowed for this vehicle)
	Unsupported,         // Command not supported by this vehicle type
	TemporarilyRejected, // Command temporarily rejected (try again later)
	Delegated,           // Command should be handled by default handler
	InProgress           // Command is being processed
};

/**
 * @brief Result of mode selection
 */
enum class ModeResult : uint8_t {
	Success,             // Mode selected successfully
	ModeUnavailable,     // Mode not available for this vehicle type
	ModeNotAllowed,      // Mode not allowed in current state
	FallbackUsed,        // Fallback mode was used
	Failed               // Mode selection failed
};

/**
 * @brief Result of task selection
 */
enum class TaskResult : uint8_t {
	Success,             // Task selected successfully
	TaskUnavailable,     // Task not available for this vehicle type
	TaskNotAllowed,      // Task not allowed in current state
	FallbackUsed,        // Fallback task was used
	NoTask,              // No task to run (disarmed, etc.)
	Failed               // Task selection failed
};

/**
 * @brief Arming check result
 */
enum class ArmingResult : uint8_t {
	Allowed,             // Arming allowed
	Denied,              // Arming denied
	PreflightRequired,   // Preflight checks required
	SafetyNotReady       // Safety switch not ready
};

/**
 * @brief Failsafe action types
 */
enum class FailsafeAction : uint8_t {
	None,                // No action
	Warn,                // Just warn user
	Hold,                // Hold position/stop
	ReturnToLaunch,      // RTL
	Land,                // Land
	EmergencyStop,       // Emergency stop (ground vehicles)
	Descend,             // Emergency descend (aircraft)
	Disarm,              // Disarm
	Terminate            // Flight termination
};

/**
 * @brief Event types for failsafe handling
 */
enum class FailsafeEvent : uint8_t {
	None,
	RcLoss,              // RC signal lost
	DatalinkLoss,        // Datalink/telemetry lost
	LowBattery,          // Low battery detected
	CriticalBattery,     // Critical battery level
	GeofenceBreach,      // Geofence breach detected
	PositionLoss,        // Position estimate lost
	ObstacleDetected,    // Obstacle detected
	MissionInvalid,      // Mission invalid
	VlaTimeout,          // VLA trajectory timeout
	SensorFailure,       // Critical sensor failure
	MotorFailure         // Motor/actuator failure
};

/**
 * @brief Convert vehicle_status_s vehicle_type to VehicleType enum
 */
inline VehicleType vehicleTypeFromStatus(uint8_t status_vehicle_type)
{
	switch (status_vehicle_type) {
	case 1: return VehicleType::RotaryWing;
	case 2: return VehicleType::FixedWing;
	case 3: return VehicleType::Rover;
	case 4: return VehicleType::WheelLoader;
	default: return VehicleType::Unknown;
	}
}

/**
 * @brief Convert VehicleType enum to vehicle_status_s vehicle_type
 */
inline uint8_t vehicleTypeToStatus(VehicleType type)
{
	return static_cast<uint8_t>(type);
}

} // namespace vehicle_strategy
