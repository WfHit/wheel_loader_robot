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
 * @file VehicleCapabilities.hpp
 *
 * Single Source of Truth for Vehicle-Type-Specific Capabilities
 *
 * This interface defines ALL vehicle-type-specific behavior in one place.
 * All modules (mode_manager, system_manager, automation) reference this
 * instead of maintaining their own duplicate hierarchies.
 *
 * Architecture:
 *   VehicleCapabilities (interface)
 *        ├── WheelLoaderCapabilities
 *        ├── RoverCapabilities
 *        ├── RotaryWingCapabilities
 *        └── FixedWingCapabilities
 *
 * Usage:
 *   const auto* caps = VehicleCapabilitiesRegistry::get(vehicle_type);
 *   if (caps->isModeAvailable(mode)) { ... }
 */

#pragma once

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <stdint.h>
#include <stddef.h>

namespace vehicle
{

//============================================================================
// Mode Mask Constants
//============================================================================

/**
 * @brief Operation mode mask bits for getAvailableModesMask()
 */
namespace ModeMask
{
constexpr uint64_t MANUAL           = (1ULL << vehicle_status_s::OPERATION_MODE_MANUAL);
constexpr uint64_t ALTITUDE         = (1ULL << vehicle_status_s::OPERATION_MODE_ALTITUDE);
constexpr uint64_t POSITION         = (1ULL << vehicle_status_s::OPERATION_MODE_POSITION);
constexpr uint64_t AUTO_MISSION     = (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_MISSION);
constexpr uint64_t AUTO_LOITER      = (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LOITER);
constexpr uint64_t AUTO_RTL         = (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_RTL);
constexpr uint64_t AUTO_TAKEOFF     = (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_TAKEOFF);
constexpr uint64_t AUTO_LAND        = (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_LAND);
constexpr uint64_t AUTO_VLA         = (1ULL << vehicle_status_s::OPERATION_MODE_AUTO_VLA);
constexpr uint64_t OFFBOARD         = (1ULL << vehicle_status_s::OPERATION_MODE_OFFBOARD);
constexpr uint64_t STABILIZED       = (1ULL << vehicle_status_s::OPERATION_MODE_STABILIZED);
constexpr uint64_t ACRO             = (1ULL << vehicle_status_s::OPERATION_MODE_ACRO);
}  // namespace ModeMask

//============================================================================
// Task Mask Constants
//============================================================================

/**
 * @brief Automation task mask bits for getAvailableTasksMask()
 */
namespace TaskMask
{
constexpr uint32_t MISSION   = (1U << 0);
constexpr uint32_t LOITER    = (1U << 1);
constexpr uint32_t RTL       = (1U << 2);
constexpr uint32_t TAKEOFF   = (1U << 3);
constexpr uint32_t LAND      = (1U << 4);
constexpr uint32_t VLA       = (1U << 5);
constexpr uint32_t PRECLAND  = (1U << 6);
constexpr uint32_t ORBIT     = (1U << 7);
constexpr uint32_t DESCEND   = (1U << 8);
}  // namespace TaskMask

//============================================================================
// Enumerations
//============================================================================

/**
 * @brief Failsafe action types
 */
enum class FailsafeAction : uint8_t {
	None,           ///< No action
	Warn,           ///< Warning only
	Hold,           ///< Stop/hover in place
	RTL,            ///< Return to launch
	Land,           ///< Land immediately
	Descend,        ///< Controlled descent
	EmergencyStop,  ///< Emergency stop (ground vehicles)
	Disarm,         ///< Disarm motors
	Terminate       ///< Flight termination
};

/**
 * @brief Failsafe event types
 */
enum class FailsafeEvent : uint8_t {
	None,
	RcLoss,              ///< RC signal lost
	DatalinkLoss,        ///< Datalink/telemetry lost
	LowBattery,          ///< Low battery detected
	CriticalBattery,     ///< Critical battery level
	GeofenceBreach,      ///< Geofence breach detected
	PositionLoss,        ///< Position estimate lost
	ObstacleDetected,    ///< Obstacle detected
	SensorFailure,       ///< Sensor failure
	VlaTimeout,          ///< VLA trajectory timeout (wheel loader)
	BoomLimit,           ///< Boom limit reached (wheel loader)
	ManualOverride       ///< Manual override requested
};

/**
 * @brief Arming check result
 */
enum class ArmingCheckResult : uint8_t {
	Allowed,             ///< Arming allowed
	Denied,              ///< Arming denied
	PreflightRequired,   ///< Preflight checks required
	SafetyNotReady       ///< Safety switch not ready
};

//============================================================================
// Capability Structures
//============================================================================

/**
 * @brief Control capabilities flags
 */
struct ControlCapabilities {
	bool altitude_control : 1;       ///< Can control altitude
	bool position_control : 1;       ///< Can control position
	bool velocity_control : 1;       ///< Can control velocity
	bool attitude_control : 1;       ///< Can control attitude
	bool boom_control : 1;           ///< Has boom actuator (wheel loader)
	bool tilt_control : 1;           ///< Has tilt actuator (wheel loader)
	bool articulated_steering : 1;   ///< Has articulated frame steering
	bool vertical_takeoff : 1;       ///< Can take off vertically
	bool landing_gear : 1;           ///< Has retractable landing gear
	bool vtol_transition : 1;        ///< Can transition between VTOL modes
};

/**
 * @brief Physical safety limits
 */
struct SafetyLimits {
	float max_velocity;            ///< Maximum allowed velocity [m/s]
	float emergency_stop_decel;    ///< Emergency stop deceleration [m/s²]
	float max_turn_rate;           ///< Maximum turn rate [rad/s]
	float geofence_margin;         ///< Geofence margin distance [m]
};

//============================================================================
// VehicleCapabilities Interface
//============================================================================

/**
 * @class VehicleCapabilities
 *
 * @brief Single source of truth for all vehicle-type-specific behavior
 *
 * This abstract interface defines everything a vehicle type can do.
 * All modules (mode_manager, system_manager, automation) should query
 * this interface rather than maintaining duplicate vehicle hierarchies.
 *
 * Benefits:
 * - Single location for vehicle-specific code
 * - No duplication across modules
 * - Easy to add new vehicle types
 * - Clear separation of concerns
 */
class VehicleCapabilities
{
public:
	virtual ~VehicleCapabilities() = default;

	//========================================================================
	// Identity
	//========================================================================

	/**
	 * @brief Get the vehicle type constant
	 * @return Vehicle type from vehicle_status_s::VEHICLE_TYPE_*
	 */
	virtual uint8_t getVehicleType() const = 0;

	/**
	 * @brief Get human-readable vehicle type name
	 * @return Static string with vehicle name
	 */
	virtual const char* getName() const = 0;

	//========================================================================
	// Mode Availability (used by mode_manager)
	//========================================================================

	/**
	 * @brief Get bitmask of available operation modes
	 * @return Bitmask using ModeMask:: constants
	 */
	virtual uint64_t getAvailableModesMask() const = 0;

	/**
	 * @brief Get default operation mode for this vehicle
	 * @return Operation mode constant from vehicle_status_s
	 */
	virtual uint8_t getDefaultMode() const = 0;

	/**
	 * @brief Get failsafe operation mode for this vehicle
	 * @return Operation mode constant from vehicle_status_s
	 */
	virtual uint8_t getFailsafeMode() const = 0;

	/**
	 * @brief Check if a specific mode is available
	 * @param mode Operation mode to check
	 * @return true if mode is available for this vehicle
	 */
	bool isModeAvailable(uint8_t mode) const {
		return (getAvailableModesMask() & (1ULL << mode)) != 0;
	}

	//========================================================================
	// Task Availability (used by automation)
	//========================================================================

	/**
	 * @brief Get bitmask of available automation tasks
	 * @return Bitmask using TaskMask:: constants
	 */
	virtual uint32_t getAvailableTasksMask() const = 0;

	/**
	 * @brief Check if a specific task is available
	 * @param task Task type to check
	 * @return true if task is available for this vehicle
	 */
	bool isTaskAvailable(uint8_t task) const {
		return (getAvailableTasksMask() & (1U << task)) != 0;
	}

	//========================================================================
	// Command Handling (used by system_manager)
	//========================================================================

	/**
	 * @brief Check if a command should be rejected for this vehicle type
	 * @param cmd_id Command ID from vehicle_command_s
	 * @return true if command should be rejected
	 */
	virtual bool shouldRejectCommand(uint16_t cmd_id) const = 0;

	/**
	 * @brief Get list of unsupported commands
	 * @param[out] cmds Array to fill with unsupported command IDs
	 * @param max Maximum number of commands to return
	 * @return Number of commands filled
	 */
	virtual size_t getUnsupportedCommands(uint16_t* cmds, size_t max) const = 0;

	//========================================================================
	// Failsafe Behavior (used by system_manager)
	//========================================================================

	/**
	 * @brief Get appropriate failsafe action for an event
	 * @param event Failsafe event that occurred
	 * @return Recommended failsafe action
	 */
	virtual FailsafeAction getFailsafeAction(FailsafeEvent event) const = 0;

	/**
	 * @brief Check if a failsafe action is supported
	 * @param action Failsafe action to check
	 * @return true if action is supported by this vehicle
	 */
	virtual bool supportsFailsafeAction(FailsafeAction action) const = 0;

	//========================================================================
	// Control Capabilities
	//========================================================================

	/**
	 * @brief Get control capability flags
	 * @return ControlCapabilities structure
	 */
	virtual ControlCapabilities getControlCapabilities() const = 0;

	//========================================================================
	// Physical Limits
	//========================================================================

	/**
	 * @brief Get safety limits for this vehicle
	 * @return SafetyLimits structure
	 */
	virtual SafetyLimits getSafetyLimits() const = 0;

	//========================================================================
	// Arming Checks (used by system_manager)
	//========================================================================

	/**
	 * @brief Get required arming checks bitmask
	 * @return Bitmask of required health checks
	 */
	virtual uint32_t getRequiredArmingChecks() const = 0;

	/**
	 * @brief Perform vehicle-specific pre-arm check
	 * @param[out] reason Human-readable failure reason (if failed)
	 * @return true if check passes
	 */
	virtual bool vehicleSpecificPrearmCheck(const char** reason) const {
		(void)reason;
		return true;
	}
};

}  // namespace vehicle
