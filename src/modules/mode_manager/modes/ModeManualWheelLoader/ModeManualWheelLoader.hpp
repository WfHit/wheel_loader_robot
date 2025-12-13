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
 * @file ModeManualWheelLoader.hpp
 *
 * Manual mode for wheel loader robot
 *
 * Control mapping:
 * - Right stick Y (pitch): Forward/backward velocity
 * - Right stick X (roll):  Steering angle
 * - Left stick Y (throttle): Boom height velocity
 * - Left stick X (yaw): Tilt angle velocity
 * - Aux1: Reserved (e.g., work mode select)
 * - Aux2: Reserved (e.g., horn/lights)
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../Mode/Mode.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/chassis_control_setpoint.h>
#include <uORB/topics/boom_control_setpoint.h>
#include <uORB/topics/tilt_control_setpoint.h>
#include <lib/mathlib/mathlib.h>
#include <motion_planning/VelocitySmoothing.hpp>

class ModeManualWheelLoader : public Mode
{
public:
	ModeManualWheelLoader();
	~ModeManualWheelLoader() override = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;
	bool updateInitialize() override;
	bool update() override;

private:
	// === Processing Methods ===

	/**
	 * Check if manual control input is valid
	 */
	bool isManualControlValid() const;

	/**
	 * Process stick inputs and generate control setpoints
	 */
	void processStickInputs(float dt);

	/**
	 * Apply deadzone to stick input
	 */
	float applyDeadzone(float input, float deadzone) const;

	/**
	 * Publish control setpoints
	 */
	void publishChassisSetpoint();
	void publishBoomSetpoint();
	void publishTiltSetpoint();

	/**
	 * Publish hold setpoints (when no valid input)
	 */
	void publishHoldSetpoints();

	// === Subscriptions ===
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	// === Publications ===
	uORB::Publication<chassis_control_setpoint_s> _pub_chassis{ORB_ID(chassis_control_setpoint)};
	uORB::Publication<boom_control_setpoint_s> _pub_boom{ORB_ID(boom_control_setpoint)};
	uORB::Publication<tilt_control_setpoint_s> _pub_tilt{ORB_ID(tilt_control_setpoint)};

	// === State ===
	manual_control_setpoint_s _manual_control{};
	hrt_abstime _last_manual_control_time{0};

	// Current setpoints
	chassis_control_setpoint_s _chassis_setpoint{};
	boom_control_setpoint_s _boom_setpoint{};
	tilt_control_setpoint_s _tilt_setpoint{};

	// Integrated positions (for velocity-based control)
	float _current_bucket_height{0.5f};  // Current bucket height from integration [m]
	float _current_tilt_angle{0.0f};     // Current tilt angle from integration [rad]
	float _current_steering_angle{0.0f}; // Current steering angle [rad]

	// === Smoothing ===
	VelocitySmoothing _velocity_smoothing;     // Forward velocity smoothing
	VelocitySmoothing _steering_smoothing;     // Steering rate smoothing
	VelocitySmoothing _bucket_height_smoothing; // Bucket height velocity smoothing
	VelocitySmoothing _tilt_smoothing;         // Tilt velocity smoothing

	// === Configuration ===
	static constexpr hrt_abstime MANUAL_CONTROL_TIMEOUT{500000}; // 500ms timeout
	static constexpr float STICK_DEADZONE{0.1f};                 // 10% deadzone
	static constexpr uint32_t MANUAL_SEQUENCE{0};                // Fixed sequence for manual mode

	// === Parameters ===
	DEFINE_PARAMETERS(
		// Chassis control limits
		(ParamFloat<px4::params::WL_MAX_VEL>) _param_max_velocity,           // Max forward velocity [m/s]
		(ParamFloat<px4::params::WL_MAX_STEER>) _param_max_steering,         // Max steering angle [rad]
		(ParamFloat<px4::params::WL_MAX_STEER_R>) _param_max_steering_rate,  // Max steering rate [rad/s]

		// Boom control limits
		(ParamFloat<px4::params::WL_BKT_H_MIN>) _param_bucket_height_min,    // Min bucket height [m]
		(ParamFloat<px4::params::WL_BKT_H_MAX>) _param_bucket_height_max,    // Max bucket height [m]
		(ParamFloat<px4::params::WL_BKT_H_VEL>) _param_bucket_height_vel,    // Max bucket height velocity [m/s]

		// Tilt control limits
		(ParamFloat<px4::params::WL_TILT_MIN>) _param_tilt_min,              // Min tilt angle [rad]
		(ParamFloat<px4::params::WL_TILT_MAX>) _param_tilt_max,              // Max tilt angle [rad]
		(ParamFloat<px4::params::WL_TILT_VEL>) _param_tilt_vel,              // Max tilt velocity [rad/s]

		// Smoothing
		(ParamFloat<px4::params::WL_JERK_MAX>) _param_max_jerk,              // Max jerk for smoothing
		(ParamFloat<px4::params::WL_ACCEL_MAX>) _param_max_accel             // Max acceleration for smoothing
	)
};
