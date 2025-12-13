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
 * @file ModeVLA.hpp
 *
 * VLA (Vision-Language-Action) Mode for wheel loader robot
 *
 * Receives VlaSetpointTriplet from Automation module at ~10Hz, performs:
 * - Smooth trajectory interpolation using PositionSmoothing
 * - Publishes ChassisControlSetpoint, BoomControlSetpoint, TiltControlSetpoint
 *
 * Note: Boom IK (bucket_height -> boom_angle) is computed in boom_control module
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../Mode/Mode.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vla_setpoint_triplet.h>
#include <uORB/topics/chassis_control_setpoint.h>
#include <uORB/topics/boom_control_setpoint.h>
#include <uORB/topics/tilt_control_setpoint.h>
#include <lib/mathlib/mathlib.h>
#include <motion_planning/PositionSmoothing.hpp>
#include <motion_planning/VelocitySmoothing.hpp>

class ModeVLA : public Mode
{
public:
	ModeVLA();
	~ModeVLA() override = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;
	bool updateInitialize() override;
	bool update() override;

private:
	static constexpr hrt_abstime VLA_TRIPLET_TIMEOUT{500000}; // 500ms timeout
	static constexpr float TRIPLET_INTERVAL{0.1f};            // 100ms between triplet updates

	// === Processing Methods ===

	/**
	 * Check if VLA setpoint triplet is valid
	 */
	bool isVlaTripletValid() const;

	/**
	 * Interpolate between current and next setpoint
	 * @param dt Time step since last update
	 */
	void interpolateSetpoints(float dt);

	/**
	 * Publish 50Hz control setpoints
	 */
	void publishChassisSetpoint();
	void publishBoomSetpoint();
	void publishTiltSetpoint();

	/**
	 * Publish hold setpoints (when no valid triplet)
	 */
	void publishHoldSetpoints();

	// === Subscriptions ===
	uORB::Subscription _sub_vla_triplet{ORB_ID(vla_setpoint_triplet)};

	// === Publications ===
	uORB::Publication<chassis_control_setpoint_s> _pub_chassis{ORB_ID(chassis_control_setpoint)};
	uORB::Publication<boom_control_setpoint_s> _pub_boom{ORB_ID(boom_control_setpoint)};
	uORB::Publication<tilt_control_setpoint_s> _pub_tilt{ORB_ID(tilt_control_setpoint)};

	// === State ===
	vla_setpoint_triplet_s _triplet{};
	hrt_abstime _triplet_timestamp{0};
	float _interpolation_progress{0.0f};
	uint32_t _last_sequence{0};

	// Interpolated setpoints
	chassis_control_setpoint_s _chassis_setpoint{};
	boom_control_setpoint_s _boom_setpoint{};
	tilt_control_setpoint_s _tilt_setpoint{};

	// Last valid setpoints (for hold mode)
	chassis_control_setpoint_s _last_chassis_setpoint{};
	boom_control_setpoint_s _last_boom_setpoint{};
	tilt_control_setpoint_s _last_tilt_setpoint{};
	// === Trajectory Smoothing ===
	PositionSmoothing _position_smoothing;          // XY position smoothing
	VelocitySmoothing _yaw_smoothing;               // Heading/yaw smoothing
	VelocitySmoothing _bucket_height_smoothing;     // Bucket height smoothing
	VelocitySmoothing _tilt_smoothing;              // Tilt angle smoothing

	// Smoothing configuration
	static constexpr float DEFAULT_MAX_JERK{5.0f};        // m/s³ or rad/s³
	static constexpr float DEFAULT_MAX_ACCEL{2.0f};       // m/s² or rad/s²
	static constexpr float DEFAULT_MAX_VEL_XY{2.0f};      // m/s
	static constexpr float DEFAULT_MAX_VEL_YAW{1.0f};     // rad/s
	static constexpr float DEFAULT_MAX_VEL_HEIGHT{0.5f};  // m/s
	static constexpr float DEFAULT_MAX_VEL_TILT{1.0f};    // rad/s
};
