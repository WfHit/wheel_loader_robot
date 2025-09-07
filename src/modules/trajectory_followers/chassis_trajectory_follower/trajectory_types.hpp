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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

namespace wheel_loader
{

/**
 * Chassis trajectory setpoint in WORLD coordinates
 * Published by operation modes, subscribed by chassis trajectory follower
 */
struct ChassisTrajectorySetpoint {
	Vector3f position;           // Position in world frame (m)
	float yaw;                   // Yaw angle in world frame (rad)
	Vector3f velocity;           // Linear velocity in world frame (m/s)
	float yaw_rate;              // Yaw rate (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * End effector trajectory setpoint with 2 DOF control in CHASSIS coordinates
 * Published by operation modes, subscribed by end effector trajectory follower
 */
struct EndEffectorTrajectorySetpoint {
	float boom_angle;            // Boom angle relative to chassis (rad)
	float bucket_angle;          // Bucket angle relative to boom (rad)
	float boom_angle_rate;       // Boom angle rate (rad/s)
	float bucket_angle_rate;     // Bucket angle rate (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * VLA trajectory point - Complete 6DOF end effector pose in WORLD frame
 * This is the input from the Vision-Language-Action model
 */
struct VlaTrajectoryPoint {
	Vector3f end_effector_position;    // End effector position in world frame (m)
	Quatf end_effector_orientation;    // End effector orientation in world frame
	Vector3f end_effector_velocity;    // End effector linear velocity in world frame (m/s)
	Vector3f end_effector_angular_velocity; // End effector angular velocity in world frame (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * Manual control inputs from RC
 */
struct ManualControlInputs {
	float chassis_velocity;      // Forward/backward velocity command (-1 to 1)
	float chassis_turn_angle;    // Turn angle command (-1 to 1)
	float boom_lift_velocity;    // Boom lift velocity command (-1 to 1)
	float bucket_angle;          // Bucket angle command (-1 to 1)
	bool mode_switch;            // Mode switch state
	hrt_abstime timestamp;       // Timestamp
};

} // namespace wheel_loader
