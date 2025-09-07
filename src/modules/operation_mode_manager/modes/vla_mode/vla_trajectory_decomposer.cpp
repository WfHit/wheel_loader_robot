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

#include "vla_trajectory_decomposer.hpp"
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;

void VlaTrajectoryDecomposer::init(float reach_limit, float coord_factor)
{
	this->max_reach = reach_limit;
	this->coordination_factor = math::constrain(coord_factor, 0.0f, 1.0f);
}

bool VlaTrajectoryDecomposer::decompose(const VlaTrajectoryPoint &vla_point,
                                        const Vector3f &current_chassis_pos,
                                        float current_chassis_yaw,
                                        ChassisTrajectorySetpoint &chassis_trajectory,
                                        EndEffectorTrajectorySetpoint &end_effector_trajectory)
{
	// Calculate how much the chassis should contribute to the motion
	Vector3f chassis_contribution = calculate_chassis_contribution(
		vla_point.end_effector_position, current_chassis_pos);

	// Generate chassis trajectory
	generate_chassis_trajectory(vla_point, current_chassis_pos, current_chassis_yaw,
	                            chassis_contribution, chassis_trajectory);

	// Generate end effector trajectory (convert from world to chassis frame)
	generate_end_effector_trajectory(vla_point, end_effector_trajectory);

	// Validate the trajectories
	return validate_trajectories(chassis_trajectory, end_effector_trajectory);
}

Vector3f VlaTrajectoryDecomposer::calculate_chassis_contribution(
	const Vector3f &end_effector_target_world,
	const Vector3f &current_chassis_pos)
{
	// Calculate distance from chassis to end effector target
	Vector3f chassis_to_end_effector = end_effector_target_world - current_chassis_pos;
	chassis_to_end_effector(2) = 0.0f;  // Only consider horizontal distance

	float reach_distance = chassis_to_end_effector.norm();

	// If end effector target is far, chassis should move to help
	if (reach_distance > max_reach * 0.6f) {
		// Calculate how much chassis should move
		float assistance_factor = math::constrain(
			(reach_distance - max_reach * 0.5f) / (max_reach * 0.5f),
			0.0f, 1.0f
		);

		// Direction chassis should move
		Vector3f assistance_direction = chassis_to_end_effector.normalized();

		// Calculate chassis contribution
		float contribution_magnitude = coordination_factor * assistance_factor *
		                               (reach_distance - max_reach * 0.5f);

		return assistance_direction * contribution_magnitude;
	}

	return Vector3f{};  // No assistance needed
}

void VlaTrajectoryDecomposer::generate_chassis_trajectory(
	const VlaTrajectoryPoint &vla_point,
	const Vector3f &current_chassis_pos,
	float current_chassis_yaw,
	const Vector3f &chassis_contribution,
	ChassisTrajectorySetpoint &chassis_trajectory)
{
	const hrt_abstime now = hrt_absolute_time();

	// Calculate target chassis position
	Vector3f target_chassis_pos = current_chassis_pos + chassis_contribution;

	// Calculate target chassis orientation to face bucket target
	Vector3f bucket_direction = vla_point.end_effector_position - target_chassis_pos;
	bucket_direction(2) = 0.0f;  // Only horizontal

	float target_chassis_yaw = current_chassis_yaw;
	if (bucket_direction.norm() > 0.1f) {
		target_chassis_yaw = atan2f(bucket_direction(1), bucket_direction(0));
	}

	// Calculate chassis velocity (simple proportional to position error)
	Vector3f position_error = target_chassis_pos - current_chassis_pos;
	Vector3f chassis_velocity = position_error * 2.0f;  // P gain of 2.0

	// Limit chassis velocity
	float velocity_norm = chassis_velocity.norm();
	if (velocity_norm > max_chassis_velocity) {
		chassis_velocity = chassis_velocity * (max_chassis_velocity / velocity_norm);
	}

	// Calculate yaw rate
	float yaw_error = matrix::wrap_pi(target_chassis_yaw - current_chassis_yaw);
	float chassis_yaw_rate = yaw_error * 2.0f;  // P gain of 2.0
	chassis_yaw_rate = math::constrain(chassis_yaw_rate, -max_chassis_turn_rate, max_chassis_turn_rate);

	// Set chassis trajectory
	chassis_trajectory.position = target_chassis_pos;
	chassis_trajectory.yaw = target_chassis_yaw;
	chassis_trajectory.velocity = chassis_velocity;
	chassis_trajectory.yaw_rate = chassis_yaw_rate;
	chassis_trajectory.timestamp = now;
	chassis_trajectory.valid = true;
}

void VlaTrajectoryDecomposer::generate_end_effector_trajectory(
	const VlaTrajectoryPoint &vla_point,
	EndEffectorTrajectorySetpoint &end_effector_trajectory)
{
	// Convert from 6DOF end effector pose in world frame to 2DOF joint angles in chassis frame
	// This involves inverse kinematics to compute boom and bucket angles

	// Get current chassis pose (this should be available from vehicle state)
	Vector3f chassis_position = Vector3f(0.0f, 0.0f, 0.0f); // Will be set from actual chassis pose
	Quatf chassis_orientation = Quatf(1.0f, 0.0f, 0.0f, 0.0f); // Will be set from actual chassis pose

	// TODO: Get actual chassis pose from vehicle state or current chassis trajectory
	// chassis_position = current_chassis_position;
	// chassis_orientation = current_chassis_orientation;

	// Transform end effector position from world frame to chassis frame
	Vector3f world_to_chassis_position = vla_point.end_effector_position - chassis_position;
	Vector3f ee_pos_chassis = chassis_orientation.inversed().rotateVector(world_to_chassis_position);

	// Simplified inverse kinematics for 2DOF boom-bucket system
	// Assuming boom rotates vertically and bucket rotates relative to boom
	const float boom_length = 2.5f;   // Robot parameter (should come from config)
	// const float bucket_length = 1.0f; // Robot parameter - not needed for 2DOF joint control
	const float boom_pivot_height = 1.2f; // Height of boom pivot above chassis

	// Calculate boom angle from horizontal distance and height
	float horizontal_distance = sqrtf(ee_pos_chassis(0)*ee_pos_chassis(0) + ee_pos_chassis(1)*ee_pos_chassis(1));
	float vertical_distance = ee_pos_chassis(2) - boom_pivot_height;

	// Boom angle (measured from horizontal)
	end_effector_trajectory.boom_angle = atan2f(vertical_distance, horizontal_distance);

	// Simplified bucket angle calculation (from end effector orientation)
	// This is a simplified model - in practice would need full inverse kinematics
	AxisAnglef ee_axis_angle = AxisAnglef(chassis_orientation.inversed() * vla_point.end_effector_orientation);
	end_effector_trajectory.bucket_angle = ee_axis_angle(1); // Pitch component as bucket angle

	// Calculate angle rates from end effector velocities (simplified)
	Vector3f ee_vel_chassis = chassis_orientation.inversed().rotateVector(vla_point.end_effector_velocity);
	Vector3f ee_angvel_chassis = chassis_orientation.inversed().rotateVector(vla_point.end_effector_angular_velocity);

	// Approximate joint rates from end effector motion (this is a simplification)
	end_effector_trajectory.boom_angle_rate = ee_vel_chassis(2) / boom_length; // Vertical velocity to boom rate
	end_effector_trajectory.bucket_angle_rate = ee_angvel_chassis(1); // Angular velocity pitch component

	// Apply joint limits
	end_effector_trajectory.boom_angle = math::constrain(end_effector_trajectory.boom_angle, -1.57f, 1.57f); // ±90°
	end_effector_trajectory.bucket_angle = math::constrain(end_effector_trajectory.bucket_angle, -1.57f, 1.57f); // ±90°

	// Apply rate limits
	end_effector_trajectory.boom_angle_rate = math::constrain(end_effector_trajectory.boom_angle_rate, -1.0f, 1.0f); // rad/s
	end_effector_trajectory.bucket_angle_rate = math::constrain(end_effector_trajectory.bucket_angle_rate, -2.0f, 2.0f); // rad/s

	end_effector_trajectory.timestamp = vla_point.timestamp;
	end_effector_trajectory.valid = vla_point.valid;
}

bool VlaTrajectoryDecomposer::validate_trajectories(
	const ChassisTrajectorySetpoint &chassis_trajectory,
	const EndEffectorTrajectorySetpoint &end_effector_trajectory)
{
	// Validate joint angles are within limits
	if (fabsf(end_effector_trajectory.boom_angle) > 1.57f) { // ±90°
		return false;
	}

	if (fabsf(end_effector_trajectory.bucket_angle) > 1.57f) { // ±90°
		return false;
	}

	// Validate joint rates are within limits
	if (fabsf(end_effector_trajectory.boom_angle_rate) > 1.0f) { // rad/s
		return false;
	}

	if (fabsf(end_effector_trajectory.bucket_angle_rate) > 2.0f) { // rad/s
		return false;
	}

	// Check chassis velocity limits
	if (chassis_trajectory.velocity.norm() > max_chassis_velocity) {
		return false;
	}

	// Check chassis angular rate limits
	if (fabsf(chassis_trajectory.yaw_rate) > max_chassis_turn_rate) {
		return false;
	}

	// Check workspace limits (forward kinematics to validate reach)
	const float boom_length = 2.5f;
	const float bucket_length = 1.0f;

	// Calculate end effector position from joint angles
	float boom_tip_x = boom_length * cosf(end_effector_trajectory.boom_angle);
	float boom_tip_z = boom_length * sinf(end_effector_trajectory.boom_angle);

	float ee_x = boom_tip_x + bucket_length * cosf(end_effector_trajectory.boom_angle + end_effector_trajectory.bucket_angle);
	float ee_z = boom_tip_z + bucket_length * sinf(end_effector_trajectory.boom_angle + end_effector_trajectory.bucket_angle);

	float reach_distance = sqrtf(ee_x*ee_x + ee_z*ee_z);
	if (reach_distance > max_reach) {
		return false;  // End effector beyond maximum reach
	}

	return true;
}
