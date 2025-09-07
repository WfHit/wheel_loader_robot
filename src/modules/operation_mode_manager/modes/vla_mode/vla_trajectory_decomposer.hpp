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

#include "../../common/trajectory_types.hpp"

namespace wheel_loader
{

/**
 * Decomposes VLA 6DOF bucket trajectory into chassis and bucket components
 *
 * All trajectories are in WORLD coordinates:
 * - Input: VLA bucket trajectory in world frame
 * - Output: Chassis trajectory in world frame
 * - Output: Bucket trajectory in world frame
 *
 * The decomposer determines how to coordinate chassis and bucket motion
 * to achieve the desired bucket trajectory in world coordinates.
 */
class VlaTrajectoryDecomposer
{
public:
	VlaTrajectoryDecomposer() = default;
	~VlaTrajectoryDecomposer() = default;

	/**
	 * Initialize the decomposer with robot constraints
	 */
	void init(float max_reach, float coord_factor);

	/**
	 * Decompose VLA trajectory into chassis and end effector world trajectories
	 *
	 * @param vla_point Input VLA trajectory point (end effector 6DOF in world frame)
	 * @param current_chassis_pos Current chassis position in world frame
	 * @param current_chassis_yaw Current chassis yaw angle
	 * @param chassis_trajectory Output chassis trajectory in world frame
	 * @param end_effector_trajectory Output end effector trajectory in world frame
	 * @return true if decomposition successful
	 */
	bool decompose(const VlaTrajectoryPoint &vla_point,
	               const Vector3f &current_chassis_pos,
	               float current_chassis_yaw,
	               ChassisTrajectorySetpoint &chassis_trajectory,
	               EndEffectorTrajectorySetpoint &end_effector_trajectory);

private:
	/**
	 * Determine optimal chassis contribution to end effector motion
	 *
	 * The chassis can help reduce end effector motion by moving closer to the target.
	 * This calculates how much the chassis should move to assist the end effector.
	 */
	Vector3f calculate_chassis_contribution(const Vector3f &end_effector_target_world,
	                                        const Vector3f &current_chassis_pos);

	/**
	 * Calculate chassis trajectory to support end effector motion
	 */
	void generate_chassis_trajectory(const VlaTrajectoryPoint &vla_point,
	                                 const Vector3f &current_chassis_pos,
	                                 float current_chassis_yaw,
	                                 const Vector3f &chassis_contribution,
	                                 ChassisTrajectorySetpoint &chassis_trajectory);

	/**
	 * Generate end effector trajectory in world frame
	 */
	void generate_end_effector_trajectory(const VlaTrajectoryPoint &vla_point,
	                                EndEffectorTrajectorySetpoint &end_effector_trajectory);

	/**
	 * Check if the decomposed trajectories are feasible
	 */
	bool validate_trajectories(const ChassisTrajectorySetpoint &chassis_trajectory,
	                           const EndEffectorTrajectorySetpoint &end_effector_trajectory);

	// Robot constraints
	float max_reach{3.0f};           // Maximum end effector reach from chassis (m)
	float coordination_factor{0.7f}; // How much chassis helps end effector (0=none, 1=full)

	// Motion limits
	float max_chassis_velocity{2.0f};        // m/s
	float max_chassis_turn_rate{1.0f};       // rad/s
	float max_end_effector_velocity{1.0f};      // m/s
	float max_end_effector_angular_rate{1.5f};  // rad/s
};

} // namespace wheel_loader
