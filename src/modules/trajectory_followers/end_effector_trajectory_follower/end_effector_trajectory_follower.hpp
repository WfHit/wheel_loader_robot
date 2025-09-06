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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/end_effector_trajectory_setpoint.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/end_effector_status.h>
#include <uORB/topics/boom_trajectory_setpoint.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

namespace wheel_loader
{

/**
 * @brief End effector trajectory follower for wheel loader
 *
 * This module takes end effector trajectory setpoints (final position/orientation relative to chassis)
 * and generates separate boom and bucket trajectory commands using inverse kinematics.
 *
 * Architecture:
 * 1. Receives: End effector trajectory setpoint in chassis/world coordinates
 * 2. Computes: Inverse kinematics to get boom and bucket joint angles
 * 3. Generates: Trajectory setpoints for boom and bucket controllers
 * 4. Publishes: Boom trajectory setpoint + Bucket trajectory setpoint
 *
 * Control Flow:
 * End Effector Setpoint → [This Module] → Boom Trajectory + Bucket Trajectory
 *                                      ↓
 *                              Boom Controller + Bucket Controller
 *
 * Key coordinate transformations:
 * - Input: End effector position in chassis/world frame
 * - Output: Boom angle (chassis→boom) + Bucket angle (boom→bucket)
 */
class EndEffectorTrajectoryFollower : public ModuleBase<EndEffectorTrajectoryFollower>, public ModuleParams,
				  public px4::ScheduledWorkItem
{
public:
	EndEffectorTrajectoryFollower();
	~EndEffectorTrajectoryFollower() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// System state management structure
	struct SystemState {
		matrix::Vector3f chassis_position{};
		matrix::Quatf chassis_orientation{};
		boom_status_s boom_status{};
		bucket_status_s bucket_status{};
		hrt_abstime timestamp{};
	};

	// Update subscriptions and process end effector trajectory setpoint
	void update_trajectory_generation(const end_effector_trajectory_setpoint_s &setpoint);

	// System state management
	bool get_current_system_state(SystemState &state);
	void generate_trajectory_commands(float target_boom_angle, float target_bucket_angle, const SystemState &current_state);

	// Setpoint validation
	bool is_valid_setpoint(const end_effector_trajectory_setpoint_s &setpoint) const;

	// Inverse kinematics: end effector position (chassis frame) → boom + bucket joint angles
	bool compute_joint_angles(const matrix::Vector3f &end_effector_position_chassis,
				  const matrix::Quatf &end_effector_orientation_chassis,
				  float &boom_angle_chassis,
				  float &bucket_angle_boom) const;

	// Forward kinematics: joint angles → end effector position (for validation)
	matrix::Vector3f compute_end_effector_position(float boom_angle_chassis, float bucket_angle_boom) const;

	// Generate smooth trajectory setpoints for controllers
	void generate_boom_trajectory(float target_boom_angle, float boom_velocity);
	void generate_bucket_trajectory(float target_bucket_angle, float bucket_velocity);

	// Calculate velocities based on current status and target angles
	float calculate_boom_velocity(float target_boom_angle, const boom_status_s &boom_status) const;
	float calculate_bucket_velocity(float target_bucket_angle, const bucket_status_s &bucket_status) const;

	// Transform between coordinate frames
	matrix::Vector3f transform_world_to_chassis(const matrix::Vector3f &position_world,
						    const matrix::Vector3f &chassis_position,
						    const matrix::Quatf &chassis_orientation) const;

	// uORB subscriptions
	uORB::Subscription _end_effector_trajectory_setpoint_sub{ORB_ID(end_effector_trajectory_setpoint)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// uORB publications - trajectory setpoints for boom and bucket controllers
	uORB::Publication<boom_trajectory_setpoint_s> _boom_trajectory_pub{ORB_ID(boom_trajectory_setpoint)};
	uORB::Publication<bucket_trajectory_setpoint_s> _bucket_trajectory_pub{ORB_ID(bucket_trajectory_setpoint)};

	// Parameters
	DEFINE_PARAMETERS(
		// Motion limits
		(ParamFloat<px4::params::EEF_MAX_VEL>) _param_max_velocity,
		(ParamFloat<px4::params::EEF_MAX_ACCEL>) _param_max_acceleration,
		(ParamFloat<px4::params::EEF_MAX_FORCE>) _param_max_force,
		// Boom limits
		(ParamFloat<px4::params::EEF_B_EXT_MIN>) _param_boom_extension_min,
		(ParamFloat<px4::params::EEF_B_EXT_MAX>) _param_boom_extension_max,
		(ParamFloat<px4::params::EEF_B_LIFT_MIN>) _param_boom_lift_min,
		(ParamFloat<px4::params::EEF_B_MAX>) _param_boom_lift_max,
		// Tilt limits
		(ParamFloat<px4::params::EEF_TILT_MIN>) _param_tilt_min,
		(ParamFloat<px4::params::EEF_TILT_MAX>) _param_tilt_max,
		// Physical properties
		(ParamFloat<px4::params::EEF_MASS>) _param_end_effector_mass
	)

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _kinematics_perf;

	// Control state
	bool _trajectory_active{false};
	matrix::Vector3f _last_end_effector_position{};
	matrix::Quatf _last_end_effector_orientation{};
	float _last_boom_angle{0.0f};
	float _last_bucket_angle{0.0f};
	hrt_abstime _last_update_time{0};

	// Kinematic parameters (should come from configuration)
	static constexpr float BOOM_LENGTH = 3.0f;     // meters
	static constexpr float BUCKET_LENGTH = 1.5f;   // meters
	static constexpr float BOOM_PIVOT_HEIGHT = 1.0f; // meters above chassis

	// Control loop timing
	static constexpr uint32_t CONTROL_LOOP_INTERVAL_US = 50000; // 50ms = 20Hz

	// Trajectory generation constants
	static constexpr float VELOCITY_PROPORTIONAL_GAIN = 2.0f;
	static constexpr float DEFAULT_TIMEOUT_US = 1000000; // 1 second

	// Coordinate frame tolerance
	static constexpr float POSITION_TOLERANCE_M = 0.01f; // 1cm
	static constexpr float ANGLE_TOLERANCE_RAD = 0.01f;  // ~0.57 degrees
	static constexpr float QUATERNION_NORM_TOLERANCE = 0.1f; // Quaternion normalization tolerance

	// Safety limits and validation
	static constexpr float MIN_BOOM_ANGLE = -20.0f * M_PI_F / 180.0f; // -20 degrees (chassis frame)
	static constexpr float MAX_BOOM_ANGLE = 60.0f * M_PI_F / 180.0f;  // 60 degrees (chassis frame)
	static constexpr float MIN_BUCKET_ANGLE = -90.0f * M_PI_F / 180.0f; // -90 degrees (boom frame)
	static constexpr float MAX_BUCKET_ANGLE = 90.0f * M_PI_F / 180.0f;  // 90 degrees (boom frame)
};

} // namespace wheel_loader
