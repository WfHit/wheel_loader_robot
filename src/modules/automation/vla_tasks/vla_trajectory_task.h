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
 * @file vla_trajectory_task.h
 *
 * VLA Trajectory Task for Automation module
 *
 * Receives 7-DOF VLA trajectory predictions from VLA model, decomposes into
 * chassis/boom/tilt channels, applies smoothing with kinematic constraints,
 * and publishes VlaSetpointTriplet at ~10Hz to ModeVLA for final interpolation.
 *
 * VLA Output: [Δx, Δy, Δheading, steering_angle, bucket_height, tilt_angle, slip_rate]
 *             16 steps × 100ms = 1.6s horizon (Body Frame)
 *
 * Architecture:
 *   VlaTrajectoryTask (10Hz smoothed triplet) → ModeVLA (50Hz interpolation + IK) → Controllers
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../task_base.h"
#include "../task_block.h"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vla_trajectory.h>
#include <uORB/topics/vla_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

class Automation;

class VlaTrajectoryTask : public TaskBlock, public ModuleParams
{
public:
	VlaTrajectoryTask(Automation *automation);
	~VlaTrajectoryTask() = default;

	void initialize() override;
	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	static constexpr int MAX_TRAJECTORY_POINTS = 16;
	static constexpr int SMOOTHED_BUFFER_SIZE = 32;  // Smoothed waypoint buffer
	static constexpr float VLA_DT = 0.1f;            // VLA step interval (100ms)
	static constexpr hrt_abstime VLA_TIMEOUT{500000}; // 500ms timeout

	// === Core Processing Methods ===

	/**
	 * Update vehicle state from automation
	 */
	void updateVehicleState();

	/**
	 * Check if VLA trajectory is valid
	 */
	bool isVlaTrajectoryValid() const;

	/**
	 * Process new VLA trajectory - main pipeline
	 */
	bool processNewVlaTrajectory();

	/**
	 * Decompose 7-DOF trajectory into separate channels
	 * Transforms body-frame deltas to world-frame absolute positions
	 * NOTE: bucket_height is passed through directly (no IK here)
	 */
	void decomposeTrajectory();

	/**
	 * Compute synchronized timing - find slowest channel
	 */
	void computeSynchronizedTiming();

	/**
	 * Apply smoothing with kinematic constraints
	 * Outputs smoothed waypoints at ~10Hz rate
	 */
	void smoothTrajectories();

	/**
	 * Smooth single DOF trajectory with jerk limiting
	 */
	void smoothSingleDof(
		const float *input_points,
		int num_input,
		float *output_pos,
		float *output_vel,
		float *output_acc,
		int &num_output,
		float max_vel,
		float max_acc,
		float max_jerk,
		float time_scale,
		float initial_pos,
		float initial_vel);

	/**
	 * Publish VLA setpoint triplet (previous/current/next)
	 * Called at ~10Hz to provide smoothed waypoints to ModeVLA
	 */
	void publishVlaSetpointTriplet();

	/**
	 * Publish hold triplet (when no valid trajectory or trajectory complete)
	 */
	void publishHoldTriplet();

	/**
	 * Build VlaSetpoint from smoothed buffer at given index
	 */
	vla_setpoint_s buildSetpointFromIndex(int idx);

	// === Subscriptions ===
	uORB::Subscription _sub_vla_trajectory{ORB_ID(vla_trajectory)};

	// === Publications ===
	uORB::Publication<vla_setpoint_triplet_s> _pub_vla_triplet{ORB_ID(vla_setpoint_triplet)};

	// === Processing State ===
	enum class ProcessorState {
		IDLE,               // No valid trajectory
		EXECUTING           // Executing smoothed trajectory
	};
	ProcessorState _processor_state{ProcessorState::IDLE};

	// Raw VLA trajectory
	vla_trajectory_s _vla_trajectory{};
	uint32_t _last_vla_sequence{0};

	// Current vehicle state
	struct VehicleState {
		float x{0.0f};
		float y{0.0f};
		float heading{0.0f};
		float vx{0.0f};
		float vy{0.0f};
		float yaw_rate{0.0f};
		bool valid{false};
	} _vehicle_state;

	// Decomposed raw trajectories (world frame)
	struct DecomposedTrajectory {
		int num_points{0};
		float x[MAX_TRAJECTORY_POINTS]{};
		float y[MAX_TRAJECTORY_POINTS]{};
		float heading[MAX_TRAJECTORY_POINTS]{};
		float steering[MAX_TRAJECTORY_POINTS]{};
		float bucket_height[MAX_TRAJECTORY_POINTS]{};  // Pass through, IK in ModeVLA
		float tilt_angle[MAX_TRAJECTORY_POINTS]{};
		float slip_rate[MAX_TRAJECTORY_POINTS]{};
	} _decomposed;

	// Synchronized timing
	struct TrajectoryTiming {
		float chassis_time{0.0f};
		float boom_time{0.0f};
		float tilt_time{0.0f};
		float sync_time{0.0f};
		float scale_factor{1.0f};
	} _timing;

	// Smoothed trajectory buffers (waypoints at ~10Hz intervals)
	struct SmoothedBuffer {
		int num_points{0};
		int current_index{0};

		// Chassis
		float chassis_x_pos[SMOOTHED_BUFFER_SIZE]{};
		float chassis_x_vel[SMOOTHED_BUFFER_SIZE]{};
		float chassis_x_acc[SMOOTHED_BUFFER_SIZE]{};
		float chassis_y_pos[SMOOTHED_BUFFER_SIZE]{};
		float chassis_y_vel[SMOOTHED_BUFFER_SIZE]{};
		float chassis_y_acc[SMOOTHED_BUFFER_SIZE]{};
		float chassis_heading[SMOOTHED_BUFFER_SIZE]{};
		float chassis_yaw_rate[SMOOTHED_BUFFER_SIZE]{};
		float chassis_yaw_acc[SMOOTHED_BUFFER_SIZE]{};
		float chassis_steering[SMOOTHED_BUFFER_SIZE]{};
		float chassis_steering_rate[SMOOTHED_BUFFER_SIZE]{};
		float slip_rate[SMOOTHED_BUFFER_SIZE]{};

		// Bucket height (passed through, IK done in ModeVLA)
		float bucket_height[SMOOTHED_BUFFER_SIZE]{};
		float bucket_height_vel[SMOOTHED_BUFFER_SIZE]{};

		// Tilt
		float tilt_pos[SMOOTHED_BUFFER_SIZE]{};
		float tilt_vel[SMOOTHED_BUFFER_SIZE]{};
		float tilt_acc[SMOOTHED_BUFFER_SIZE]{};
	} _smoothed;

	// Execution state
	hrt_abstime _trajectory_start_time{0};
	uint32_t _current_trajectory_sequence{0};

	// Last published setpoint (for hold mode)
	vla_setpoint_s _last_setpoint{};

	// === Parameters ===
	DEFINE_PARAMETERS(
		// Chassis constraints
		(ParamFloat<px4::params::VTP_CHS_VEL_MAX>) _param_vtp_chs_vel_max,
		(ParamFloat<px4::params::VTP_CHS_ACC_MAX>) _param_vtp_chs_acc_max,
		(ParamFloat<px4::params::VTP_CHS_JERK_MAX>) _param_vtp_chs_jerk_max,
		(ParamFloat<px4::params::VTP_CHS_YAW_MAX>) _param_vtp_chs_yaw_max,
		(ParamFloat<px4::params::VTP_CHS_YACC_MAX>) _param_vtp_chs_yacc_max,
		(ParamFloat<px4::params::VTP_STR_ANG_MAX>) _param_vtp_str_ang_max,
		(ParamFloat<px4::params::VTP_STR_RATE_MAX>) _param_vtp_str_rate_max,

		// Bucket height constraints (for smoothing, no IK here)
		(ParamFloat<px4::params::VTP_BKT_HGT_MIN>) _param_vtp_bkt_hgt_min,
		(ParamFloat<px4::params::VTP_BKT_HGT_MAX>) _param_vtp_bkt_hgt_max,
		(ParamFloat<px4::params::VTP_BKT_VEL_MAX>) _param_vtp_bkt_vel_max,
		(ParamFloat<px4::params::VTP_BKT_ACC_MAX>) _param_vtp_bkt_acc_max,
		(ParamFloat<px4::params::VTP_BKT_JERK_MAX>) _param_vtp_bkt_jerk_max,

		// Tilt constraints
		(ParamFloat<px4::params::VTP_TILT_ANG_MIN>) _param_vtp_tilt_ang_min,
		(ParamFloat<px4::params::VTP_TILT_ANG_MAX>) _param_vtp_tilt_ang_max,
		(ParamFloat<px4::params::VTP_TILT_VEL_MAX>) _param_vtp_tilt_vel_max,
		(ParamFloat<px4::params::VTP_TILT_ACC_MAX>) _param_vtp_tilt_acc_max,
		(ParamFloat<px4::params::VTP_TILT_JERK_MAX>) _param_vtp_tilt_jerk_max,

		// Synchronization
		(ParamFloat<px4::params::VTP_SYNC_MARGIN>) _param_vtp_sync_margin
	)
};
