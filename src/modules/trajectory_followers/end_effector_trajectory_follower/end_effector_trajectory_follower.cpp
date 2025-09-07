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

#include "end_effector_trajectory_follower.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/time.h>

using namespace matrix;
using namespace wheel_loader;

EndEffectorTrajectoryFollower::EndEffectorTrajectoryFollower() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME)),
	_kinematics_perf(perf_alloc(PC_ELAPSED, MODULE_NAME"_kinematics"))
{
	_last_boom_angle = 0.0f;
	_last_bucket_angle = 0.0f;
}

EndEffectorTrajectoryFollower::~EndEffectorTrajectoryFollower()
{
	// Clean up performance counters
	perf_free(_loop_perf);
	perf_free(_kinematics_perf);
}

bool EndEffectorTrajectoryFollower::init()
{
	// Schedule at fixed interval for smooth trajectory following
	ScheduleOnInterval(CONTROL_LOOP_INTERVAL_US);

	PX4_INFO("End Effector Trajectory Follower initialized (rate: %.1f Hz)",
	         (double)(1e6f / CONTROL_LOOP_INTERVAL_US));

	return true;
}

void EndEffectorTrajectoryFollower::Run()
{
	// Check for module exit
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters if they have changed
	updateParams();

	// Process new end effector trajectory setpoints
	end_effector_trajectory_setpoint_s setpoint{};
	if (_end_effector_trajectory_setpoint_sub.update(&setpoint)) {

		// Validate setpoint before processing
		if (is_valid_setpoint(setpoint)) {
			update_trajectory_generation(setpoint);
		} else {
			PX4_WARN("Invalid end effector setpoint received");
		}
	}

	perf_end(_loop_perf);
}

bool EndEffectorTrajectoryFollower::is_valid_setpoint(const end_effector_trajectory_setpoint_s &setpoint) const
{
	// Check timestamp validity
	if (setpoint.timestamp == 0) {
		return false;
	}

	// Check boom angle limits (chassis frame)
	const float boom_angle_limit = M_PI_F / 2.0f;  // ±90 degrees
	if (setpoint.boom_angle < -boom_angle_limit || setpoint.boom_angle > boom_angle_limit) {
		return false;
	}

	// Check bucket angle limits (boom frame)
	const float bucket_angle_limit = M_PI_F / 2.0f;  // ±90 degrees
	if (setpoint.bucket_angle < -bucket_angle_limit || setpoint.bucket_angle > bucket_angle_limit) {
		return false;
	}

	// Check for NaN values
	if (!PX4_ISFINITE(setpoint.boom_angle) || !PX4_ISFINITE(setpoint.bucket_angle)) {
		return false;
	}

	// Check rate limits if provided
	if (PX4_ISFINITE(setpoint.boom_angle_rate)) {
		const float max_boom_rate = 2.0f; // rad/s
		if (fabsf(setpoint.boom_angle_rate) > max_boom_rate) {
			return false;
		}
	}

	if (PX4_ISFINITE(setpoint.bucket_angle_rate)) {
		const float max_bucket_rate = 2.0f; // rad/s
		if (fabsf(setpoint.bucket_angle_rate) > max_bucket_rate) {
			return false;
		}
	}

	return true;
}

void EndEffectorTrajectoryFollower::update_trajectory_generation(const end_effector_trajectory_setpoint_s &setpoint)
{
	perf_begin(_kinematics_perf);

	// Get current system state
	SystemState current_state;
	if (!get_current_system_state(current_state)) {
		PX4_WARN("Failed to get system state");
		perf_end(_kinematics_perf);
		return;
	}

	// For 2DOF joint control, use the joint angles directly
	float target_boom_angle = setpoint.boom_angle;
	float target_bucket_angle = setpoint.bucket_angle;

	// Generate smooth trajectory commands
	generate_trajectory_commands(target_boom_angle, target_bucket_angle, current_state);

	// Update trajectory state
	_trajectory_active = true;
	_last_update_time = hrt_absolute_time();
	_last_boom_angle = setpoint.boom_angle;
	_last_bucket_angle = setpoint.bucket_angle;

	perf_end(_kinematics_perf);
}

bool EndEffectorTrajectoryFollower::get_current_system_state(SystemState &state)
{
	// Get vehicle position
	vehicle_local_position_s vehicle_pos{};
	if (!_vehicle_local_position_sub.copy(&vehicle_pos)) {
		PX4_DEBUG("No vehicle position data available");
		return false;
	}

	// Get boom status
	if (!_boom_status_sub.copy(&state.boom_status)) {
		PX4_DEBUG("No boom status data available");
		return false;
	}

	// Get bucket status
	if (!_bucket_status_sub.copy(&state.bucket_status)) {
		PX4_DEBUG("No bucket status data available");
		return false;
	}

	// Fill state structure
	state.chassis_position = Vector3f(vehicle_pos.x, vehicle_pos.y, vehicle_pos.z);
	state.chassis_orientation = Quatf(Eulerf(vehicle_pos.heading, 0.0f, 0.0f));
	state.timestamp = hrt_absolute_time();

	return true;
}

void EndEffectorTrajectoryFollower::generate_trajectory_commands(
	float target_boom_angle,
	float target_bucket_angle,
	const SystemState &current_state)
{
	// Calculate velocities based on current status and target angles
	const float boom_velocity = calculate_boom_velocity(target_boom_angle, current_state.boom_status);
	const float bucket_velocity = calculate_bucket_velocity(target_bucket_angle, current_state.bucket_status);

	// Generate and publish trajectory setpoints
	generate_boom_trajectory(target_boom_angle, boom_velocity);
	generate_bucket_trajectory(target_bucket_angle, bucket_velocity);

	// Store last commanded angles
	_last_boom_angle = target_boom_angle;
	_last_bucket_angle = target_bucket_angle;
}

bool EndEffectorTrajectoryFollower::compute_joint_angles(
	const Vector3f &end_effector_position_chassis,
	const Quatf &end_effector_orientation_chassis,
	float &boom_angle_chassis,
	float &bucket_angle_boom) const
{
	// Extract position components relative to boom pivot point
	const float x = end_effector_position_chassis(0);
	const float y = end_effector_position_chassis(1);
	const float z = end_effector_position_chassis(2) - BOOM_PIVOT_HEIGHT;

	// Calculate horizontal reach (2D projection)
	const float horizontal_reach = sqrtf(x * x + y * y);

	// Check workspace limits
	const float max_reach = BOOM_LENGTH + BUCKET_LENGTH;
	const float min_reach = fabsf(BOOM_LENGTH - BUCKET_LENGTH);

	if (horizontal_reach > max_reach) {
		PX4_DEBUG("Target unreachable: horizontal reach %.2f > max %.2f",
		          (double)horizontal_reach, (double)max_reach);
		return false;
	}

	if (horizontal_reach < min_reach) {
		PX4_DEBUG("Target too close: horizontal reach %.2f < min %.2f",
		          (double)horizontal_reach, (double)min_reach);
		return false;
	}

	// Calculate boom angle using 2-DOF inverse kinematics
	// For simplified kinematic model: boom angle from chassis horizontal
	boom_angle_chassis = atan2f(z, horizontal_reach);

	// Extract bucket angle from end effector orientation
	// Assuming pitch component represents bucket tilt relative to boom
	const Eulerf euler_angles(end_effector_orientation_chassis);
	bucket_angle_boom = euler_angles.theta(); // Pitch angle

	// Apply joint limits and validate solution
	boom_angle_chassis = math::constrain(boom_angle_chassis, MIN_BOOM_ANGLE, MAX_BOOM_ANGLE);
	bucket_angle_boom = math::constrain(bucket_angle_boom, MIN_BUCKET_ANGLE, MAX_BUCKET_ANGLE);

	// Validate the computed solution using forward kinematics
	const Vector3f computed_position = compute_end_effector_position(boom_angle_chassis, bucket_angle_boom);
	const Vector3f position_error = computed_position - end_effector_position_chassis;

	if (position_error.norm() > POSITION_TOLERANCE_M) {
		PX4_DEBUG("IK solution validation failed: error %.3f m", (double)position_error.norm());
		return false;
	}

	return true;
}

Vector3f EndEffectorTrajectoryFollower::compute_end_effector_position(
	float boom_angle_chassis,
	float bucket_angle_boom) const
{
	// Forward kinematics for validation and trajectory planning
	const float boom_x = BOOM_LENGTH * cosf(boom_angle_chassis);
	const float boom_z = BOOM_LENGTH * sinf(boom_angle_chassis);

	const float bucket_x = boom_x + BUCKET_LENGTH * cosf(boom_angle_chassis + bucket_angle_boom);
	const float bucket_z = boom_z + BUCKET_LENGTH * sinf(boom_angle_chassis + bucket_angle_boom);

	return Vector3f(bucket_x, 0.0f, bucket_z + BOOM_PIVOT_HEIGHT);
}

float EndEffectorTrajectoryFollower::calculate_boom_velocity(
	float target_boom_angle,
	const boom_status_s &boom_status) const
{
	const float angle_error = target_boom_angle - boom_status.angle;
	const float max_velocity = _param_max_velocity.get();

	// Proportional velocity control with saturation
	const float commanded_velocity = math::constrain(
		angle_error * VELOCITY_PROPORTIONAL_GAIN,
		-max_velocity,
		max_velocity
	);

	return commanded_velocity;
}

float EndEffectorTrajectoryFollower::calculate_bucket_velocity(
	float target_bucket_angle,
	const bucket_status_s &bucket_status) const
{
	const float angle_error = target_bucket_angle - bucket_status.bucket_angle;
	const float max_velocity = _param_max_velocity.get();

	// Proportional velocity control with saturation
	const float commanded_velocity = math::constrain(
		angle_error * VELOCITY_PROPORTIONAL_GAIN,
		-max_velocity,
		max_velocity
	);

	return commanded_velocity;
}

void EndEffectorTrajectoryFollower::generate_boom_trajectory(float target_boom_angle, float boom_velocity)
{
	boom_trajectory_setpoint_s boom_setpoint{};
	boom_setpoint.timestamp = hrt_absolute_time();
	boom_setpoint.angle = target_boom_angle;
	boom_setpoint.angular_velocity = boom_velocity;

	// Publish the trajectory setpoint
	if (_boom_trajectory_pub.publish(boom_setpoint)) {
		_last_boom_angle = target_boom_angle;
	} else {
		PX4_WARN("Failed to publish boom trajectory setpoint");
	}
}

void EndEffectorTrajectoryFollower::generate_bucket_trajectory(float target_bucket_angle, float bucket_velocity)
{
	bucket_trajectory_setpoint_s bucket_setpoint{};
	bucket_setpoint.timestamp = hrt_absolute_time();
	bucket_setpoint.target_angle = target_bucket_angle;
	bucket_setpoint.angular_velocity = bucket_velocity;

	// Publish the trajectory setpoint
	if (_bucket_trajectory_pub.publish(bucket_setpoint)) {
		_last_bucket_angle = target_bucket_angle;
	} else {
		PX4_WARN("Failed to publish bucket trajectory setpoint");
	}
}

Vector3f EndEffectorTrajectoryFollower::transform_world_to_chassis(
	const Vector3f &position_world,
	const Vector3f &chassis_position,
	const Quatf &chassis_orientation) const
{
	// Transform from world coordinates to chassis-relative coordinates
	const Vector3f relative_position = position_world - chassis_position;
	return chassis_orientation.rotateVectorInverse(relative_position);
}

int EndEffectorTrajectoryFollower::print_status()
{
	PX4_INFO("End Effector Trajectory Follower Status:");
	PX4_INFO("  Trajectory active: %s", _trajectory_active ? "yes" : "no");

	// Current joint angles
	PX4_INFO("  Last boom angle: %.2f rad (%.1f deg)",
	         (double)_last_boom_angle,
	         (double)math::degrees(_last_boom_angle));
	PX4_INFO("  Last bucket angle: %.2f rad (%.1f deg)",
	         (double)_last_bucket_angle,
	         (double)math::degrees(_last_bucket_angle));

	// Compute current end effector position from joint angles
	matrix::Vector3f current_position = compute_end_effector_position(_last_boom_angle, _last_bucket_angle);
	PX4_INFO("  Computed EE position: [%.2f, %.2f, %.2f] m",
	         (double)current_position(0),
	         (double)current_position(1),
	         (double)current_position(2));

	// Timing information
	const hrt_abstime now = hrt_absolute_time();
	if (_last_update_time > 0) {
		const float time_since_last_update = (now - _last_update_time) / 1e6f;
		PX4_INFO("  Time since last update: %.3f s", (double)time_since_last_update);
	}

	// Performance counters
	PX4_INFO("Performance Counters:");
	perf_print_counter(_loop_perf);
	perf_print_counter(_kinematics_perf);

	return 0;
}

int EndEffectorTrajectoryFollower::task_spawn(int argc, char *argv[])
{
	EndEffectorTrajectoryFollower *instance = new EndEffectorTrajectoryFollower();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int EndEffectorTrajectoryFollower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EndEffectorTrajectoryFollower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
End effector trajectory follower for wheel loader. Takes end effector trajectory setpoints
(bucket position/orientation from system perspective) and generates boom and bucket trajectory
setpoints using inverse kinematics.

The module:
- Subscribes to end_effector_trajectory_setpoint
- Uses boom_status and bucket_status for current state feedback
- Performs inverse kinematics to calculate boom and bucket movements
- Publishes boom_trajectory_setpoint and bucket_trajectory_setpoint

### Usage
end_effector_trajectory_follower <command> [arguments...]

### Commands
  start|stop|status   Start/stop/show status of the module
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("end_effector_trajectory_follower", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}
