/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file vla_proxy.hpp
 * @author PX4 Development Team
 *
 * VLA (Vision-Language-Action) proxy driver header
 * Follows ST3215 servo driver pattern for robust UART communication
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

using namespace time_literals;

class VLAProxy : public ModuleBase<VLAProxy>,
		 public ModuleParams,
		 public px4::ScheduledWorkItem
{
public:
	VLAProxy(const char *serial_port = "/dev/ttyS3");
	~VLAProxy() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	void Run() override;

	// Serial communication (following ST3215 pattern)
	bool configure_port();
	bool send_packet(const uint8_t *data, size_t length);
	bool receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);
	uint8_t calculate_checksum(const uint8_t *data, size_t length);

	// VLA communication protocol
	bool send_robot_status();
	bool receive_trajectory_commands();
	void process_vla_message();

	// Robot status structure (optimized for UART transmission)
	struct __attribute__((packed)) RobotStatus {
		uint64_t timestamp;
		float position[3];        // x, y, z in meters
		float velocity[3];        // vx, vy, vz in m/s
		float quaternion[4];      // w, x, y, z
		float angular_velocity[3]; // roll_rate, pitch_rate, yaw_rate in rad/s
		uint8_t armed;           // 0=disarmed, 1=armed
		uint8_t nav_state;       // Navigation state
		float battery_voltage;    // Battery voltage in volts
	};

	// VLA trajectory waypoint for 6DOF bucket control (optimized for UART)
	struct __attribute__((packed)) VLAWaypoint {
		uint64_t timestamp_us;
		float position[3];         // x, y, z position in meters
		float orientation[3];      // roll, pitch, yaw angles in radians
		float velocity[3];         // linear velocity (vx, vy, vz) in m/s
		float angular_velocity[3]; // angular velocity in rad/s
		float acceleration[3];     // linear acceleration in m/s²
		float angular_acceleration[3]; // angular acceleration in rad/s²
		uint8_t control_mode;      // Control mode for bucket
		uint8_t priority;          // Trajectory priority
	};

	// VLA Protocol constants
	static constexpr uint8_t VLA_HEADER1 = 0xAA;
	static constexpr uint8_t VLA_HEADER2 = 0x55;
	static constexpr uint8_t VLA_MSG_STATUS = 0x01;    // Robot status message
	static constexpr uint8_t VLA_MSG_WAYPOINT = 0x02;  // Trajectory waypoint
	static constexpr uint8_t VLA_MSG_COMMAND = 0x03;   // VLA command
	static constexpr uint8_t VLA_MSG_ACK = 0x04;       // Acknowledgment

	// Helper functions
	void publish_trajectory_setpoint(const VLAWaypoint &waypoint);
	void collect_robot_status(RobotStatus &status);
	bool validate_waypoint(const VLAWaypoint &waypoint);

	// Serial port
	int _uart{-1};
	char _port_name[32];

	// uORB topics
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<bucket_trajectory_setpoint_s> _bucket_trajectory_pub{ORB_ID(bucket_trajectory_setpoint)};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;
	perf_counter_t _packet_count_perf;
	perf_counter_t _status_sent_perf;
	perf_counter_t _waypoint_received_perf;

	// Communication state
	bool _connection_ok{false};
	bool _vla_active{false};
	int _consecutive_errors{0};
	hrt_abstime _last_status_sent{0};
	hrt_abstime _last_waypoint_received{0};
	hrt_abstime _last_connection_check{0};

	// Waypoint buffer (circular buffer for better performance)
	static constexpr size_t WAYPOINT_BUFFER_SIZE = 10;
	VLAWaypoint _waypoint_buffer[WAYPOINT_BUFFER_SIZE];
	size_t _waypoint_buffer_head{0};
	size_t _waypoint_buffer_tail{0};
	size_t _waypoint_buffer_count{0};

	// Timing constants
	static constexpr unsigned SCHEDULE_INTERVAL = 20_ms;  // 50Hz update rate
	static constexpr uint32_t PACKET_TIMEOUT_MS = 10;
	static constexpr uint32_t STATUS_SEND_INTERVAL_MS = 50;  // Send status at 20Hz
	static constexpr uint32_t CONNECTION_CHECK_INTERVAL_MS = 1000;  // Check connection every 1s

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VLA_ENABLED>) _param_vla_enabled,
		(ParamInt<px4::params::VLA_BAUDRATE>) _param_baudrate,
		(ParamFloat<px4::params::VLA_STATUS_RATE>) _param_status_rate,
		(ParamInt<px4::params::VLA_TIMEOUT_MS>) _param_timeout_ms,
		(ParamInt<px4::params::VLA_DEBUG>) _param_debug_level
	)
};
