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
 * @file uorb_uart_bridge.hpp
 * @author PX4 Development Team
 *
 * uORB UART Bridge module for X7+ main board
 * Bridges uORB messages between X7+ and NXT boards via UART
 * Redesigned using ST3215 servo patterns for robust UART communication
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/slip_estimation.h>
#include <uORB/topics/traction_control.h>
#include <uORB/topics/boom_trajectory_setpoint.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/steering_command.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/load_lamp_command.h>
#include <uORB/topics/sensor_quad_encoder.h>

#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

using namespace time_literals;

class UorbUartBridge : public ModuleBase<UorbUartBridge>,
		       public ModuleParams,
		       public px4::ScheduledWorkItem
{
public:
	UorbUartBridge(const char *serial_port = "/dev/ttyS3");
	~UorbUartBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	void Run() override;

	// Serial communication (ST3215 pattern)
	bool configure_port();
	bool send_packet(const uint8_t *data, size_t length);
	bool receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);
	uint8_t calculate_checksum(const uint8_t *data, size_t length);

	// Message processing
	void process_outgoing_messages();
	void process_incoming_messages();
	void send_heartbeat();
	void handle_received_frame(const uint8_t *data, size_t length);

	// Serial port (ST3215 pattern)
	int _uart{-1};
	char _port_name[32];

	// Communication state
	bool _connection_ok{false};
	hrt_abstime _last_update_time{0};
	hrt_abstime _last_heartbeat_time{0};
	int _consecutive_errors{0};

	// Sequence numbers for packet tracking
	uint16_t _tx_sequence{0};
	uint16_t _last_rx_sequence{0};

	// Performance counters (ST3215 pattern)
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;
	perf_counter_t _packet_count_perf;
	perf_counter_t _tx_bytes_perf;
	perf_counter_t _rx_bytes_perf;

	// Subscriptions for outgoing topics (X7+ → NXT)
	uORB::Subscription _actuator_outputs_front_sub{ORB_ID(actuator_outputs), 0};
	uORB::Subscription _actuator_outputs_rear_sub{ORB_ID(actuator_outputs), 1};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};
	uORB::Subscription _boom_trajectory_setpoint_sub{ORB_ID(boom_trajectory_setpoint)};
	uORB::Subscription _bucket_trajectory_setpoint_sub{ORB_ID(bucket_trajectory_setpoint)};
	uORB::Subscription _steering_command_sub{ORB_ID(steering_command)};
	uORB::Subscription _load_lamp_command_sub{ORB_ID(load_lamp_command)};

	// Publications for incoming topics (NXT → X7+)
	uORB::Publication<sensor_limit_switch_s> _limit_sensor_bucket_pub{ORB_ID(sensor_limit_switch)};
	uORB::Publication<slip_estimation_s> _slip_estimation_front_pub{ORB_ID(slip_estimation), 0};
	uORB::Publication<slip_estimation_s> _slip_estimation_rear_pub{ORB_ID(slip_estimation), 1};
	uORB::Publication<boom_status_s> _boom_status_pub{ORB_ID(boom_status)};
	uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
	uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};

	// HBridge status publications (multi-instance, NXT → X7+)
	uORB::Publication<hbridge_status_s> _hbridge_status_front_0_pub{ORB_ID(hbridge_status), 0};
	uORB::Publication<hbridge_status_s> _hbridge_status_front_1_pub{ORB_ID(hbridge_status), 1};
	uORB::Publication<hbridge_status_s> _hbridge_status_rear_0_pub{ORB_ID(hbridge_status), 2};
	uORB::Publication<hbridge_status_s> _hbridge_status_rear_1_pub{ORB_ID(hbridge_status), 3};

	// Sensor quad encoder publications (multi-instance, NXT → X7+)
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_front_0_pub{ORB_ID(sensor_quad_encoder), 0};
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_front_1_pub{ORB_ID(sensor_quad_encoder), 1};
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_rear_0_pub{ORB_ID(sensor_quad_encoder), 2};
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_rear_1_pub{ORB_ID(sensor_quad_encoder), 3};

	// Protocol constants
	static constexpr uint8_t UART_SYNC_BYTE1 = 0xFF;
	static constexpr uint8_t UART_SYNC_BYTE2 = 0xFE;
	static constexpr uint8_t PROTOCOL_VERSION = 0x01;

	// Message IDs (simplified from distributed_uorb)
	enum class MessageId : uint8_t {
		HEARTBEAT = 0x00,
		ACTUATOR_OUTPUTS_FRONT = 0x01,
		ACTUATOR_OUTPUTS_REAR = 0x02,
		VEHICLE_STATUS = 0x03,
		TRACTION_CONTROL = 0x04,
		BOOM_TRAJECTORY_SETPOINT = 0x05,
		BUCKET_TRAJECTORY_SETPOINT = 0x06,
		STEERING_COMMAND = 0x07,
		LOAD_LAMP_COMMAND = 0x08,
		SENSOR_LIMIT_SWITCH = 0x10,
		SLIP_ESTIMATION_FRONT = 0x11,
		SLIP_ESTIMATION_REAR = 0x12,
		BOOM_STATUS = 0x13,
		BUCKET_STATUS = 0x14,
		STEERING_STATUS = 0x15,
		HBRIDGE_STATUS = 0x20,  // Base ID, instance in payload
		SENSOR_QUAD_ENCODER = 0x30  // Base ID, instance in payload
	};

	// Packet structure (simplified)
	struct __attribute__((packed)) UartPacket {
		uint8_t sync1;
		uint8_t sync2;
		uint8_t msg_id;
		uint8_t length;
		uint16_t sequence;
		uint8_t payload[256];  // Variable length
		// Checksum at end
	};

	// Timing constants (ST3215 pattern)
	static constexpr unsigned SCHEDULE_INTERVAL = 20_ms;
	static constexpr uint32_t PACKET_TIMEOUT_MS = 50;
	static constexpr uint32_t HEARTBEAT_INTERVAL = 1_s;
	static constexpr uint32_t CONNECTION_TIMEOUT = 3_s;

	// Module parameters (ST3215 pattern)
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UART_BRIDGE_EN>) _param_enable,
		(ParamInt<px4::params::UART_BRIDGE_BAUD>) _param_baudrate,
		(ParamInt<px4::params::UART_BRIDGE_PORT>) _param_port
	)
};

extern "C" __EXPORT int uorb_uart_bridge_main(int argc, char *argv[]);
