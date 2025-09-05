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
 * @file uorb_uart_proxy.hpp
 * @author PX4 Development Team
 *
 * uORB UART Proxy module for NXT boards
 * Provides transparent uORB messaging to/from the X7+ main board via UART
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

/**
 * uORB UART Proxy module for NXT boards
 *
 * This module runs on NXT-Dual controller boards and provides transparent
 * uORB messaging to/from the X7+ main board via UART. It receives commands
 * from the main board and sends back status information.
 * Redesigned using ST3215 servo patterns for robust UART communication.
 */
class UorbUartProxy : public ModuleBase<UorbUartProxy>,
		      public ModuleParams,
		      public px4::ScheduledWorkItem
{
public:
	UorbUartProxy(const char *serial_port = "/dev/ttyS1");
	~UorbUartProxy();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	// Serial communication (ST3215 pattern)
	bool configure_port();
	bool send_packet(const uint8_t *data, size_t length);
	bool receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);
	uint8_t calculate_checksum(const uint8_t *data, size_t length);

	// Message processing
	void process_incoming_messages();
	void process_outgoing_messages();
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

	// Subscriptions for outgoing topics (NXT → X7+)
	uORB::Subscription _sensor_limit_switch_sub{ORB_ID(sensor_limit_switch)};
	uORB::Subscription _slip_estimation_sub{ORB_ID(slip_estimation)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};

	// HBridge status subscriptions (multi-instance, NXT → X7+)
	uORB::Subscription _hbridge_status_sub_0{ORB_ID(hbridge_status), 0};
	uORB::Subscription _hbridge_status_sub_1{ORB_ID(hbridge_status), 1};

	// Sensor quad encoder subscriptions (multi-instance, NXT → X7+)
	uORB::Subscription _sensor_quad_encoder_sub_0{ORB_ID(sensor_quad_encoder), 0};
	uORB::Subscription _sensor_quad_encoder_sub_1{ORB_ID(sensor_quad_encoder), 1};

	// Publications for incoming topics (X7+ → NXT)
	uORB::Publication<actuator_outputs_s> _actuator_outputs_front_pub{ORB_ID(actuator_outputs), 0};
	uORB::Publication<actuator_outputs_s> _actuator_outputs_rear_pub{ORB_ID(actuator_outputs), 1};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<traction_control_s> _traction_control_pub{ORB_ID(traction_control)};
	uORB::Publication<boom_trajectory_setpoint_s> _boom_trajectory_setpoint_pub{ORB_ID(boom_trajectory_setpoint)};
	uORB::Publication<bucket_trajectory_setpoint_s> _bucket_trajectory_setpoint_pub{ORB_ID(bucket_trajectory_setpoint)};
	uORB::Publication<steering_command_s> _steering_command_pub{ORB_ID(steering_command)};
	uORB::Publication<load_lamp_command_s> _load_lamp_command_pub{ORB_ID(load_lamp_command)};

	// Protocol constants (same as bridge)
	static constexpr uint8_t UART_SYNC_BYTE1 = 0xFF;
	static constexpr uint8_t UART_SYNC_BYTE2 = 0xFE;
	static constexpr uint8_t PROTOCOL_VERSION = 0x01;

	// Message IDs (same as bridge)
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

	// Packet structure (same as bridge)
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
		(ParamInt<px4::params::UART_PROXY_EN>) _param_enable,
		(ParamInt<px4::params::UART_PROXY_BAUD>) _param_baudrate,
		(ParamInt<px4::params::UART_PROXY_PORT>) _param_port,
		(ParamInt<px4::params::UART_PROXY_TYPE>) _param_board_type  // 0=front, 1=rear
	)
};

extern "C" __EXPORT int uorb_uart_proxy_main(int argc, char *argv[]);
