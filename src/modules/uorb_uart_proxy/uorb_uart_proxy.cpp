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
 * @file uorb_uart_proxy.cpp
 * @author PX4 Development Team
 *
 * uORB UART Proxy implementation for NXT boards
 * Redesigned using ST3215 servo patterns for robust UART communication
 */

#include "uorb_uart_proxy.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

UorbUartProxy::UorbUartProxy(const char *serial_port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": loop")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_err")),
	_packet_count_perf(perf_alloc(PC_COUNT, MODULE_NAME": packets")),
	_tx_bytes_perf(perf_alloc(PC_COUNT, MODULE_NAME": tx_bytes")),
	_rx_bytes_perf(perf_alloc(PC_COUNT, MODULE_NAME": rx_bytes"))
{
	strncpy(_port_name, serial_port, sizeof(_port_name) - 1);
	_port_name[sizeof(_port_name) - 1] = '\0';
}

UorbUartProxy::~UorbUartProxy()
{
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
	perf_free(_packet_count_perf);
	perf_free(_tx_bytes_perf);
	perf_free(_rx_bytes_perf);
}

bool UorbUartProxy::init()
{
	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("uORB UART proxy started on %s", _port_name);
	return true;
}

bool UorbUartProxy::configure_port()
{
	// Close existing connection if open
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	// Check if enabled
	if (_param_enable.get() == 0) {
		PX4_DEBUG("uORB UART proxy disabled by parameter");
		return false;
	}

	// Get port from parameter
	int32_t port_param = _param_port.get();
	const char *device_path;
	switch (port_param) {
		case 1: device_path = "/dev/ttyS1"; break;
		case 2: device_path = "/dev/ttyS2"; break;
		case 3: device_path = "/dev/ttyS3"; break;
		case 4: device_path = "/dev/ttyS4"; break;
		default: 
			PX4_ERR("Invalid port parameter: %d", port_param);
			return false;
	}

	strncpy(_port_name, device_path, sizeof(_port_name) - 1);
	_port_name[sizeof(_port_name) - 1] = '\0';

	// Open serial port
	PX4_INFO("Opening serial port %s...", _port_name);
	_uart = ::open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart < 0) {
		PX4_ERR("Failed to open %s: %s", _port_name, strerror(errno));
		return false;
	}

	PX4_INFO("Serial port opened successfully (fd=%d)", _uart);

	// Configure port settings (same as bridge/ST3215)
	struct termios uart_config;

	if (tcgetattr(_uart, &uart_config) != 0) {
		PX4_ERR("Error getting serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Get baudrate parameter
	int32_t baudrate = _param_baudrate.get();
	if (baudrate <= 0) {
		baudrate = 921600;  // Default for uORB proxy
	}

	PX4_INFO("Configuring baudrate: %d", baudrate);

	speed_t speed;
	switch (baudrate) {
	case 9600:    speed = B9600; break;
	case 19200:   speed = B19200; break;
	case 38400:   speed = B38400; break;
	case 57600:   speed = B57600; break;
	case 115200:  speed = B115200; break;
	case 230400:  speed = B230400; break;
	case 460800:  speed = B460800; break;
	case 921600:  speed = B921600; break;
	case 1000000: speed = B1000000; break;
	default:
		PX4_WARN("Unsupported baudrate: %d, using 921600", baudrate);
		speed = B921600;
		break;
	}

	cfsetospeed(&uart_config, speed);
	cfsetispeed(&uart_config, speed);

	// Configure port settings (8N1, no flow control)
	uart_config.c_cflag &= ~PARENB;  // No parity
	uart_config.c_cflag &= ~CSTOPB;  // One stop bit
	uart_config.c_cflag &= ~CSIZE;   // Clear size bits
	uart_config.c_cflag |= CS8;      // 8 data bits
	uart_config.c_cflag &= ~CRTSCTS; // No hardware flow control
	uart_config.c_cflag |= CREAD | CLOCAL; // Enable reading and ignore modem control lines

	uart_config.c_lflag &= ~ICANON;  // Non-canonical mode
	uart_config.c_lflag &= ~ECHO;    // No echo
	uart_config.c_lflag &= ~ECHOE;   // No echo erase
	uart_config.c_lflag &= ~ECHONL;  // No echo newline
	uart_config.c_lflag &= ~ISIG;    // No signal processing

	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	uart_config.c_oflag &= ~OPOST;   // No output processing
	uart_config.c_oflag &= ~ONLCR;   // No CR to NL translation

	// Set timeouts for blocking reads
	uart_config.c_cc[VTIME] = 1;     // Wait for up to 0.1s (1 decisecond)
	uart_config.c_cc[VMIN] = 0;      // No minimum number of characters

	if (tcsetattr(_uart, TCSANOW, &uart_config) != 0) {
		PX4_ERR("Error setting serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Clear any existing data
	tcflush(_uart, TCIOFLUSH);

	return true;
}

void UorbUartProxy::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters
	updateParams();

	// Configure serial port if not already configured
	if (_uart < 0) {
		if (!configure_port()) {
			PX4_DEBUG("Failed to configure serial port, will retry later");
			perf_end(_loop_perf);
			return;
		}
	}

	// Process incoming messages (X7+ → NXT)
	process_incoming_messages();

	// Process outgoing messages (NXT → X7+)
	process_outgoing_messages();

	// Send periodic heartbeat
	hrt_abstime now = hrt_absolute_time();
	if (now - _last_heartbeat_time > HEARTBEAT_INTERVAL) {
		send_heartbeat();
		_last_heartbeat_time = now;
	}

	// Check connection timeout
	if (_connection_ok && (now - _last_update_time > CONNECTION_TIMEOUT)) {
		PX4_WARN("Connection timeout - no data received in %llu ms", (now - _last_update_time) / 1000);
		_connection_ok = false;
		_consecutive_errors = 0;
	}

	perf_end(_loop_perf);
}

bool UorbUartProxy::send_packet(const uint8_t *data, size_t length)
{
	if (_uart < 0 || !data || length == 0) {
		return false;
	}

	// Clear input buffer before sending to avoid stale data
	tcflush(_uart, TCIFLUSH);

	// Send the packet
	ssize_t bytes_written = ::write(_uart, data, length);
	if (bytes_written != (ssize_t)length) {
		perf_count(_comms_error_perf);
		return false;
	}

	// Wait for transmission to complete
	tcdrain(_uart);
	perf_count(_tx_bytes_perf);
	perf_add(_tx_bytes_perf, length);
	return true;
}

bool UorbUartProxy::receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
	if (_uart < 0 || !buffer || buffer_size == 0) {
		return false;
	}

	hrt_abstime start_time = hrt_absolute_time();
	size_t bytes_received = 0;

	// First, read at least the header (sync1, sync2, msg_id, length, sequence)
	const size_t header_size = 6;

	while (bytes_received < header_size && bytes_received < buffer_size) {
		// Check for timeout
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			return false;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			return false;
		}

		// Small delay to prevent CPU spinning
		usleep(500);
	}

	// Check if we have valid header
	if (bytes_received < header_size) {
		return false;
	}

	// Validate sync bytes
	if (buffer[0] != UART_SYNC_BYTE1 || buffer[1] != UART_SYNC_BYTE2) {
		return false;
	}

	// Parse packet length from header
	uint8_t payload_length = buffer[3];

	// Calculate total expected packet size
	size_t total_expected = header_size + payload_length + 1;

	if (total_expected > buffer_size) {
		return false;
	}

	// Read remaining bytes (payload + checksum)
	while (bytes_received < total_expected && bytes_received < buffer_size) {
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			break;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			return false;
		}
		usleep(500);
	}

	// Check if we got complete packet
	if (bytes_received < total_expected) {
		return false;
	}

	// Verify checksum
	uint8_t calculated_checksum = calculate_checksum(buffer, total_expected - 1);
	uint8_t received_checksum = buffer[total_expected - 1];

	if (calculated_checksum == received_checksum) {
		perf_count(_rx_bytes_perf);
		perf_add(_rx_bytes_perf, bytes_received);
		return true;
	} else {
		perf_count(_comms_error_perf);
		return false;
	}
}

uint8_t UorbUartProxy::calculate_checksum(const uint8_t *data, size_t length)
{
	uint8_t sum = 0;
	for (size_t i = 0; i < length; i++) {
		sum += data[i];
	}
	return ~sum;
}

void UorbUartProxy::process_incoming_messages()
{
	if (_uart < 0) {
		return;
	}

	uint8_t buffer[512];
	
	// Try to receive messages (non-blocking)
	while (receive_packet(buffer, sizeof(buffer), 5)) { // 5ms timeout per packet
		handle_received_frame(buffer, sizeof(buffer));
		_last_update_time = hrt_absolute_time();
		_connection_ok = true;
		_consecutive_errors = 0;
		perf_count(_packet_count_perf);
	}
}

void UorbUartProxy::process_outgoing_messages()
{
	if (_uart < 0) {
		return;
	}

	// Determine which instance this proxy represents based on board type
	int32_t board_type = _param_board_type.get(); // 0=front, 1=rear

	// Process HBridge status messages
	hbridge_status_s hbridge_status;
	if (_hbridge_status_sub_0.update(&hbridge_status)) {
		UartPacket packet;
		packet.sync1 = UART_SYNC_BYTE1;
		packet.sync2 = UART_SYNC_BYTE2;
		packet.msg_id = static_cast<uint8_t>(MessageId::HBRIDGE_STATUS);
		packet.length = sizeof(hbridge_status) + 1; // +1 for instance
		packet.sequence = _tx_sequence++;

		// Instance-based on board type: front=0,1 rear=2,3
		uint8_t instance = (board_type == 0) ? 0 : 2;
		packet.payload[0] = instance;
		memcpy(&packet.payload[1], &hbridge_status, sizeof(hbridge_status));

		size_t packet_size = 6 + packet.length + 1;
		uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
		packet.payload[packet.length] = checksum;

		if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
			perf_count(_packet_count_perf);
		}
	}

	if (_hbridge_status_sub_1.update(&hbridge_status)) {
		UartPacket packet;
		packet.sync1 = UART_SYNC_BYTE1;
		packet.sync2 = UART_SYNC_BYTE2;
		packet.msg_id = static_cast<uint8_t>(MessageId::HBRIDGE_STATUS);
		packet.length = sizeof(hbridge_status) + 1;
		packet.sequence = _tx_sequence++;

		uint8_t instance = (board_type == 0) ? 1 : 3;
		packet.payload[0] = instance;
		memcpy(&packet.payload[1], &hbridge_status, sizeof(hbridge_status));

		size_t packet_size = 6 + packet.length + 1;
		uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
		packet.payload[packet.length] = checksum;

		if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
			perf_count(_packet_count_perf);
		}
	}

	// Process sensor quad encoder messages
	sensor_quad_encoder_s encoder;
	if (_sensor_quad_encoder_sub_0.update(&encoder)) {
		UartPacket packet;
		packet.sync1 = UART_SYNC_BYTE1;
		packet.sync2 = UART_SYNC_BYTE2;
		packet.msg_id = static_cast<uint8_t>(MessageId::SENSOR_QUAD_ENCODER);
		packet.length = sizeof(encoder) + 1;
		packet.sequence = _tx_sequence++;

		uint8_t instance = (board_type == 0) ? 0 : 2;
		packet.payload[0] = instance;
		memcpy(&packet.payload[1], &encoder, sizeof(encoder));

		size_t packet_size = 6 + packet.length + 1;
		uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
		packet.payload[packet.length] = checksum;

		if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
			perf_count(_packet_count_perf);
		}
	}

	if (_sensor_quad_encoder_sub_1.update(&encoder)) {
		UartPacket packet;
		packet.sync1 = UART_SYNC_BYTE1;
		packet.sync2 = UART_SYNC_BYTE2;
		packet.msg_id = static_cast<uint8_t>(MessageId::SENSOR_QUAD_ENCODER);
		packet.length = sizeof(encoder) + 1;
		packet.sequence = _tx_sequence++;

		uint8_t instance = (board_type == 0) ? 1 : 3;
		packet.payload[0] = instance;
		memcpy(&packet.payload[1], &encoder, sizeof(encoder));

		size_t packet_size = 6 + packet.length + 1;
		uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
		packet.payload[packet.length] = checksum;

		if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
			perf_count(_packet_count_perf);
		}
	}

	// Process other status messages based on board type
	if (board_type == 1) { // Rear board handles boom/bucket
		boom_status_s boom_status;
		if (_boom_status_sub.update(&boom_status)) {
			UartPacket packet;
			packet.sync1 = UART_SYNC_BYTE1;
			packet.sync2 = UART_SYNC_BYTE2;
			packet.msg_id = static_cast<uint8_t>(MessageId::BOOM_STATUS);
			packet.length = sizeof(boom_status);
			packet.sequence = _tx_sequence++;

			memcpy(packet.payload, &boom_status, sizeof(boom_status));

			size_t packet_size = 6 + packet.length + 1;
			uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
			packet.payload[packet.length] = checksum;

			if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
				perf_count(_packet_count_perf);
			}
		}

		bucket_status_s bucket_status;
		if (_bucket_status_sub.update(&bucket_status)) {
			UartPacket packet;
			packet.sync1 = UART_SYNC_BYTE1;
			packet.sync2 = UART_SYNC_BYTE2;
			packet.msg_id = static_cast<uint8_t>(MessageId::BUCKET_STATUS);
			packet.length = sizeof(bucket_status);
			packet.sequence = _tx_sequence++;

			memcpy(packet.payload, &bucket_status, sizeof(bucket_status));

			size_t packet_size = 6 + packet.length + 1;
			uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
			packet.payload[packet.length] = checksum;

			if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
				perf_count(_packet_count_perf);
			}
		}
	}

	if (board_type == 0) { // Front board handles steering
		steering_status_s steering_status;
		if (_steering_status_sub.update(&steering_status)) {
			UartPacket packet;
			packet.sync1 = UART_SYNC_BYTE1;
			packet.sync2 = UART_SYNC_BYTE2;
			packet.msg_id = static_cast<uint8_t>(MessageId::STEERING_STATUS);
			packet.length = sizeof(steering_status);
			packet.sequence = _tx_sequence++;

			memcpy(packet.payload, &steering_status, sizeof(steering_status));

			size_t packet_size = 6 + packet.length + 1;
			uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
			packet.payload[packet.length] = checksum;

			if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
				perf_count(_packet_count_perf);
			}
		}
	}

	// Process slip estimation (both boards)
	slip_estimation_s slip_estimation;
	if (_slip_estimation_sub.update(&slip_estimation)) {
		UartPacket packet;
		packet.sync1 = UART_SYNC_BYTE1;
		packet.sync2 = UART_SYNC_BYTE2;
		packet.msg_id = (board_type == 0) ? static_cast<uint8_t>(MessageId::SLIP_ESTIMATION_FRONT) : 
						static_cast<uint8_t>(MessageId::SLIP_ESTIMATION_REAR);
		packet.length = sizeof(slip_estimation);
		packet.sequence = _tx_sequence++;

		memcpy(packet.payload, &slip_estimation, sizeof(slip_estimation));

		size_t packet_size = 6 + packet.length + 1;
		uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
		packet.payload[packet.length] = checksum;

		if (send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size)) {
			perf_count(_packet_count_perf);
		}
	}
}

void UorbUartProxy::handle_received_frame(const uint8_t *data, size_t length)
{
	if (!data || length < 7) { // Minimum packet size
		return;
	}

	uint8_t msg_id = data[2];
	uint8_t payload_length = data[3];
	uint16_t sequence = (data[5] << 8) | data[4];

	// Check for duplicate packets
	if (sequence == _last_rx_sequence) {
		return;
	}
	_last_rx_sequence = sequence;

	const uint8_t *payload = &data[6];

	int32_t board_type = _param_board_type.get();

	switch (static_cast<MessageId>(msg_id)) {
		case MessageId::ACTUATOR_OUTPUTS_FRONT: {
			if (payload_length == sizeof(actuator_outputs_s) && board_type == 0) {
				actuator_outputs_s actuator_outputs;
				memcpy(&actuator_outputs, payload, sizeof(actuator_outputs));
				_actuator_outputs_front_pub.publish(actuator_outputs);
			}
			break;
		}
		case MessageId::ACTUATOR_OUTPUTS_REAR: {
			if (payload_length == sizeof(actuator_outputs_s) && board_type == 1) {
				actuator_outputs_s actuator_outputs;
				memcpy(&actuator_outputs, payload, sizeof(actuator_outputs));
				_actuator_outputs_rear_pub.publish(actuator_outputs);
			}
			break;
		}
		case MessageId::VEHICLE_STATUS: {
			if (payload_length == sizeof(vehicle_status_s)) {
				vehicle_status_s vehicle_status;
				memcpy(&vehicle_status, payload, sizeof(vehicle_status));
				_vehicle_status_pub.publish(vehicle_status);
			}
			break;
		}
		case MessageId::BOOM_TRAJECTORY_SETPOINT: {
			if (payload_length == sizeof(boom_trajectory_setpoint_s) && board_type == 1) {
				boom_trajectory_setpoint_s boom_setpoint;
				memcpy(&boom_setpoint, payload, sizeof(boom_setpoint));
				_boom_trajectory_setpoint_pub.publish(boom_setpoint);
			}
			break;
		}
		case MessageId::BUCKET_TRAJECTORY_SETPOINT: {
			if (payload_length == sizeof(bucket_trajectory_setpoint_s) && board_type == 1) {
				bucket_trajectory_setpoint_s bucket_setpoint;
				memcpy(&bucket_setpoint, payload, sizeof(bucket_setpoint));
				_bucket_trajectory_setpoint_pub.publish(bucket_setpoint);
			}
			break;
		}
		case MessageId::STEERING_COMMAND: {
			if (payload_length == sizeof(steering_command_s) && board_type == 0) {
				steering_command_s steering_command;
				memcpy(&steering_command, payload, sizeof(steering_command));
				_steering_command_pub.publish(steering_command);
			}
			break;
		}
		default:
			break;
	}
}

void UorbUartProxy::send_heartbeat()
{
	if (_uart < 0) {
		return;
	}

	UartPacket packet;
	packet.sync1 = UART_SYNC_BYTE1;
	packet.sync2 = UART_SYNC_BYTE2;
	packet.msg_id = static_cast<uint8_t>(MessageId::HEARTBEAT);
	packet.length = 0;  // No payload for heartbeat
	packet.sequence = _tx_sequence++;

	size_t packet_size = 6 + 1; // header + checksum (no payload)
	uint8_t checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&packet), packet_size - 1);
	packet.payload[0] = checksum;

	send_packet(reinterpret_cast<uint8_t*>(&packet), packet_size);
}

int UorbUartProxy::print_status()
{
	PX4_INFO("uORB UART Proxy Status:");
	PX4_INFO("  Port: %s", _port_name);
	PX4_INFO("  UART FD: %d", _uart);
	PX4_INFO("  Board Type: %s", _param_board_type.get() == 0 ? "FRONT" : "REAR");
	PX4_INFO("  Connection: %s", _connection_ok ? "OK" : "TIMEOUT");
	PX4_INFO("  TX Sequence: %u", _tx_sequence);
	PX4_INFO("  Last RX Sequence: %u", _last_rx_sequence);
	PX4_INFO("  Consecutive Errors: %d", _consecutive_errors);
	
	if (_last_update_time > 0) {
		PX4_INFO("  Last Update: %llu ms ago", (hrt_absolute_time() - _last_update_time) / 1000);
	} else {
		PX4_INFO("  Last Update: Never");
	}

	// Print performance counters
	perf_print_counter(_loop_perf);
	perf_print_counter(_comms_error_perf);
	perf_print_counter(_packet_count_perf);
	perf_print_counter(_tx_bytes_perf);
	perf_print_counter(_rx_bytes_perf);

	return 0;
}

int UorbUartProxy::task_spawn(int argc, char *argv[])
{
	const char *serial_port = "/dev/ttyS1";

	// Parse command line arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;
		case '?':
			return print_usage("unrecognized flag");
		default:
			PX4_WARN("unrecognized flag");
			return -1;
		}
	}

	UorbUartProxy *instance = new UorbUartProxy(serial_port);

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

int UorbUartProxy::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UorbUartProxy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB UART Proxy module for NXT boards.

This module runs on NXT-Dual controller boards and provides transparent
uORB messaging to/from the X7+ main board via UART. It receives commands
from the main board and sends back status information.

Redesigned using ST3215 servo patterns for robust UART communication.

### Examples
Start the proxy (front board):
$ uorb_uart_proxy start

Check status:
$ uorb_uart_proxy status

Stop the proxy:
$ uorb_uart_proxy stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb_uart_proxy", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uorb_uart_proxy_main(int argc, char *argv[])
{
	return UorbUartProxy::main(argc, argv);
}
