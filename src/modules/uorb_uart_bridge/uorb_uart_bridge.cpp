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
 * @file uorb_uart_bridge.cpp
 * @author PX4 Development Team
 *
 * Refactored uORB UART Bridge implementation
 * Uses improved distributed uORB protocol for robust communication
 */

#include "uorb_uart_bridge.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace distributed_uorb;

UorbUartBridge::UorbUartBridge() :
	ModuleBase(MODULE_NAME),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_errors")),
	_packet_tx_perf(perf_alloc(PC_COUNT, MODULE_NAME": tx_packets")),
	_packet_rx_perf(perf_alloc(PC_COUNT, MODULE_NAME": rx_packets")),
	_bytes_tx_perf(perf_alloc(PC_COUNT, MODULE_NAME": tx_bytes")),
	_bytes_rx_perf(perf_alloc(PC_COUNT, MODULE_NAME": rx_bytes"))
{
}

UorbUartBridge::~UorbUartBridge()
{
	cleanup_topic_handlers();

	for (auto &conn : _connections) {
		close_connection(conn);
	}

	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
	perf_free(_packet_tx_perf);
	perf_free(_packet_rx_perf);
	perf_free(_bytes_tx_perf);
	perf_free(_bytes_rx_perf);
}

bool UorbUartBridge::init()
{
	if (!configure_connections()) {
		PX4_ERR("Failed to configure connections");
		return false;
	}

	if (!setup_topic_handlers()) {
		PX4_ERR("Failed to setup topic handlers");
		return false;
	}

	ScheduleOnInterval(SCHEDULE_INTERVAL);
	PX4_INFO("uORB UART Bridge initialized");
	return true;
}

bool UorbUartBridge::configure_connections()
{
	_connections.clear();
	_connections.reserve(2);

	// Front NXT connection
	RemoteNodeConnection front_conn{};
	front_conn.node_id = NodeId::NXT_FRONT;
	snprintf(front_conn.device_path, sizeof(front_conn.device_path), "/dev/ttyS%d", _param_front_port.get());
	front_conn.uart_fd = -1;
	front_conn.is_connected = false;
	front_conn.last_heartbeat = 0;
	front_conn.last_message = 0;
	front_conn.tx_sequence = 0;
	front_conn.last_rx_sequence = 0;
	_connections.push_back(front_conn);

	// Rear NXT connection
	RemoteNodeConnection rear_conn{};
	rear_conn.node_id = NodeId::NXT_REAR;
	snprintf(rear_conn.device_path, sizeof(rear_conn.device_path), "/dev/ttyS%d", _param_rear_port.get());
	rear_conn.uart_fd = -1;
	rear_conn.is_connected = false;
	rear_conn.last_heartbeat = 0;
	rear_conn.last_message = 0;
	rear_conn.tx_sequence = 0;
	rear_conn.last_rx_sequence = 0;
	_connections.push_back(rear_conn);

	return true;
}

bool UorbUartBridge::open_connection(RemoteNodeConnection &conn)
{
	if (conn.is_connected) {
		return true;
	}

	conn.uart_fd = open(conn.device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (conn.uart_fd < 0) {
		handle_communication_error(conn, "Failed to open UART");
		return false;
	}

	struct termios config;
	memset(&config, 0, sizeof(config));

	if (tcgetattr(conn.uart_fd, &config) != 0) {
		handle_communication_error(conn, "Failed to get UART config");
		close(conn.uart_fd);
		conn.uart_fd = -1;
		return false;
	}

	// Configure UART (8N1, no flow control)
	config.c_cflag = CS8 | CREAD | CLOCAL;
	config.c_iflag = IGNPAR;
	config.c_oflag = 0;
	config.c_lflag = 0;
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 0;

	cfsetospeed(&config, _param_baudrate.get());
	cfsetispeed(&config, _param_baudrate.get());

	if (tcsetattr(conn.uart_fd, TCSANOW, &config) != 0) {
		handle_communication_error(conn, "Failed to set UART config");
		close(conn.uart_fd);
		conn.uart_fd = -1;
		return false;
	}

	conn.is_connected = true;
	conn.tx_errors = 0;
	conn.rx_errors = 0;

	// Initialize TX queue
	tx_queue_init(conn);

	PX4_INFO("Opened connection to node %d on %s", static_cast<int>(conn.node_id), conn.device_path);
	return true;
}

void UorbUartBridge::close_connection(RemoteNodeConnection &conn)
{
	if (conn.uart_fd >= 0) {
		close(conn.uart_fd);
		conn.uart_fd = -1;
	}

	conn.is_connected = false;
}

void UorbUartBridge::Run()
{
	if (should_exit()) {
		return;
	}

	perf_begin(_loop_perf);

	// Check and maintain connections
	check_connections();

	// Process messages in both directions
	process_outgoing_messages();
	process_incoming_messages();

	// Send periodic heartbeat
	send_heartbeat();

	// Update statistics
	update_statistics();

	perf_end(_loop_perf);
}

void UorbUartBridge::check_connections()
{
	const hrt_abstime now = hrt_absolute_time();

	for (auto &conn : _connections) {
		if (!open_connection(conn)) {
			continue;
		}

		// Check for connection timeout
		if (conn.last_heartbeat > 0 &&
			(now - conn.last_heartbeat) > (CONNECTION_TIMEOUT_MS * 1000)) {

			PX4_WARN("Connection timeout for node %d", static_cast<int>(conn.node_id));
			close_connection(conn);
			open_connection(conn); // Try to reconnect
		}
	}

	// Update overall connection status
	_all_connections_ready = true;
	for (const auto &conn : _connections) {
		if (!conn.is_connected) {
			_all_connections_ready = false;
			break;
		}
	}
}

void UorbUartBridge::send_heartbeat()
{
	const hrt_abstime now = hrt_absolute_time();

	if ((now - _last_heartbeat_time) >= (HEARTBEAT_INTERVAL_MS * 1000)) {
		for (auto &conn : _connections) {
			if (!conn.is_connected) {
				continue;
			}

			ProtocolFrame frame;
			if (_frame_builder.create_heartbeat_frame(&frame, NodeId::X7_MAIN, conn.node_id, conn.tx_sequence++)) {
				send_frame(conn, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
				perf_count(_packet_tx_perf);
			}
		}

		_last_heartbeat_time = now;
	}
}

bool UorbUartBridge::setup_topic_handlers()
{
	// Initialize topic registry with all known topics
	DistributedTopicRegistry &registry = DistributedTopicRegistry::instance();

	// Register common topics used in wheel loader communication
	registry.register_topic("actuator_outputs", ORB_ID(actuator_outputs));
	registry.register_topic("vehicle_status", ORB_ID(vehicle_status));
	registry.register_topic("boom_trajectory_setpoint", ORB_ID(boom_trajectory_setpoint));
	registry.register_topic("bucket_trajectory_setpoint", ORB_ID(bucket_trajectory_setpoint));
	registry.register_topic("steering_command", ORB_ID(steering_command));
	registry.register_topic("boom_status", ORB_ID(boom_status));
	registry.register_topic("bucket_status", ORB_ID(bucket_status));
	registry.register_topic("steering_status", ORB_ID(steering_status));
	registry.register_topic("sensor_limit_switch", ORB_ID(sensor_limit_switch));
	registry.register_topic("sensor_quad_encoder", ORB_ID(sensor_quad_encoder));
	registry.register_topic("slip_estimation", ORB_ID(slip_estimation));
	registry.register_topic("hbridge_status", ORB_ID(hbridge_status));

	PX4_INFO("Registered %zu topics in distributed registry", registry.get_topic_count());
	return true;
}

void UorbUartBridge::cleanup_topic_handlers()
{
	for (auto &handler : _topic_handlers) {
		if (handler.subscription != nullptr) {
			delete handler.subscription;
			handler.subscription = nullptr;
		}

		if (handler.publication != nullptr) {
			orb_unadvertise(handler.publication);
			handler.publication = nullptr;
		}
	}

	_topic_handlers.clear();
}

void UorbUartBridge::process_outgoing_messages()
{
	DistributedTopicRegistry &registry = DistributedTopicRegistry::instance();

	// Check all registered topics for updates to send to NXT nodes
	const auto topics = registry.get_all_topics();

	for (const auto &topic_info : topics) {
		// Create subscription if needed
		uORB::Subscription *sub = new uORB::Subscription(topic_info.meta);

		// Check for new data
		if (sub->updated()) {
			uint8_t data[topic_info.meta->o_size];

			if (sub->copy(data)) {
				// Send to all connected nodes
				for (auto &conn : _connections) {
					if (!conn.is_connected) {
						continue;
					}

					ProtocolFrame frame;
					if (_frame_builder.create_data_frame(&frame, NodeId::X7_MAIN, conn.node_id,
											topic_info.topic_id, 0, data, topic_info.meta->o_size,
											conn.tx_sequence++)) {
						send_frame(conn, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
						perf_count(_packet_tx_perf);
						perf_count_n(_bytes_tx_perf, sizeof(frame));
					}
				}
			}
		}

		delete sub;
	}
}

void UorbUartBridge::process_incoming_messages()
{
	for (auto &conn : _connections) {
		if (conn.is_connected) {
			receive_frames(conn);
		}
	}
}

bool UorbUartBridge::send_frame(RemoteNodeConnection &conn, const uint8_t *frame_data, size_t frame_size)
{
	if (!conn.is_connected || conn.uart_fd < 0) {
		return false;
	}

	ssize_t written = write(conn.uart_fd, frame_data, frame_size);

	if (written != static_cast<ssize_t>(frame_size)) {
		conn.tx_errors++;
		handle_communication_error(conn, "UART write failed");
		return false;
	}

	conn.tx_packets++;
	return true;
}

bool UorbUartBridge::receive_frames(RemoteNodeConnection &conn)
{
	if (!conn.is_connected || conn.uart_fd < 0) {
		return false;
	}

	uint8_t buffer[512];
	ssize_t bytes_read = read(conn.uart_fd, buffer, sizeof(buffer));

	if (bytes_read > 0) {
		conn.last_message = hrt_absolute_time();
		perf_count_n(_bytes_rx_perf, bytes_read);

		// Parse received data for complete frames
		for (ssize_t i = 0; i < bytes_read; ) {
			const ProtocolFrame *frame = nullptr;
			size_t frame_size = 0;

			if (_frame_parser.parse_frame(&buffer[i], bytes_read - i, &frame, &frame_size)) {
				handle_received_frame(conn, frame);
				conn.rx_packets++;
				perf_count(_packet_rx_perf);
				i += frame_size;
			} else {
				i++; // Skip this byte and continue parsing
			}
		}

		return true;
	}

	return false;
}

void UorbUartBridge::handle_received_frame(RemoteNodeConnection &conn, const ProtocolFrame *frame)
{
	if (frame == nullptr) {
		return;
	}

	// Update heartbeat timestamp
	conn.last_heartbeat = hrt_absolute_time();

	switch (frame->message_type) {
	case MessageType::DATA: {
		// Publish received topic data to local uORB
		publish_to_local_topic(frame->topic_id, frame->instance, frame->payload, frame->payload_size);
		break;
	}

	case MessageType::HEARTBEAT: {
		// Connection is alive, no action needed
		break;
	}

	case MessageType::ACK: {
		// Handle acknowledgment if needed
		break;
	}

	default:
		PX4_WARN("Unknown message type: %d", static_cast<int>(frame->message_type));
		break;
	}
}

bool UorbUartBridge::publish_to_local_topic(uint16_t topic_id, uint8_t instance, const void *data, size_t data_size)
{
	DistributedTopicRegistry &registry = DistributedTopicRegistry::instance();
	const orb_metadata *meta = registry.get_topic_metadata(topic_id);

	if (meta == nullptr) {
		PX4_WARN("Unknown topic ID: %d", topic_id);
		return false;
	}

	if (data_size != meta->o_size) {
		PX4_WARN("Size mismatch for topic %s: expected %zu, got %zu", meta->o_name, meta->o_size, data_size);
		return false;
	}

	// Find or create topic handler
	TopicHandler *handler = find_topic_handler(topic_id);

	if (handler == nullptr) {
		// Create new handler
		TopicHandler new_handler{};
		new_handler.topic_id = topic_id;
		new_handler.meta = meta;
		new_handler.subscription = nullptr;
		new_handler.publication = orb_advertise_multi(meta, data, &instance);
		new_handler.last_published = hrt_absolute_time();
		new_handler.publish_count = 1;
		new_handler.is_outgoing = false;
		_topic_handlers.push_back(new_handler);
	} else {
		// Update existing publication
		if (handler->publication != nullptr) {
			orb_publish(meta, handler->publication, data);
			handler->last_published = hrt_absolute_time();
			handler->publish_count++;
		}
	}

	return true;
}

TopicHandler *UorbUartBridge::find_topic_handler(uint16_t topic_id)
{
	for (auto &handler : _topic_handlers) {
		if (handler.topic_id == topic_id) {
			return &handler;
		}
	}

	return nullptr;
}

void UorbUartBridge::handle_communication_error(RemoteNodeConnection &conn, const char *error_msg)
{
	conn.tx_errors++;
	perf_count(_comms_error_perf);
	PX4_WARN("Node %d error: %s", static_cast<int>(conn.node_id), error_msg);
}

void UorbUartBridge::update_statistics()
{
	if (!_param_enable_stats.get()) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if ((now - _last_statistics_time) >= (STATISTICS_INTERVAL_MS * 1000)) {
		PX4_INFO("=== uORB UART Bridge Statistics ===");

		for (const auto &conn : _connections) {
			print_connection_status(conn);
		}

		PX4_INFO("Topic handlers: %zu", _topic_handlers.size());
		_last_statistics_time = now;
	}
}

void UorbUartBridge::print_connection_status(const RemoteNodeConnection &conn) const
{
	const char *node_name = (conn.node_id == NodeId::NXT_FRONT) ? "FRONT" : "REAR";

	PX4_INFO("Node %s: %s, TX: %u/%u, RX: %u/%u",
		node_name,
		conn.is_connected ? "CONNECTED" : "DISCONNECTED",
		conn.tx_packets, conn.tx_errors,
		conn.rx_packets, conn.rx_errors);
}

int UorbUartBridge::print_status()
{
	PX4_INFO("uORB UART Bridge status:");
	PX4_INFO("  Connections: %zu", _connections.size());
	PX4_INFO("  All ready: %s", _all_connections_ready ? "YES" : "NO");
	PX4_INFO("  Topic handlers: %zu", _topic_handlers.size());

	for (const auto &conn : _connections) {
		print_connection_status(conn);
	}

	return 0;
}

int UorbUartBridge::task_spawn(int argc, char *argv[])
{
	UorbUartBridge *instance = new UorbUartBridge();

	if (instance == nullptr) {
		PX4_ERR("Failed to allocate instance");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		PX4_ERR("Failed to initialize");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int UorbUartBridge::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = UorbUartBridge::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int UorbUartBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB UART Bridge for distributed messaging on Wheel Loader Robot.

This module runs on the X7+ main board and provides communication with
Front and Rear NXT controller boards via UART. It uses an improved
distributed uORB protocol with CRC32 validation and dynamic topic registry.

### Examples
Start the bridge:
$ uorb_uart_bridge start

Check status:
$ uorb_uart_bridge status

Stop the bridge:
$ uorb_uart_bridge stop
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb_uart_bridge", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

// TX Queue management implementation (fixed-size circular buffer)
void UorbUartBridge::tx_queue_init(RemoteNodeConnection &conn)
{
	conn.tx_queue_head = 0;
	conn.tx_queue_tail = 0;
	conn.tx_queue_count = 0;
}

bool UorbUartBridge::tx_queue_push(RemoteNodeConnection &conn, const uint8_t *data, size_t size)
{
	if (tx_queue_full(conn) || size > RemoteNodeConnection::MAX_PACKET_SIZE) {
		return false;
	}

	memcpy(conn.tx_buffer[conn.tx_queue_tail], data, size);
	conn.tx_packet_sizes[conn.tx_queue_tail] = size;
	conn.tx_queue_tail = (conn.tx_queue_tail + 1) % RemoteNodeConnection::TX_QUEUE_SIZE;
	conn.tx_queue_count++;
	return true;
}

bool UorbUartBridge::tx_queue_pop(RemoteNodeConnection &conn, uint8_t *data, size_t *size)
{
	if (tx_queue_empty(conn)) {
		return false;
	}

	*size = conn.tx_packet_sizes[conn.tx_queue_head];
	memcpy(data, conn.tx_buffer[conn.tx_queue_head], *size);
	conn.tx_queue_head = (conn.tx_queue_head + 1) % RemoteNodeConnection::TX_QUEUE_SIZE;
	conn.tx_queue_count--;
	return true;
}

bool UorbUartBridge::tx_queue_empty(const RemoteNodeConnection &conn) const
{
	return conn.tx_queue_count == 0;
}

bool UorbUartBridge::tx_queue_full(const RemoteNodeConnection &conn) const
{
	return conn.tx_queue_count >= RemoteNodeConnection::TX_QUEUE_SIZE;
}

extern "C" __EXPORT int uorb_uart_bridge_main(int argc, char *argv[])
{
	return UorbUartBridge::main(argc, argv);
}
