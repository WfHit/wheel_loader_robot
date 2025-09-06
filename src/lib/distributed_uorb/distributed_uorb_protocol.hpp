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
 * @file distributed_uorb_protocol.hpp
 * @author PX4 Development Team
 *
 * Distributed uORB protocol definitions and utilities
 * Shared between uorb_uart_bridge and uorb_uart_proxy modules
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <cstdint>
#include <cstddef>

namespace distributed_uorb
{

// Protocol version
static constexpr uint8_t PROTOCOL_VERSION = 2;

// Frame sync bytes for packet detection
static constexpr uint8_t SYNC_BYTE1 = 0xFF;
static constexpr uint8_t SYNC_BYTE2 = 0xFE;

// Node identifiers
enum class NodeId : uint8_t {
	BROADCAST = 0,
	X7_MAIN = 1,
	NXT_FRONT = 2,
	NXT_REAR = 3,
	RESERVED = 255
};

// Message types
enum class MessageType : uint8_t {
	DATA = 0x01,         // Topic data message
	SUBSCRIBE = 0x02,    // Subscribe request
	ADVERTISE = 0x03,    // Advertise topic
	HEARTBEAT = 0x04,    // Keep-alive message
	TIME_SYNC = 0x05,    // Time synchronization
	TOPIC_INFO = 0x06,   // Topic information exchange
	ERROR = 0x07,        // Error notification
	ACK = 0x08,          // Acknowledgment
	DISCOVERY = 0x09     // Node/topic discovery
};

// Priority levels
enum class Priority : uint8_t {
	LOW = 0,
	NORMAL = 1,
	HIGH = 2,
	CRITICAL = 3
};

// Reliability levels
enum class Reliability : uint8_t {
	BEST_EFFORT = 0,     // No delivery guarantee
	RELIABLE = 1,        // At least once delivery
	GUARANTEED = 2       // Exactly once delivery
};

// Compression types
enum class Compression : uint8_t {
	NONE = 0,
	ZLIB = 1,
	LZ4 = 2
};

// Error flags
enum class ErrorFlags : uint8_t {
	NONE = 0x00,
	CHECKSUM_FAILED = 0x01,
	TIMEOUT = 0x02,
	BUFFER_OVERFLOW = 0x04,
	PROTOCOL_ERROR = 0x08,
	NODE_OFFLINE = 0x10,
	TOPIC_NOT_FOUND = 0x20,
	PERMISSION_DENIED = 0x40,
	UNKNOWN_ERROR = 0x80
};

// Maximum payload size (optimized for common uORB messages)
static constexpr size_t MAX_PAYLOAD_SIZE = 512;

// Timing constants
static constexpr uint32_t DEFAULT_HEARTBEAT_INTERVAL_MS = 1000;
static constexpr uint32_t DEFAULT_CONNECTION_TIMEOUT_MS = 3000;
static constexpr uint32_t DEFAULT_ACK_TIMEOUT_MS = 100;
static constexpr uint32_t DEFAULT_RETRY_INTERVAL_MS = 50;
static constexpr uint8_t DEFAULT_MAX_RETRIES = 3;

/**
 * Wire protocol frame structure
 * Total overhead: 20 bytes + payload
 */
struct __attribute__((packed)) ProtocolFrame {
	// Frame header (6 bytes)
	uint8_t sync1;              // Sync byte 1 (0xFF)
	uint8_t sync2;              // Sync byte 2 (0xFE)
	uint8_t protocol_version;   // Protocol version
	uint8_t message_type;       // MessageType enum
	uint16_t frame_length;      // Total frame length including header

	// Message metadata (8 bytes)
	uint16_t topic_id;          // uORB topic identifier
	uint8_t instance;           // Multi-instance topic instance
	uint8_t source_node;        // Source NodeId
	uint8_t dest_node;          // Destination NodeId
	uint16_t sequence_num;      // Sequence number
	uint8_t flags;              // Combination of priority, reliability, compression

	// Quality of service (4 bytes)
	uint16_t payload_length;    // Actual payload data length
	uint16_t ttl_ms;            // Time-to-live in milliseconds

	// Payload data (variable length, max MAX_PAYLOAD_SIZE)
	uint8_t payload[0];

	// Footer (CRC32) follows payload data
};

/**
 * Message flags bit layout:
 * Bits 0-1: Priority (Priority enum)
 * Bits 2-3: Reliability (Reliability enum)
 * Bits 4-5: Compression (Compression enum)
 * Bits 6-7: Reserved
 */
inline uint8_t encode_flags(Priority priority, Reliability reliability, Compression compression)
{
	return (static_cast<uint8_t>(priority) & 0x03) |
	       ((static_cast<uint8_t>(reliability) & 0x03) << 2) |
	       ((static_cast<uint8_t>(compression) & 0x03) << 4);
}

inline void decode_flags(uint8_t flags, Priority &priority, Reliability &reliability, Compression &compression)
{
	priority = static_cast<Priority>(flags & 0x03);
	reliability = static_cast<Reliability>((flags >> 2) & 0x03);
	compression = static_cast<Compression>((flags >> 4) & 0x03);
}

/**
 * Heartbeat message payload
 */
struct __attribute__((packed)) HeartbeatPayload {
	uint64_t timestamp;         // Current timestamp
	uint32_t uptime_ms;         // Node uptime in milliseconds
	uint8_t system_health;      // System health status (0-100%)
	uint8_t cpu_load;           // CPU load percentage
	uint16_t free_memory_kb;    // Available memory in KB
	uint32_t tx_packets;        // Total transmitted packets
	uint32_t rx_packets;        // Total received packets
	uint32_t tx_errors;         // Transmission error count
	uint32_t rx_errors;         // Reception error count
};

/**
 * Topic information message payload
 */
struct __attribute__((packed)) TopicInfoPayload {
	uint16_t topic_id;          // Topic identifier
	uint8_t instance;           // Instance number
	char topic_name[32];        // Topic name string
	uint16_t message_size;      // Message structure size
	uint8_t publisher_count;    // Number of publishers
	uint8_t subscriber_count;   // Number of subscribers
	uint32_t publish_rate_hz;   // Publishing rate in Hz
	uint64_t last_published;    // Last publish timestamp
};

/**
 * Error notification payload
 */
struct __attribute__((packed)) ErrorPayload {
	uint8_t error_code;         // ErrorFlags enum
	uint16_t failed_sequence;   // Sequence number that failed
	uint16_t topic_id;          // Related topic ID (if applicable)
	char error_message[64];     // Human-readable error description
};

/**
 * CRC32 calculation for data integrity
 */
uint32_t calculate_crc32(const void *data, size_t length);

/**
 * Frame validation
 */
bool validate_frame(const ProtocolFrame *frame, size_t total_length);

/**
 * Frame builder helper class
 */
class FrameBuilder
{
public:
	FrameBuilder() = default;
	~FrameBuilder() = default;

	/**
	 * Build a data frame
	 */
	size_t build_data_frame(uint8_t *buffer, size_t buffer_size,
				uint16_t topic_id, uint8_t instance,
				NodeId source, NodeId dest,
				const void *payload, size_t payload_size,
				Priority priority = Priority::NORMAL,
				Reliability reliability = Reliability::BEST_EFFORT,
				uint16_t sequence = 0, uint16_t ttl_ms = 1000);

	/**
	 * Build a heartbeat frame
	 */
	size_t build_heartbeat_frame(uint8_t *buffer, size_t buffer_size,
				     NodeId source, const HeartbeatPayload &payload,
				     uint16_t sequence = 0);

	/**
	 * Build an error frame
	 */
	size_t build_error_frame(uint8_t *buffer, size_t buffer_size,
				 NodeId source, NodeId dest,
				 const ErrorPayload &payload,
				 uint16_t sequence = 0);

private:
	uint16_t _next_sequence{1};
};

/**
 * Frame parser helper class
 */
class FrameParser
{
public:
	FrameParser() = default;
	~FrameParser() = default;

	/**
	 * Parse incoming frame from buffer
	 * Returns nullptr if frame is invalid
	 */
	const ProtocolFrame *parse_frame(const uint8_t *buffer, size_t buffer_size);

	/**
	 * Extract payload data from frame
	 */
	const void *get_payload(const ProtocolFrame *frame) const;

	/**
	 * Get payload size
	 */
	size_t get_payload_size(const ProtocolFrame *frame) const;

	/**
	 * Validate frame CRC
	 */
	bool validate_crc(const ProtocolFrame *frame, size_t total_length) const;
};

} // namespace distributed_uorb
