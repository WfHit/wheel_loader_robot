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
 * @file wk2132_test.cpp
 * @author PX4 Development Team
 *
 * WK2132 UART test command for wheel loader serial communication testing.
 * This utility provides comprehensive testing capabilities for the WK2132
 * I2C-to-UART bridge used in wheel loader applications.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <poll.h>
#include <sys/ioctl.h>

static const char *DEFAULT_DEVICE = "/dev/ttyS10";  // Default WK2132 serial device
static const int DEFAULT_BAUDRATE = 115200;
static const int DEFAULT_TEST_DURATION = 10;  // seconds

struct test_config {
	const char *device;
	int baudrate;
	int duration;
	bool loopback;
	bool verbose;
	bool continuous;
	int pattern;
	int data_bits;
	int stop_bits;
	bool parity;
	bool echo_test;
};

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
WK2132 UART test utility

This command provides comprehensive testing for the WK2132 I2C-to-UART bridge
used in wheel loader serial communication systems.

Test modes:
- Basic connectivity test
- Loopback test (requires hardware loopback)
- Echo test (requires external echo device)
- Pattern transmission test
- Continuous transmission test
- Send test (send message every second at 115200 baud)
- Receive test (listen and output received data for 30 seconds)
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wk2132_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("basic", "Run basic connectivity test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("loopback", "Run loopback test (requires TX-RX jumper)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("echo", "Run echo test (requires external echo device)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pattern", "Send test pattern");
	PRINT_MODULE_USAGE_COMMAND_DESCR("continuous", "Continuous transmission test");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send", "Send test: send message every second at 115200 baud");
	PRINT_MODULE_USAGE_COMMAND_DESCR("receive", "Receive test: listen and output received data for 30 seconds");

	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS4", "<device>", "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 9600, 1200, 921600, "Baud rate", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 10, 1, 3600, "Test duration (seconds)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 3, "Test pattern (0=incremental, 1=0x55, 2=0xAA, 3=random)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "Verbose output", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('c', "Continuous mode (until ESC)", true);

	PRINT_MODULE_USAGE_PARAM_COMMENT("Data format options:");
	PRINT_MODULE_USAGE_PARAM_INT('D', 8, 5, 8, "Data bits (5-8)", true);
	PRINT_MODULE_USAGE_PARAM_INT('S', 1, 1, 2, "Stop bits (1-2)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('P', "Enable even parity", true);
}

static int setup_serial_port(int fd, const struct test_config *config)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		PX4_ERR("Error getting terminal attributes: %s", strerror(errno));
		return -1;
	}

	// Set baud rate
	speed_t speed;
	switch (config->baudrate) {
	case 1200:   speed = B1200; break;
	case 2400:   speed = B2400; break;
	case 4800:   speed = B4800; break;
	case 9600:   speed = B9600; break;
	case 19200:  speed = B19200; break;
	case 38400:  speed = B38400; break;
	case 57600:  speed = B57600; break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	case 460800: speed = B460800; break;
	case 921600: speed = B921600; break;
	default:
		PX4_ERR("Unsupported baud rate: %d", config->baudrate);
		return -1;
	}

	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);

	// Configure data format
	tty.c_cflag &= ~CSIZE;
	switch (config->data_bits) {
	case 5: tty.c_cflag |= CS5; break;
	case 6: tty.c_cflag |= CS6; break;
	case 7: tty.c_cflag |= CS7; break;
	case 8: tty.c_cflag |= CS8; break;
	default:
		PX4_ERR("Invalid data bits: %d", config->data_bits);
		return -1;
	}

	// Configure stop bits
	if (config->stop_bits == 2) {
		tty.c_cflag |= CSTOPB;
	} else {
		tty.c_cflag &= ~CSTOPB;
	}

	// Configure parity
	if (config->parity) {
		tty.c_cflag |= PARENB;  // Enable parity
		tty.c_cflag &= ~PARODD; // Even parity
	} else {
		tty.c_cflag &= ~PARENB; // No parity
	}

	// Raw mode configuration
	tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls, enable reading
	tty.c_cflag &= ~CRTSCTS;            // No hardware flow control
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL); // No input processing
	tty.c_oflag &= ~OPOST;              // No output processing

	// Set timeouts
	tty.c_cc[VMIN] = 0;   // Non-blocking read
	tty.c_cc[VTIME] = 5;  // 0.5 second timeout

	if (tcsetattr(fd, TCSANOW, &tty) < 0) {
		PX4_ERR("Error setting terminal attributes: %s", strerror(errno));
		return -1;
	}

	// Flush any existing data
	tcflush(fd, TCIOFLUSH);

	return 0;
}

static void generate_test_data(uint8_t *buffer, size_t size, int pattern, int *counter)
{
	for (size_t i = 0; i < size; i++) {
		switch (pattern) {
		case 0: // Incremental pattern
			buffer[i] = (uint8_t)((*counter)++ & 0xFF);
			break;
		case 1: // 0x55 pattern
			buffer[i] = 0x55;
			break;
		case 2: // 0xAA pattern
			buffer[i] = 0xAA;
			break;
		case 3: // Random pattern
			buffer[i] = (uint8_t)(rand() & 0xFF);
			break;
		default:
			buffer[i] = 0x00;
			break;
		}
	}
}

static int test_basic_connectivity(const struct test_config *config)
{
	PX4_INFO("Starting basic connectivity test on %s at %d baud", config->device, config->baudrate);

	int fd = open(config->device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", config->device, strerror(errno));
		return -1;
	}

	if (setup_serial_port(fd, config) < 0) {
		close(fd);
		return -1;
	}

	// Test basic write capability
	const char *test_msg = "WK2132 Test Message\r\n";
	size_t msg_len = strlen(test_msg);
	ssize_t written = write(fd, test_msg, msg_len);

	if (written < 0) {
		PX4_ERR("Write failed: %s", strerror(errno));
		close(fd);
		return -1;
	} else if (written != (ssize_t)msg_len) {
		PX4_WARN("Partial write: wrote %zd of %zu bytes", written, msg_len);
	} else {
		PX4_INFO("Successfully wrote %zd bytes: 'WK2132 Test Message'", written);
	}

	// Test read capability (with timeout)
	char read_buffer[64];
	ssize_t bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);

	if (bytes_read > 0) {
		read_buffer[bytes_read] = '\0';
		PX4_INFO("Read %zd bytes: %s", bytes_read, read_buffer);
	} else if (bytes_read == 0) {
		PX4_INFO("No data received (timeout)");
	} else {
		PX4_WARN("Read failed: %s", strerror(errno));
	}

	close(fd);
	PX4_INFO("Basic connectivity test completed");
	return 0;
}

static int test_loopback(const struct test_config *config)
{
	PX4_INFO("Starting loopback test on %s (ensure TX-RX are connected)", config->device);

	int fd = open(config->device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", config->device, strerror(errno));
		return -1;
	}

	if (setup_serial_port(fd, config) < 0) {
		close(fd);
		return -1;
	}

	uint8_t test_data[256];
	uint8_t read_data[256];
	int counter = 0;
	int errors = 0;
	int tests = 0;

	PX4_INFO("Running loopback test for %d seconds...", config->duration);

	time_t start_time = time(NULL);
	time_t end_time = start_time + config->duration;

	while (time(NULL) < end_time) {
		// Generate test data
		size_t test_size = 1 + (rand() % 32); // Random size 1-32 bytes
		generate_test_data(test_data, test_size, config->pattern, &counter);

		// Write data
		ssize_t written = write(fd, test_data, test_size);
		if (written != (ssize_t)test_size) {
			PX4_ERR("Write error: expected %zu, got %zd", test_size, written);
			errors++;
			continue;
		}

		// Read back data
		usleep(10000); // 10ms delay for data to loop back
		ssize_t bytes_read = read(fd, read_data, test_size);

		if (bytes_read != (ssize_t)test_size) {
			PX4_ERR("Read size mismatch: expected %zu, got %zd", test_size, bytes_read);
			errors++;
			continue;
		}

		// Compare data
		if (memcmp(test_data, read_data, test_size) != 0) {
			PX4_ERR("Data mismatch on test %d", tests);
			errors++;
			if (config->verbose) {
				printf("Expected: ");
				for (size_t i = 0; i < test_size; i++) {
					printf("%02X ", test_data[i]);
				}
				printf("\nReceived: ");
				for (size_t i = 0; i < test_size; i++) {
					printf("%02X ", read_data[i]);
				}
				printf("\n");
			}
		}

		tests++;
		if (config->verbose && (tests % 100 == 0)) {
			PX4_INFO("Completed %d tests, %d errors", tests, errors);
		}
	}

	close(fd);

	PX4_INFO("Loopback test completed: %d tests, %d errors (%.2f%% success rate)",
		 tests, errors, tests > 0 ? (100.0 * (tests - errors) / tests) : 0.0);

	return errors == 0 ? 0 : -1;
}

static int test_echo(const struct test_config *config)
{
	PX4_INFO("Starting echo test on %s (requires external echo device)", config->device);

	int fd = open(config->device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", config->device, strerror(errno));
		return -1;
	}

	if (setup_serial_port(fd, config) < 0) {
		close(fd);
		return -1;
	}

	const char *test_messages[] = {
		"Hello WK2132\r\n",
		"Echo Test 123\r\n",
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n",
		"0123456789\r\n"
	};

	int num_messages = sizeof(test_messages) / sizeof(test_messages[0]);
	char read_buffer[256];

	PX4_INFO("Sending %d test messages...", num_messages);

	for (int i = 0; i < num_messages; i++) {
		const char *msg = test_messages[i];
		size_t msg_len = strlen(msg);

		// Send message
		ssize_t written = write(fd, msg, msg_len);
		if (written != (ssize_t)msg_len) {
			PX4_ERR("Write failed for message %d", i);
			continue;
		}

		// Wait for echo
		usleep(100000); // 100ms delay

		// Read echo
		ssize_t bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);
		if (bytes_read > 0) {
			read_buffer[bytes_read] = '\0';
			PX4_INFO("Sent: %s", msg);
			PX4_INFO("Echo: %s", read_buffer);
		} else {
			PX4_WARN("No echo received for message %d", i);
		}
	}

	close(fd);
	PX4_INFO("Echo test completed");
	return 0;
}

static int test_pattern(const struct test_config *config)
{
	PX4_INFO("Starting pattern test on %s", config->device);

	int fd = open(config->device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", config->device, strerror(errno));
		return -1;
	}

	if (setup_serial_port(fd, config) < 0) {
		close(fd);
		return -1;
	}

	uint8_t pattern_data[64];
	int counter = 0;
	int bytes_sent = 0;

	PX4_INFO("Sending pattern %d for %d seconds...", config->pattern, config->duration);

	time_t start_time = time(NULL);
	time_t end_time = start_time + config->duration;

	while (time(NULL) < end_time) {
		generate_test_data(pattern_data, sizeof(pattern_data), config->pattern, &counter);

		ssize_t written = write(fd, pattern_data, sizeof(pattern_data));
		if (written > 0) {
			bytes_sent += written;
		} else {
			PX4_ERR("Write failed: %s", strerror(errno));
			break;
		}

		usleep(10000); // 10ms delay between transmissions
	}

	close(fd);
	PX4_INFO("Pattern test completed: %d bytes sent", bytes_sent);
	return 0;
}

static int test_send(const struct test_config *config)
{
	PX4_INFO("Starting send test on %s at 115200 baud", config->device);

	int fd = open(config->device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", config->device, strerror(errno));
		return -1;
	}

	// Force baudrate to 115200 for send test
	struct test_config send_config = *config;
	send_config.baudrate = 115200;

	if (setup_serial_port(fd, &send_config) < 0) {
		close(fd);
		return -1;
	}

	int message_count = 0;

	PX4_INFO("Sending messages every second for %d seconds...", config->duration);
	PX4_INFO("Press Ctrl+C to stop");

	time_t start_time = time(NULL);
	time_t end_time = start_time + config->duration;
	time_t last_send = 0;

	while (time(NULL) < end_time) {
		time_t current_time = time(NULL);

		// Send message every second
		if (current_time != last_send) {
			char send_buffer[64];
			snprintf(send_buffer, sizeof(send_buffer), "WK2132 Test #%d - Time: %ld\r\n",
				++message_count, current_time);

			size_t msg_len = strlen(send_buffer);
			ssize_t written = write(fd, send_buffer, msg_len);

			if (written == (ssize_t)msg_len) {
				PX4_INFO("Sent: %zu bytes", msg_len);
			} else if (written > 0) {
				PX4_WARN("Partial write: %zd of %zu bytes sent", written, msg_len);
			} else {
				PX4_ERR("Write failed: %s", strerror(errno));
			}

			last_send = current_time;

		}
		usleep(100000); // 100ms sleep to avoid busy loop
	}

	close(fd);
	PX4_INFO("Send test completed: sent %d messages", message_count);
	return 0;
}

static int test_receive(const struct test_config *config)
{
	PX4_INFO("Starting receive test on %s at %d baud", config->device, config->baudrate);
	PX4_INFO("Listening for incoming data for 30 seconds...");

	int fd = open(config->device, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", config->device, strerror(errno));
		return -1;
	}

	if (setup_serial_port(fd, config) < 0) {
		close(fd);
		return -1;
	}

	char read_buffer[256];
	int total_bytes_received = 0;
	int message_count = 0;

	time_t start_time = time(NULL);
	time_t end_time = start_time + 30; // Fixed 30 seconds duration

	PX4_INFO("Starting to listen... Press Ctrl+C to stop early");

	while (time(NULL) < end_time) {
		// Use poll to check for data availability
		struct pollfd pfd;
		pfd.fd = fd;
		pfd.events = POLLIN;
		pfd.revents = 0;

		int poll_result = poll(&pfd, 1, 100); // 100ms timeout

		if (poll_result > 0 && (pfd.revents & POLLIN)) {
			// Data is available to read
			ssize_t bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);

			if (bytes_read > 0) {
				read_buffer[bytes_read] = '\0';
				total_bytes_received += bytes_read;
				message_count++;

				// Get current timestamp
				time_t current_time = time(NULL);
				time_t elapsed = current_time - start_time;

				PX4_INFO("[%02ld:%02ld] Msg #%d (%zd bytes): %s",
					elapsed / 60, elapsed % 60, message_count, bytes_read, read_buffer);

				if (config->verbose) {
					// Also print hex dump
					printf("Hex: ");
					for (ssize_t i = 0; i < bytes_read; i++) {
						printf("%02X ", (unsigned char)read_buffer[i]);
					}
					printf("\n");
				}
			} else if (bytes_read < 0) {
				PX4_ERR("Read error: %s", strerror(errno));
				break;
			}
		} else if (poll_result < 0) {
			PX4_ERR("Poll error: %s", strerror(errno));
			break;
		}
		// If poll_result == 0, it's just a timeout, continue loop
	}

	close(fd);

	time_t total_time = time(NULL) - start_time;
	PX4_INFO("Receive test completed:");
	PX4_INFO("  Duration: %ld seconds", total_time);
	PX4_INFO("  Messages received: %d", message_count);
	PX4_INFO("  Total bytes received: %d", total_bytes_received);
	PX4_INFO("  Average rate: %.2f bytes/sec", total_time > 0 ? (double)total_bytes_received / total_time : 0.0);

	return 0;
}

extern "C" __EXPORT int wk2132_test_main(int argc, char *argv[])
{
	struct test_config config = {
		.device = DEFAULT_DEVICE,
		.baudrate = DEFAULT_BAUDRATE,
		.duration = DEFAULT_TEST_DURATION,
		.loopback = false,
		.verbose = false,
		.continuous = false,
		.pattern = 0,
		.data_bits = 8,
		.stop_bits = 1,
		.parity = false,
		.echo_test = false
	};

	if (argc < 2) {
		usage();
		return 1;
	}

	// Parse command
	const char *command = argv[1];

	// Parse options starting from argv[2]
	int ch;
	int myoptind = 2;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:t:p:vcD:S:P", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			config.device = myoptarg;
			break;
		case 'b':
			config.baudrate = atoi(myoptarg);
			break;
		case 't':
			config.duration = atoi(myoptarg);
			break;
		case 'p':
			config.pattern = atoi(myoptarg);
			break;
		case 'v':
			config.verbose = true;
			break;
		case 'c':
			config.continuous = true;
			break;
		case 'D':
			config.data_bits = atoi(myoptarg);
			break;
		case 'S':
			config.stop_bits = atoi(myoptarg);
			break;
		case 'P':
			config.parity = true;
			break;
		default:
			usage();
			return 1;
		}
	}

	// Execute the requested test
	if (strcmp(command, "basic") == 0) {
		return test_basic_connectivity(&config);
	} else if (strcmp(command, "loopback") == 0) {
		return test_loopback(&config);
	} else if (strcmp(command, "echo") == 0) {
		return test_echo(&config);
	} else if (strcmp(command, "pattern") == 0) {
		return test_pattern(&config);
	} else if (strcmp(command, "continuous") == 0) {
		config.continuous = true;
		config.duration = 3600; // 1 hour max
		return test_pattern(&config);
	} else if (strcmp(command, "send") == 0) {
		return test_send(&config);
	} else if (strcmp(command, "receive") == 0) {
		return test_receive(&config);
	} else {
		PX4_ERR("Unknown command: %s", command);
		usage();
		return 1;
	}
}
