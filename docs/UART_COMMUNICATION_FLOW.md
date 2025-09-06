# Direct UART Communication Architecture

## How It Actually Works (Correct Approach)

The communication between X7+ and NXT boards uses **direct UART communication** with the `ProtocolFrame` structure, NOT uORB messages.

## Communication Flow

### 1. **X7+ Bridge → NXT Proxy (Command Flow)**

```cpp
// On X7+ Bridge: Send actuator commands to NXT
void UorbUartBridge::process_outgoing_messages()
{
    // Get data from local uORB
    actuator_outputs_s actuator_data;
    if (_actuator_outputs_sub.update(&actuator_data)) {

        // Build UART frame directly
        uint8_t uart_frame[1024];
        size_t frame_size = _frame_builder.build_data_frame(
            uart_frame, sizeof(uart_frame),
            TOPIC_ID_ACTUATOR_OUTPUTS, 0,  // topic_id, instance
            NodeId::X7_MAIN, NodeId::NXT_FRONT,  // source, dest
            &actuator_data, sizeof(actuator_data)  // payload
        );

        // Send directly via UART (not uORB!)
        write(uart_fd, uart_frame, frame_size);
    }
}
```

### 2. **NXT Proxy → Local uORB (Receive Commands)**

```cpp
// On NXT Proxy: Receive UART frames and publish to local uORB
void UorbUartProxy::process_incoming_messages()
{
    uint8_t uart_buffer[1024];
    ssize_t bytes = read(uart_fd, uart_buffer, sizeof(uart_buffer));

    if (bytes > 0) {
        // Parse UART frame directly
        const ProtocolFrame* frame = _frame_parser.parse_frame(uart_buffer, bytes);

        if (frame && frame->topic_id == TOPIC_ID_ACTUATOR_OUTPUTS) {
            // Extract payload and publish to local uORB
            const actuator_outputs_s* data =
                (const actuator_outputs_s*)_frame_parser.get_payload(frame);

            orb_publish(ORB_ID(actuator_outputs), _actuator_pub, data);
        }
    }
}
```

### 3. **NXT Proxy → X7+ Bridge (Status Flow)**

```cpp
// On NXT Proxy: Send sensor data to X7+
void UorbUartProxy::process_outgoing_messages()
{
    // Get sensor data from local uORB
    sensor_quad_encoder_s encoder_data;
    if (_encoder_sub.update(&encoder_data)) {

        // Build UART frame directly
        uint8_t uart_frame[1024];
        size_t frame_size = _frame_builder.build_data_frame(
            uart_frame, sizeof(uart_frame),
            TOPIC_ID_SENSOR_QUAD_ENCODER, 0,
            NodeId::NXT_FRONT, NodeId::X7_MAIN,
            &encoder_data, sizeof(encoder_data)
        );

        // Send via UART
        write(uart_fd, uart_frame, frame_size);
    }
}
```

### 4. **X7+ Bridge → Local uORB (Receive Status)**

```cpp
// On X7+ Bridge: Receive status from NXT and publish locally
void UorbUartBridge::handle_received_frame(const ProtocolFrame* frame)
{
    if (frame->topic_id == TOPIC_ID_SENSOR_QUAD_ENCODER) {
        // Extract sensor data from UART frame
        const sensor_quad_encoder_s* data =
            (const sensor_quad_encoder_s*)_frame_parser.get_payload(frame);

        // Publish to local uORB on X7+
        orb_publish(ORB_ID(sensor_quad_encoder), _encoder_pub, data);
    }
}
```

## Key Points

### ✅ **Correct Approach:**
- **UART Protocol**: Direct binary communication using `ProtocolFrame` structure
- **uORB Interface**: Standard uORB subscribe/publish on each board locally
- **Bridge Function**: Convert between local uORB ↔ UART frames
- **No Extra Messages**: No wrapper messages, direct topic data transmission

### ❌ **Wrong Approach (What we removed):**
- Creating `UorbProxyMessage` as a uORB message
- Using uORB to send UART data (circular dependency)
- Unnecessary message wrapping and unwrapping
- Added complexity with no benefit

## Physical Communication Path

```
X7+ Board:                           NXT Board:
┌─────────────────┐                  ┌─────────────────┐
│ wheel_loader    │                  │ actuator_module │
│ ↓ uORB publish  │                  │ ← uORB subscribe│
│ actuator_outputs│                  │ actuator_outputs│
│ ↓               │                  │ ↑               │
│ uorb_uart_bridge│ ═══UART═══════► │ uorb_uart_proxy │
│ ↓ UART frames   │  (ProtocolFrame) │ ↑ UART frames   │
│ /dev/ttyS1      │                  │ /dev/ttyS1      │
└─────────────────┘                  └─────────────────┘
```

## Benefits of Direct UART Approach

1. **Low Latency**: Direct binary transmission, no extra message layers
2. **Efficient**: Minimal protocol overhead, only necessary metadata
3. **Reliable**: CRC32 validation, sequence numbering, error handling
4. **Transparent**: Existing modules use standard uORB API unchanged
5. **Scalable**: Easy to add new topics without protocol changes

You were absolutely correct - the UorbProxyMessage approach was unnecessary complexity. The direct UART communication with ProtocolFrame is the right way to do distributed uORB messaging.
