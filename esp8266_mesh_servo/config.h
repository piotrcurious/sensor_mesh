#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>

// ============================================================================
// Node Pairing & Identity Settings (Compile-time configuration)
// ============================================================================

// Unique ID for this node (e.g., Node 1)
#ifndef MY_NODE_ID
#define MY_NODE_ID 1
#endif

// Target Node ID to pair with (Node ID to receive our sensor data)
#ifndef TARGET_NODE_ID
#define TARGET_NODE_ID 2
#endif

// ============================================================================
// Pin Definitions
// ============================================================================

// Analog sensor pin (ESP8266 ADC A0) for position/analog input
#define SENSOR_PIN A0

// Servo Control Output Pin (e.g. GPIO 5 / D1)
#define SERVO_PIN D1

// Digital Input Pin (e.g. GPIO 4 / D2)
#define DIGITAL_INPUT_PIN D2

// Digital Output Pin (e.g. GPIO 0 / D3)
#define DIGITAL_OUTPUT_PIN D3

// Min Limit Switch Pin (Active LOW with internal pull-up, e.g. GPIO 12 / D6)
#define MIN_LIMIT_PIN D6

// Max Limit Switch Pin (Active LOW with internal pull-up, e.g. GPIO 13 / D7)
#define MAX_LIMIT_PIN D7

// ============================================================================
// Servo Parameters & Microsecond Pulse Calibration
// ============================================================================

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Micro-servo pulse durations in microseconds for full 0 to 180 degree range
#define SERVO_MIN_PULSE_WIDTH 544
#define SERVO_MAX_PULSE_WIDTH 2400

// Change detection threshold in 1/16th microseconds (FP4)
#define PULSE_FP4_CHANGE_THRESHOLD 8

// Maximum interval between transmissions even if no state changes (heartbeat ms)
#define HEARTBEAT_INTERVAL_MS 5000

// Polling interval (50 ms = 20 Hz sampling) for Wi-Fi stack stability
#define POLL_INTERVAL_MS 50

// Network rate limit: minimum time between mesh packet broadcasts (200 ms = 5 Hz max packet rate)
#define MIN_TX_INTERVAL_MS 200

// Peer Discovery Retry Interval (ms) during DISCOVERING state
#define DISCOVERY_INTERVAL_MS 1000

// ============================================================================
// DSP & Precision Parameters (Oversampling, Kahan, Outlier, Kalman)
// ============================================================================

// Oversampling count per sample cycle
#define ADC_OVERSAMPLE_COUNT 4

// Kalman Filter Tuning Parameters
#define KALMAN_PROCESS_NOISE_Q 0.05f
#define KALMAN_MEASUREMENT_NOISE_R 4.0f

// ============================================================================
// Mesh Network Configuration
// ============================================================================

#define MESH_PREFIX     "ESP8266_ServoMesh"
#define MESH_PASSWORD   "MeshServoPassword123"
#define MESH_PORT       5555

// Message Type Magic Bytes for Protocol
#define MSG_TYPE_DATA       0xBA
#define MSG_TYPE_HELLO      0xE3
#define MSG_TYPE_HELLO_ACK  0xE4

// ============================================================================
// Compact Binary Struct Definitions (Includes Session Incarnation ID)
// ============================================================================

// Data Payload Struct
struct __attribute__((__packed__)) ServoMeshMessage {
    uint8_t  magic;            // Magic header byte (MSG_TYPE_DATA = 0xBA)
    uint16_t sender_id;        // Custom compile-time sender node ID
    uint16_t target_id;        // Custom compile-time target node ID
    uint32_t session_id;       // Sender reboot/boot session incarnation token
    uint16_t target_us_fp4;    // Target pulse width in fixed-point 1/16th microseconds (us * 16)
    uint8_t  digital_value;    // Digital input state (0 or 1)
    uint8_t  min_limit_active; // Min limit switch state (1 = triggered/active, 0 = open)
    uint8_t  max_limit_active; // Max limit switch state (1 = triggered/active, 0 = open)
    uint32_t seq;              // Message sequence counter
};

// Handshake Discovery Struct (HELLO / HELLO_ACK)
struct __attribute__((__packed__)) HandshakeMessage {
    uint8_t  magic;            // Magic header byte (MSG_TYPE_HELLO / MSG_TYPE_HELLO_ACK)
    uint16_t sender_id;        // Custom compile-time sender node ID
    uint16_t target_id;        // Custom compile-time target node ID
    uint32_t session_id;       // Sender reboot/boot session incarnation token
    uint32_t seq;              // Sequence counter / timestamp
};

// Peer Discovery State Machine
enum class PeerState : uint8_t {
    UNKNOWN,
    DISCOVERING,
    CONNECTED
};

#endif // SERVO_CONFIG_H
