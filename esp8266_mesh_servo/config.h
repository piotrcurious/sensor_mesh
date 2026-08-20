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

// Alarm Reset Pin (Active LOW with internal pull-up, e.g. GPIO 14 / D5)
#define RESET_ALARM_PIN D5

// ============================================================================
// Alarm Latching Feature Option
// ============================================================================
// Uncomment the line below to enable sticky alarm output latching.
// When enabled, receiving a HIGH digital input state latches DIGITAL_OUTPUT_PIN
// to HIGH permanently (even if door/switch closes again) until RESET_ALARM_PIN is pulled LOW.
// Default: Disabled (standard non-latched mirroring).
// #define LATCH_DIGITAL_OUTPUT_HIGH

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
#define ADC_OVERSAMPLE_COUNT 8

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
// Compact Binary Struct Definitions (Includes Session ID & CRC16)
// ============================================================================

// Data Payload Struct (20 bytes packed)
struct __attribute__((__packed__)) ServoMeshMessage {
    uint8_t  magic;            // 1 byte  (MSG_TYPE_DATA = 0xBA)
    uint16_t sender_id;        // 2 bytes
    uint16_t target_id;        // 2 bytes
    uint32_t session_id;       // 4 bytes
    uint16_t target_us_fp4;    // 2 bytes
    uint8_t  digital_value;    // 1 byte
    uint8_t  min_limit_active; // 1 byte
    uint8_t  max_limit_active; // 1 byte
    uint32_t seq;              // 4 bytes
    uint16_t crc16;            // 2 bytes CRC16 checksum
};

// Handshake Discovery Struct (15 bytes packed)
struct __attribute__((__packed__)) HandshakeMessage {
    uint8_t  magic;            // 1 byte  (MSG_TYPE_HELLO / MSG_TYPE_HELLO_ACK)
    uint16_t sender_id;        // 2 bytes
    uint16_t target_id;        // 2 bytes
    uint32_t session_id;       // 4 bytes
    uint32_t seq;              // 4 bytes
    uint16_t crc16;            // 2 bytes CRC16 checksum
};

// Static assertions to ensure struct packing without undefined padding
static_assert(sizeof(ServoMeshMessage) == 20, "ServoMeshMessage struct size must be exactly 20 bytes");
static_assert(sizeof(HandshakeMessage) == 15, "HandshakeMessage struct size must be exactly 15 bytes");

#define SERVO_WIRE_HEX_LEN     (sizeof(ServoMeshMessage) * 2)  // 40 hex chars
#define HANDSHAKE_WIRE_HEX_LEN (sizeof(HandshakeMessage) * 2)  // 30 hex chars

// Peer Discovery State Machine
enum class PeerState : uint8_t {
    UNKNOWN,
    DISCOVERING,
    CONNECTED
};

#endif // SERVO_CONFIG_H
