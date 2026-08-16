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

// Change detection thresholds (in microseconds)
#define PULSE_CHANGE_THRESHOLD 3 // Minimum 3us delta to trigger event-driven transmission

// Maximum interval between transmissions even if no state changes (heartbeat ms)
#define HEARTBEAT_INTERVAL_MS 5000

// Polling interval (50 ms = 20 Hz sampling) for Wi-Fi stack stability
#define POLL_INTERVAL_MS 50

// Network rate limit: minimum time between mesh packet broadcasts (200 ms = 5 Hz max packet rate)
#define MIN_TX_INTERVAL_MS 200

// ============================================================================
// DSP & Precision Parameters (Oversampling, Kahan, Outlier, Kalman)
// ============================================================================

// Oversampling count per sample cycle (4 samples to protect ESP8266 Wi-Fi PHY time-slicing)
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

// Message Magic Byte for verifying packed struct binary integrity
#define MESSAGE_MAGIC   0xB8

// ============================================================================
// Compact Binary Struct Definition
// ============================================================================

struct __attribute__((__packed__)) ServoMeshMessage {
    uint8_t  magic;            // Magic header byte (0xB8)
    uint16_t sender_id;        // Custom compile-time sender node ID
    uint16_t target_id;        // Custom compile-time target node ID
    uint16_t target_us;        // Target servo pulse width in microseconds (544 - 2400 us)
    uint8_t  digital_value;    // Digital input state (0 or 1)
    uint8_t  min_limit_active; // Min limit switch state (1 = triggered/active, 0 = open)
    uint8_t  max_limit_active; // Max limit switch state (1 = triggered/active, 0 = open)
    uint32_t seq;              // Message sequence counter
};

#endif // SERVO_CONFIG_H
