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
// Servo Parameters
// ============================================================================

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Change detection thresholds
#define ANALOG_CHANGE_THRESHOLD 8 // Minimum ADC delta (~0.8% change) to trigger transmission

// Maximum interval between transmissions even if no state changes (heartbeat ms)
#define HEARTBEAT_INTERVAL_MS 5000

// Poll interval for input change detection (ms)
#define POLL_INTERVAL_MS 50

// ============================================================================
// Mesh Network Configuration
// ============================================================================

#define MESH_PREFIX     "ESP8266_ServoMesh"
#define MESH_PASSWORD   "MeshServoPassword123"
#define MESH_PORT       5555

// Message Magic Byte for verifying packed struct binary integrity
#define MESSAGE_MAGIC   0xB7

// ============================================================================
// Compact Binary Struct Definition
// ============================================================================

struct __attribute__((__packed__)) ServoMeshMessage {
    uint8_t  magic;            // Magic header byte (0xB7)
    uint16_t sender_id;        // Custom compile-time sender node ID
    uint16_t target_id;        // Custom compile-time target node ID
    uint16_t sensor_value;     // Analog sensor value (0-1023)
    uint8_t  digital_value;    // Digital input state (0 or 1)
    uint8_t  min_limit_active; // Min limit switch state (1 = triggered/active, 0 = open)
    uint8_t  max_limit_active; // Max limit switch state (1 = triggered/active, 0 = open)
    uint32_t seq;              // Message sequence counter
};

#endif // SERVO_CONFIG_H
