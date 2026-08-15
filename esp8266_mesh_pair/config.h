#ifndef CONFIG_H
#define CONFIG_H

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

// Analog sensor pin (ESP8266 single ADC pin A0)
#define SENSOR_PIN A0

// PWM Output Pin (e.g., GPIO 5 / D1 on NodeMCU/Wemos D1 Mini)
#define PWM_PIN D1

// Digital Input Pin (sending node)
#define DIGITAL_INPUT_PIN D2

// Digital Output Pin (receiving node)
#define DIGITAL_OUTPUT_PIN D3

// PWM resolution max value (0 to 1023 for ESP8266 default analogWrite range)
#define PWM_RANGE 1023

// ============================================================================
// Mesh Network Configuration
// ============================================================================

#define MESH_PREFIX     "ESP8266_SensorMesh"
#define MESH_PASSWORD   "MeshPairPassword123"
#define MESH_PORT       5555

// Transmission interval in milliseconds (1/sec = 1000 ms)
#define SEND_INTERVAL_MS 1000

// Message Magic Byte for verifying packed struct binary integrity
#define MESSAGE_MAGIC   0xA5

// ============================================================================
// Compact Binary Struct Definition
// ============================================================================

struct __attribute__((__packed__)) SensorMessage {
    uint8_t  magic;          // Magic header byte (0xA5)
    uint16_t sender_id;      // Custom compile-time sender node ID
    uint16_t target_id;      // Custom compile-time target node ID
    uint16_t sensor_value;   // Analog sensor value (0-1023)
    uint8_t  digital_value;  // Digital input state (0 or 1 from D2)
    uint32_t seq;            // Message sequence counter
};

#endif // CONFIG_H
