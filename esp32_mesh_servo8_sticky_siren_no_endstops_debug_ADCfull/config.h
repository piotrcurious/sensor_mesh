#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>

// ============================================================================
// Node Pairing & Identity Settings
// ============================================================================

#ifndef MY_NODE_ID
#define MY_NODE_ID 3
#endif

#ifndef TARGET_NODE_ID
#define TARGET_NODE_ID 4
#endif

#define DEBUG_CONNECTION

// ============================================================================
// ESP32 Pin Definitions
// ============================================================================
//
// Recommended for classic ESP32-WROOM:
//
// GPIO34 = ADC1 input
// GPIO18 = Servo
// GPIO19 = Digital input
// GPIO23 = Digital output
// GPIO21 = Alarm reset
// GPIO22 = Siren
//
// GPIO34 is input-only, which is ideal for the ADC.
//
// IMPORTANT:
// Do not move SENSOR_PIN to ADC2 when Wi-Fi is active.
// On classic ESP32, ADC2 is shared with Wi-Fi.
//

#define SENSOR_PIN          34
#define ADC_RESOLUTION      4095.0f // ESP32 uses 12bit
#define SERVO_PIN           18
#define DIGITAL_INPUT_PIN   19
#define DIGITAL_OUTPUT_PIN  23
#define RESET_ALARM_PIN     21

#define NO_ENDSTOPS

#ifndef NO_ENDSTOPS
#define MIN_LIMIT_PIN       25
#define MAX_LIMIT_PIN       26
#endif

// ============================================================================
// Alarm Feature Options
// ============================================================================

//#define LATCH_DIGITAL_OUTPUT_HIGH

#define ENABLE_SIREN_OUTPUT

#ifdef ENABLE_SIREN_OUTPUT

#define SIREN_PIN       22

#define SIREN_FREQ_LOW   600
#define SIREN_FREQ_HIGH  1200
#define SIREN_STEP_HZ    80
#define SIREN_TICK_MS    50

#endif

// ============================================================================
// Servo Parameters
// ============================================================================

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

#define SERVO_MIN_PULSE_WIDTH 544
#define SERVO_MAX_PULSE_WIDTH 2400

#define PULSE_FP4_CHANGE_THRESHOLD 32

#define HEARTBEAT_INTERVAL_MS 5000
#define POLL_INTERVAL_MS      50
#define MIN_TX_INTERVAL_MS    200
#define DISCOVERY_INTERVAL_MS 1000

// ============================================================================
// ADC / DSP
// ============================================================================

#define ADC_OVERSAMPLE_COUNT 16

#define KALMAN_PROCESS_NOISE_Q      0.5f
#define KALMAN_MEASUREMENT_NOISE_R  2.0f

// ============================================================================
// Mesh
// ============================================================================

#define MESH_PREFIX     "ESP8266_ServoMesh"
#define MESH_PASSWORD   "MeshServoPassword123"
#define MESH_PORT       5555

#define MSG_TYPE_DATA       0xBA
#define MSG_TYPE_HELLO      0xE3
#define MSG_TYPE_HELLO_ACK  0xE4

// ============================================================================
// Compact Binary Protocol
// ============================================================================

struct __attribute__((packed)) ServoMeshMessage {
    uint8_t  magic;
    uint16_t sender_id;
    uint16_t target_id;
    uint32_t session_id;
    uint16_t target_us_fp4;
    uint8_t  digital_value;
    uint8_t  min_limit_active;
    uint8_t  max_limit_active;
    uint32_t seq;
    uint16_t crc16;
};

struct __attribute__((packed)) HandshakeMessage {
    uint8_t  magic;
    uint16_t sender_id;
    uint16_t target_id;
    uint32_t session_id;
    uint32_t seq;
    uint16_t crc16;
};

static_assert(
    sizeof(ServoMeshMessage) == 20,
    "ServoMeshMessage struct size must be exactly 20 bytes"
);

static_assert(
    sizeof(HandshakeMessage) == 15,
    "HandshakeMessage struct size must be exactly 15 bytes"
);

#define SERVO_WIRE_HEX_LEN     (sizeof(ServoMeshMessage) * 2)
#define HANDSHAKE_WIRE_HEX_LEN (sizeof(HandshakeMessage) * 2)

// ============================================================================
// Peer State
// ============================================================================

enum class PeerState : uint8_t {
    UNKNOWN,
    DISCOVERING,
    CONNECTED
};

#endif
