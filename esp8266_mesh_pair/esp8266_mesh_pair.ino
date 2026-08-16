/*
  ESP8266 Bi-directional Sensor Mesh Node Firmware (Unicast & Sequence Checking)
  Uses painlessMesh to create an auto-organizing mesh network.

  Fixes & Enhancements:
  - Sequence Tracking & Out-of-Order Rejection:
    * Implements wraparound-safe sequence comparison (isNewerSequence).
    * Rejects stale, duplicate, or reordered packets.
  - Unicast Mesh Transport & Dynamic Target Discovery:
    * Dynamically maps logical TARGET_NODE_ID to painlessMesh uint32_t transport node ID.
    * Uses targeted mesh.sendSingle(targetMeshNodeId, payload) for direct unicast transport when connected,
      falling back to mesh.sendBroadcast(payload) during initial discovery.
  - High-Precision DSP filtering (Kahan summation, outlier rejection, 1D Kalman filter).
  - Drives PWM pin (D1) and digital output pin (D3).
*/

#include <painlessMesh.h>
#include "config.h"

// TaskScheduler and painlessMesh instance
Scheduler userScheduler;
painlessMesh mesh;

// Function declarations
void sendSensorData();
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
float readAnalogFiltered();
bool isNewerSequence(uint32_t incoming, uint32_t last);

// Task to read sensor and transmit data every 1 second (SEND_INTERVAL_MS)
Task taskSendSensorData(SEND_INTERVAL_MS, TASK_FOREVER, &sendSensorData);

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// Sequence tracking
static uint32_t messageSequence = 0;
static uint32_t lastReceivedSeq = 0;
static bool     hasReceivedFirstPacket = false;
static uint32_t targetMeshNodeId = 0; // Discovered painlessMesh uint32_t node ID for TARGET_NODE_ID

// Helper: Convert uint8_t hex character ('0'-'9', 'A'-'F', 'a'-'f') to byte value
static uint8_t hexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

/**
 * Wraparound-safe 32-bit sequence comparison.
 * Returns true if 'incoming' is strictly newer than 'last'.
 */
bool isNewerSequence(uint32_t incoming, uint32_t last) {
    if (!hasReceivedFirstPacket) {
        return true;
    }
    return ((int32_t)(incoming - last)) > 0;
}

/**
 * High-Precision Analog Read with Kahan Summation and Outlier Rejection.
 */
float readAnalogFiltered() {
    uint16_t minVal = 1023;
    uint16_t maxVal = 0;
    float sum = 0.0f;
    float c = 0.0f;

    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        uint16_t val = analogRead(SENSOR_PIN);
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;

        float y = (float)val - c;
        float t = sum + y;
        c = (t - sum) - y;
        sum = t;

        optimistic_yield(1000);
    }

    float averageAdc;
    if (ADC_OVERSAMPLE_COUNT >= 4) {
        float trimmedSum = sum - (float)minVal - (float)maxVal;
        averageAdc = trimmedSum / (float)(ADC_OVERSAMPLE_COUNT - 2);
    } else {
        averageAdc = sum / (float)ADC_OVERSAMPLE_COUNT;
    }

    kalman_p = kalman_p + KALMAN_PROCESS_NOISE_Q;
    float k_gain = kalman_p / (kalman_p + KALMAN_MEASUREMENT_NOISE_R);
    kalman_x = kalman_x + k_gain * (averageAdc - kalman_x);
    kalman_p = (1.0f - k_gain) * kalman_p;

    return kalman_x;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("ESP8266 Bi-directional Sensor Mesh Node (DSP Enhanced)\n");
    Serial.printf("My Node ID: %u -> Target Node ID: %u\n", MY_NODE_ID, TARGET_NODE_ID);
    Serial.printf("Analog In: A0 (DSP Kahan+Kalman) | PWM Out: GPIO %d (D1)\n", PWM_PIN);
    Serial.printf("Digital In: GPIO %d (D2) | Digital Out: GPIO %d (D3)\n", DIGITAL_INPUT_PIN, DIGITAL_OUTPUT_PIN);
    Serial.println("==================================================");

    // Initialize hardware pins
    pinMode(SENSOR_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0);

    pinMode(DIGITAL_INPUT_PIN, INPUT_PULLUP);
    pinMode(DIGITAL_OUTPUT_PIN, OUTPUT);
    digitalWrite(DIGITAL_OUTPUT_PIN, LOW);

    // Initialize Kalman state
    kalman_x = (float)analogRead(SENSOR_PIN);

    // Initialize painlessMesh network
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    // Add and enable sensor task to TaskScheduler
    userScheduler.addTask(taskSendSensorData);
    taskSendSensorData.enable();
}

void loop() {
    mesh.update();
}

/**
 * Reads DSP-filtered analog input and digital input, packs into binary struct,
 * hex-encodes it, and sends via targeted sendSingle or broadcast fallback.
 */
void sendSensorData() {
    float filteredVal = readAnalogFiltered();
    uint16_t highResSensorVal = (uint16_t)constrain((int)roundf(filteredVal), 0, 1023);

    uint8_t digitalVal = digitalRead(DIGITAL_INPUT_PIN);

    SensorMessage msg;
    msg.magic = MESSAGE_MAGIC;
    msg.sender_id = MY_NODE_ID;
    msg.target_id = TARGET_NODE_ID;
    msg.sensor_value = highResSensorVal;
    msg.digital_value = digitalVal;
    msg.seq = ++messageSequence;

    const size_t structSize = sizeof(SensorMessage);
    const size_t hexLen = structSize * 2;
    char hexBuffer[hexLen + 1];
    const uint8_t* rawBytes = (const uint8_t*)&msg;

    for (size_t i = 0; i < structSize; i++) {
        sprintf(&hexBuffer[i * 2], "%02X", rawBytes[i]);
    }
    hexBuffer[hexLen] = '\0';

    String payload(hexBuffer);

    // Targeted Unicast vs Discovery Broadcast
    bool sentDirect = false;
    if (targetMeshNodeId != 0 && mesh.isConnected(targetMeshNodeId)) {
        sentDirect = mesh.sendSingle(targetMeshNodeId, payload);
    }

    if (!sentDirect) {
        mesh.sendBroadcast(payload);
    }

    Serial.printf("[TX #%u] Filtered ADC: %.2f (Val: %u) | Digital D2: %u | Transport: %s (Dest MeshID: %u)\n",
                  msg.seq, filteredVal, highResSensorVal, digitalVal,
                  sentDirect ? "UNICAST (sendSingle)" : "BROADCAST", targetMeshNodeId);
}

/**
 * Callback when a mesh message is received.
 */
void receivedCallback(uint32_t from, String &msg) {
    const size_t expectedStructSize = sizeof(SensorMessage);
    const size_t expectedHexLen = expectedStructSize * 2;

    if (msg.length() != expectedHexLen) {
        return;
    }

    SensorMessage incoming;
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < expectedStructSize; i++) {
        char highNibble = msg.charAt(i * 2);
        char lowNibble = msg.charAt(i * 2 + 1);
        rawBytes[i] = (hexCharToNibble(highNibble) << 4) | hexCharToNibble(lowNibble);
    }

    if (incoming.magic != MESSAGE_MAGIC) {
        return;
    }

    if (incoming.target_id == MY_NODE_ID) {
        // Automatically learn target's transport painlessMesh Node ID
        if (incoming.sender_id == TARGET_NODE_ID) {
            targetMeshNodeId = from;
        }

        // Sequence check: reject stale, duplicate, or reordered packets
        if (!isNewerSequence(incoming.seq, lastReceivedSeq)) {
            Serial.printf("[RX DROP #%u] Out-of-order or duplicate packet dropped (Last Seq: %u, From MeshID: %u)\n",
                          incoming.seq, lastReceivedSeq, from);
            return;
        }

        lastReceivedSeq = incoming.seq;
        hasReceivedFirstPacket = true;

        // Map sensor value (0-1023) to PWM range
        uint16_t pwmValue = incoming.sensor_value;
        if (pwmValue > PWM_RANGE) {
            pwmValue = PWM_RANGE;
        }

        analogWrite(PWM_PIN, pwmValue);

        uint8_t digitalState = incoming.digital_value ? HIGH : LOW;
        digitalWrite(DIGITAL_OUTPUT_PIN, digitalState);

        Serial.printf("[RX #%u] From Node: %u | Analog: %u -> PWM Duty: %u/%d | Digital D2 -> D3: %u (Mesh NodeID: %u)\n",
                      incoming.seq, incoming.sender_id, incoming.sensor_value, pwmValue, PWM_RANGE, digitalState, from);
    } else {
        Serial.printf("[RELAY] From Node: %u to Target Node: %u (relayed via mesh node %u)\n",
                      incoming.sender_id, incoming.target_id, from);
    }
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("[MESH] New Connection, nodeId = %u (Local Mesh Node ID = %u)\n", nodeId, mesh.getNodeId());
}

void changedConnectionCallback() {
    Serial.printf("[MESH] Topology changed (Local Mesh Node ID = %u)\n", mesh.getNodeId());
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("[MESH] System time adjusted: %d us\n", offset);
}
