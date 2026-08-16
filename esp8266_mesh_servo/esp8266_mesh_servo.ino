/*
  ESP8266 Bi-directional Servo & Sensor Mesh Node Firmware (Sender-Validated Unicast & Sequence Tracking)
  Uses painlessMesh to create an auto-organizing mesh network.

  Fixes & Enhancements:
  - Explicit Paired Sender Validation & Single-Source Sequence Verification:
    * Rejects packets unless incoming.sender_id == TARGET_NODE_ID AND incoming.target_id == MY_NODE_ID.
    * Ensures the single sequence tracking state (isNewerSequence) is logically isolated to the paired sender.
    * Prevents sequence corruption or packet rejection from un-paired mesh nodes.
  - Unicast Mesh Transport & Dynamic Target Discovery:
    * Dynamically maps logical TARGET_NODE_ID to painlessMesh uint32_t transport node ID.
    * Uses targeted mesh.sendSingle(targetMeshNodeId, payload) for direct unicast transport when connected.
  - Sub-microsecond Fixed-Point Precision (FP4 = 1/16th us resolution).
  - Wi-Fi PHY & Mesh connection stability (yield-friendly ADC oversampling).
  - Directional limit switch safety clamping.
*/

#include <painlessMesh.h>
#include <Servo.h>
#include "config.h"

// TaskScheduler, painlessMesh, and Servo instances
Scheduler userScheduler;
painlessMesh mesh;
Servo myServo;

// Function declarations
void checkAndTransmitInputs();
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void updateLocalServoFp4(uint16_t requestedUsFp4);
float readAnalogFiltered();
bool isNewerSequence(uint32_t incoming, uint32_t last);

// Input polling task (50 ms)
Task taskPollInputs(POLL_INTERVAL_MS, TASK_FOREVER, &checkAndTransmitInputs);

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// State tracking for change detection & network rate limiting (1/16th us units)
static uint16_t lastTransmittedUsFp4 = 0xFFFF;
static uint8_t  lastDigitalVal = 0xFF;
static uint8_t  lastMinLimit = 0xFF;
static uint8_t  lastMaxLimit = 0xFF;
static uint32_t lastTxTime = 0;
static uint32_t messageSequence = 0;

// Sequence verification and dynamic target painlessMesh Node ID mapping (Isolated to TARGET_NODE_ID)
static uint32_t lastReceivedSeq = 0;
static bool     hasReceivedFirstPacket = false;
static uint32_t targetMeshNodeId = 0; // Discovered painlessMesh uint32_t node ID for TARGET_NODE_ID

// Motion control state separation in fixed-point 1/16th microseconds (FP4)
static uint16_t requestedServoUsFp4 = 23552; // 1472 us * 16
static uint16_t appliedServoUsFp4   = 23552;
static uint16_t lastSafeUsFp4       = 23552;

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
    // Signed difference handles 32-bit integer overflow/wraparound correctly
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
    Serial.printf("ESP8266 Bi-directional Servo Mesh Node\n");
    Serial.printf("My Node ID: %u -> Target Node ID: %u\n", MY_NODE_ID, TARGET_NODE_ID);
    Serial.printf("Analog In: A0 (DSP Kahan+Kalman) | Servo Pin: GPIO %d (D1) [%d - %d us]\n",
                  SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    Serial.printf("Digital In: GPIO %d (D2) | Digital Out: GPIO %d (D3)\n", DIGITAL_INPUT_PIN, DIGITAL_OUTPUT_PIN);
    Serial.printf("Min Limit Pin: GPIO %d (D6) | Max Limit Pin: GPIO %d (D7)\n", MIN_LIMIT_PIN, MAX_LIMIT_PIN);
    Serial.println("==================================================");

    // Initialize hardware pins
    pinMode(SENSOR_PIN, INPUT);
    pinMode(DIGITAL_INPUT_PIN, INPUT_PULLUP);
    pinMode(DIGITAL_OUTPUT_PIN, OUTPUT);
    digitalWrite(DIGITAL_OUTPUT_PIN, LOW);

    pinMode(MIN_LIMIT_PIN, INPUT_PULLUP);
    pinMode(MAX_LIMIT_PIN, INPUT_PULLUP);

    // Initialize Kalman state
    kalman_x = (float)analogRead(SENSOR_PIN);

    // Attach Servo with calibrated pulse width range
    myServo.attach(SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    updateLocalServoFp4(requestedServoUsFp4);

    // Initialize painlessMesh network
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    // Add and enable polling task
    userScheduler.addTask(taskPollInputs);
    taskPollInputs.enable();
}

void loop() {
    mesh.update();
}

/**
 * Polls inputs and transmits packet.
 * Uses mesh.sendSingle() when targetMeshNodeId is discovered, falling back to broadcast during discovery.
 */
void checkAndTransmitInputs() {
    float filteredAdc = readAnalogFiltered();

    float targetPulseUs = (float)SERVO_MIN_PULSE_WIDTH + (filteredAdc / 1023.0f) * (float)(SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH);
    float targetPulseFp4Float = targetPulseUs * 16.0f;

    uint16_t minUsFp4 = SERVO_MIN_PULSE_WIDTH * 16;
    uint16_t maxUsFp4 = SERVO_MAX_PULSE_WIDTH * 16;

    uint16_t currentUsFp4 = (uint16_t)constrain((int)roundf(targetPulseFp4Float), minUsFp4, maxUsFp4);

    uint8_t currentDigital = digitalRead(DIGITAL_INPUT_PIN);
    uint8_t currentMinLimit = (digitalRead(MIN_LIMIT_PIN) == LOW) ? 1 : 0;
    uint8_t currentMaxLimit = (digitalRead(MAX_LIMIT_PIN) == LOW) ? 1 : 0;

    if (currentMinLimit != lastMinLimit || currentMaxLimit != lastMaxLimit) {
        updateLocalServoFp4(requestedServoUsFp4);
    }

    bool pulseChanged = (abs((int)currentUsFp4 - (int)lastTransmittedUsFp4) >= PULSE_FP4_CHANGE_THRESHOLD);
    bool digitalChanged = (currentDigital != lastDigitalVal);
    bool minLimitChanged = (currentMinLimit != lastMinLimit);
    bool maxLimitChanged = (currentMaxLimit != lastMaxLimit);
    bool stateChanged = (pulseChanged || digitalChanged || minLimitChanged || maxLimitChanged);

    uint32_t now = millis();
    bool rateLimitElapsed = (now - lastTxTime >= MIN_TX_INTERVAL_MS);
    bool heartbeatElapsed = (now - lastTxTime >= HEARTBEAT_INTERVAL_MS);

    if ((stateChanged && rateLimitElapsed) || heartbeatElapsed) {
        ServoMeshMessage msg;
        msg.magic = MESSAGE_MAGIC;
        msg.sender_id = MY_NODE_ID;
        msg.target_id = TARGET_NODE_ID;
        msg.target_us_fp4 = currentUsFp4;
        msg.digital_value = currentDigital;
        msg.min_limit_active = currentMinLimit;
        msg.max_limit_active = currentMaxLimit;
        msg.seq = ++messageSequence;

        const size_t structSize = sizeof(ServoMeshMessage);
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

        lastTransmittedUsFp4 = currentUsFp4;
        lastDigitalVal = currentDigital;
        lastMinLimit = currentMinLimit;
        lastMaxLimit = currentMaxLimit;
        lastTxTime = now;

        Serial.printf("[TX #%u] Filtered ADC: %.2f | Target Pulse: %.2f us (FP4: %u) | Transport: %s (Dest MeshID: %u)\n",
                      msg.seq, filteredAdc, (float)currentUsFp4 / 16.0f, currentUsFp4,
                      sentDirect ? "UNICAST (sendSingle)" : "BROADCAST", targetMeshNodeId);
    }
}

/**
 * Calculates and updates local servo pulse width with FP4 sub-microsecond precision & directional limit clamping.
 */
void updateLocalServoFp4(uint16_t newRequestedUsFp4) {
    requestedServoUsFp4 = newRequestedUsFp4;

    uint16_t minUsFp4 = SERVO_MIN_PULSE_WIDTH * 16;
    uint16_t maxUsFp4 = SERVO_MAX_PULSE_WIDTH * 16;

    uint16_t targetUsFp4 = constrain(requestedServoUsFp4, minUsFp4, maxUsFp4);

    bool minActive = (digitalRead(MIN_LIMIT_PIN) == LOW);
    bool maxActive = (digitalRead(MAX_LIMIT_PIN) == LOW);

    if (minActive && targetUsFp4 < lastSafeUsFp4) {
        targetUsFp4 = lastSafeUsFp4;
    }

    if (maxActive && targetUsFp4 > lastSafeUsFp4) {
        targetUsFp4 = lastSafeUsFp4;
    }

    uint16_t targetUsInt = (uint16_t)roundf((float)targetUsFp4 / 16.0f);

    myServo.writeMicroseconds(targetUsInt);

    appliedServoUsFp4 = targetUsFp4;
    lastSafeUsFp4 = targetUsFp4;

    Serial.printf("[SERVO FP4] Requested: %.2f us -> Applied: %u us (%u FP4) | LastSafe: %u FP4 (MinLim: %d, MaxLim: %d)\n",
                  (float)requestedServoUsFp4 / 16.0f, targetUsInt, appliedServoUsFp4, lastSafeUsFp4, minActive, maxActive);
}

/**
 * Callback when a mesh message is received.
 * Explicitly validates both sender_id and target_id before sequence checking.
 */
void receivedCallback(uint32_t from, String &msg) {
    const size_t expectedStructSize = sizeof(ServoMeshMessage);
    const size_t expectedHexLen = expectedStructSize * 2;

    if (msg.length() != expectedHexLen) {
        return;
    }

    ServoMeshMessage incoming;
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < expectedStructSize; i++) {
        char highNibble = msg.charAt(i * 2);
        char lowNibble = msg.charAt(i * 2 + 1);
        rawBytes[i] = (hexCharToNibble(highNibble) << 4) | hexCharToNibble(lowNibble);
    }

    if (incoming.magic != MESSAGE_MAGIC) {
        return;
    }

    // Explicit Sender & Target Validation:
    // Only accept messages originating from compile-time TARGET_NODE_ID addressed to MY_NODE_ID
    if (incoming.sender_id != TARGET_NODE_ID || incoming.target_id != MY_NODE_ID) {
        return;
    }

    // Automatically learn/update target's transport painlessMesh Node ID
    targetMeshNodeId = from;

    // Single-source sequence check: reject stale, duplicate, or reordered packets from paired sender
    if (!isNewerSequence(incoming.seq, lastReceivedSeq)) {
        Serial.printf("[RX DROP #%u] Out-of-order or duplicate packet dropped (Last Seq: %u, From Sender: %u, MeshID: %u)\n",
                      incoming.seq, lastReceivedSeq, incoming.sender_id, from);
        return;
    }

    lastReceivedSeq = incoming.seq;
    hasReceivedFirstPacket = true;

    // Update local digital output (D3)
    digitalWrite(DIGITAL_OUTPUT_PIN, incoming.digital_value ? HIGH : LOW);

    // Update local servo
    updateLocalServoFp4(incoming.target_us_fp4);

    Serial.printf("[RX #%u] From Node: %u | Target Pulse: %.2f us (%u FP4) | Digital: %u | Remote MinLim: %u | Remote MaxLim: %u\n",
                  incoming.seq, incoming.sender_id, (float)incoming.target_us_fp4 / 16.0f, incoming.target_us_fp4,
                  incoming.digital_value, incoming.min_limit_active, incoming.max_limit_active);
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
