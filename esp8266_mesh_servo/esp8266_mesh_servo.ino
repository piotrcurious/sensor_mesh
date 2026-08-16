/*
  ESP8266 Bi-directional Servo & Sensor Mesh Node Firmware (High-Precision DSP & Rate Limited)
  Uses painlessMesh to create an auto-organizing mesh network.

  DSP & Rate Limiting Features:
  - High-frequency input polling (every 5 ms / 200 Hz) for immediate local motion update and limit switch clamping.
  - Rate-limited network transmissions (minimum 200 ms between mesh packet broadcasts).
  - Analog oversampling with Kahan Summation to eliminate floating point accumulation error.
  - Outlier rejection (trimmed mean discarding highest and lowest ADC samples).
  - 1D Kalman Filter to produce ultra-smooth analog readings.
  - High-precision Servo driving using myServo.writeMicroseconds().
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
void updateLocalServoMicroseconds(uint16_t requestedUs);
float readAnalogFiltered();

// High-rate polling task (5 ms)
Task taskPollInputs(POLL_INTERVAL_MS, TASK_FOREVER, &checkAndTransmitInputs);

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// State tracking for change detection & network rate limiting
static uint16_t lastTransmittedUs = 0xFFFF;
static uint8_t  lastDigitalVal = 0xFF;
static uint8_t  lastMinLimit = 0xFF;
static uint8_t  lastMaxLimit = 0xFF;
static uint32_t lastTxTime = 0;
static uint32_t messageSequence = 0;

// Current target microsecond pulse width for local servo
static uint16_t currentTargetUs = 1472; // Default midpoint (~90 deg)

// Helper: Convert uint8_t hex character ('0'-'9', 'A'-'F', 'a'-'f') to byte value
static uint8_t hexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

/**
 * High-Precision Analog Read:
 * 1. Takes ADC_OVERSAMPLE_COUNT samples.
 * 2. Uses Kahan Summation algorithm to accumulate total without precision loss.
 * 3. Applies Outlier Rejection (discards min and max values).
 * 4. Filters result through 1D Kalman Filter.
 */
float readAnalogFiltered() {
    uint16_t samples[ADC_OVERSAMPLE_COUNT];
    uint16_t minVal = 1024;
    uint16_t maxVal = 0;

    // 1. Oversample ADC
    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        uint16_t val = analogRead(SENSOR_PIN);
        samples[i] = val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }

    // 2. Kahan Summation with Outlier Rejection
    float sum = 0.0f;
    float c = 0.0f; // Compensation variable for lost low-order bits
    size_t validSamplesCount = 0;

    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        // Outlier rejection: discard min and max if oversampling count >= 4
        if (ADC_OVERSAMPLE_COUNT >= 4 && (samples[i] == minVal || samples[i] == maxVal)) {
            continue;
        }
        float y = (float)samples[i] - c;
        float t = sum + y;
        c = (t - sum) - y;
        sum = t;
        validSamplesCount++;
    }

    float averageAdc = (validSamplesCount > 0) ? (sum / (float)validSamplesCount) : (float)minVal;

    // 3. 1D Kalman Filter Update
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
    Serial.printf("ESP8266 Bi-directional Servo Mesh Node (5ms Poll / 200ms Rate Limit)\n");
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

    // Limit switches are Active LOW with internal pullup
    pinMode(MIN_LIMIT_PIN, INPUT_PULLUP);
    pinMode(MAX_LIMIT_PIN, INPUT_PULLUP);

    // Initialize Kalman state
    kalman_x = (float)analogRead(SENSOR_PIN);

    // Attach Servo with calibrated pulse width range (544 to 2400 us)
    myServo.attach(SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    updateLocalServoMicroseconds(currentTargetUs);

    // Initialize painlessMesh network
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    // Add and enable high-rate polling task
    userScheduler.addTask(taskPollInputs);
    taskPollInputs.enable();
}

void loop() {
    // Keep painlessMesh and TaskScheduler running
    mesh.update();
}

/**
 * Polls inputs at high frequency (5 ms).
 * Updates local limit switch clamping immediately.
 * Enforces MIN_TX_INTERVAL_MS (200 ms) rate limiting on mesh broadcast transmissions.
 */
void checkAndTransmitInputs() {
    // Read high-precision filtered analog value (0.0f - 1023.0f)
    float filteredAdc = readAnalogFiltered();

    // Map high-precision ADC reading directly to servo microseconds (544 - 2400 us)
    float targetPulseUs = SERVO_MIN_PULSE_WIDTH + (filteredAdc / 1023.0f) * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH);
    uint16_t currentUs = (uint16_t)constrain((int)roundf(targetPulseUs), SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

    uint8_t currentDigital = digitalRead(DIGITAL_INPUT_PIN);

    // Limit switches are active LOW (0 when pressed/active, 1 when open)
    uint8_t currentMinLimit = (digitalRead(MIN_LIMIT_PIN) == LOW) ? 1 : 0;
    uint8_t currentMaxLimit = (digitalRead(MAX_LIMIT_PIN) == LOW) ? 1 : 0;

    // Local limit switch reaction (updates local servo immediately every 5ms)
    if (currentMinLimit != lastMinLimit || currentMaxLimit != lastMaxLimit) {
        updateLocalServoMicroseconds(currentTargetUs);
    }

    // Determine if state changed significantly
    bool pulseChanged = (abs((int)currentUs - (int)lastTransmittedUs) >= PULSE_CHANGE_THRESHOLD);
    bool digitalChanged = (currentDigital != lastDigitalVal);
    bool minLimitChanged = (currentMinLimit != lastMinLimit);
    bool maxLimitChanged = (currentMaxLimit != lastMaxLimit);
    bool stateChanged = (pulseChanged || digitalChanged || minLimitChanged || maxLimitChanged);

    uint32_t now = millis();
    bool rateLimitElapsed = (now - lastTxTime >= MIN_TX_INTERVAL_MS);
    bool heartbeatElapsed = (now - lastTxTime >= HEARTBEAT_INTERVAL_MS);

    // Enforce 200 ms network rate limiting
    if ((stateChanged && rateLimitElapsed) || heartbeatElapsed) {
        // Construct binary payload
        ServoMeshMessage msg;
        msg.magic = MESSAGE_MAGIC;
        msg.sender_id = MY_NODE_ID;
        msg.target_id = TARGET_NODE_ID;
        msg.target_us = currentUs;
        msg.digital_value = currentDigital;
        msg.min_limit_active = currentMinLimit;
        msg.max_limit_active = currentMaxLimit;
        msg.seq = ++messageSequence;

        // Hex-encode struct
        const size_t structSize = sizeof(ServoMeshMessage);
        const size_t hexLen = structSize * 2;
        char hexBuffer[hexLen + 1];
        const uint8_t* rawBytes = (const uint8_t*)&msg;

        for (size_t i = 0; i < structSize; i++) {
            sprintf(&hexBuffer[i * 2], "%02X", rawBytes[i]);
        }
        hexBuffer[hexLen] = '\0';

        mesh.sendBroadcast(String(hexBuffer));

        // Update last transmitted state tracking & timestamp
        lastTransmittedUs = currentUs;
        lastDigitalVal = currentDigital;
        lastMinLimit = currentMinLimit;
        lastMaxLimit = currentMaxLimit;
        lastTxTime = now;

        Serial.printf("[TX #%u] Filtered ADC: %.2f | Target Pulse: %u us | Digital: %u | MinLim: %u | MaxLim: %u (Hex: %s)\n",
                      msg.seq, filteredAdc, currentUs, currentDigital, currentMinLimit, currentMaxLimit, hexBuffer);
    }
}

/**
 * Calculates and updates local servo pulse width in microseconds while respecting local limit switch clamping.
 */
void updateLocalServoMicroseconds(uint16_t requestedUs) {
    currentTargetUs = requestedUs;

    uint16_t targetUs = constrain(requestedUs, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

    // Read current local limit switches
    bool minLimitActive = (digitalRead(MIN_LIMIT_PIN) == LOW);
    bool maxLimitActive = (digitalRead(MAX_LIMIT_PIN) == LOW);

    uint16_t finalUs = targetUs;
    uint16_t midPulse = (SERVO_MIN_PULSE_WIDTH + SERVO_MAX_PULSE_WIDTH) / 2;

    // Clamp servo movement if limit switches are triggered
    if (minLimitActive && finalUs < midPulse) {
        finalUs = SERVO_MIN_PULSE_WIDTH;
    }
    if (maxLimitActive && finalUs > midPulse) {
        finalUs = SERVO_MAX_PULSE_WIDTH;
    }

    // Drive servo with microsecond resolution
    myServo.writeMicroseconds(finalUs);

    Serial.printf("[SERVO us] Requested Pulse: %u us -> Final Pulse: %u us (MinLim: %d, MaxLim: %d)\n",
                  requestedUs, finalUs, minLimitActive, maxLimitActive);
}

/**
 * Callback when a mesh message is received.
 */
void receivedCallback(uint32_t from, String &msg) {
    const size_t expectedStructSize = sizeof(ServoMeshMessage);
    const size_t expectedHexLen = expectedStructSize * 2;

    if (msg.length() != expectedHexLen) {
        return;
    }

    // Decode HEX string
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

    if (incoming.target_id == MY_NODE_ID) {
        // Update local digital output (D3)
        digitalWrite(DIGITAL_OUTPUT_PIN, incoming.digital_value ? HIGH : LOW);

        // Update local servo with received high-precision microsecond target
        updateLocalServoMicroseconds(incoming.target_us);

        Serial.printf("[RX #%u] From Node: %u | Target Pulse: %u us | Digital: %u | Remote MinLim: %u | Remote MaxLim: %u\n",
                      incoming.seq, incoming.sender_id, incoming.target_us, incoming.digital_value,
                      incoming.min_limit_active, incoming.max_limit_active);
    } else {
        Serial.printf("[RELAY] From Node: %u to Target Node: %u (relayed via %u)\n",
                      incoming.sender_id, incoming.target_id, from);
    }
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("[MESH] New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
    Serial.printf("[MESH] Topology changed\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("[MESH] System time adjusted: %d us\n", offset);
}
