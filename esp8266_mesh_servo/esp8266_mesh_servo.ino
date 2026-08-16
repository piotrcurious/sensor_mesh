/*
  ESP8266 Bi-directional Servo & Sensor Mesh Node Firmware (Sub-Microsecond Fixed-Point Precision & Directional Safety)
  Uses painlessMesh to create an auto-organizing mesh network.

  Fixes & Enhancements:
  - Sub-microsecond Fixed-Point Precision (FP4 = 1/16th us resolution):
    Preserves fractional ADC precision gained through Kahan oversampling and Kalman filtering,
    avoiding coarse integer rounding before transmission.
  - Wi-Fi PHY & Mesh connection stability: ADC sampling is yield-friendly to avoid blocking Wi-Fi PHY interrupts.
  - Directional limit switch safety clamping:
    * MIN active: forbids further movement toward MIN (targetUsFp4 < lastSafeUsFp4), allows movement toward MAX.
    * MAX active: forbids further movement toward MAX (targetUsFp4 > lastSafeUsFp4), allows movement toward MIN.
    * Separates requestedServoUsFp4 from appliedServoUsFp4 / lastSafeUsFp4.
  - Correct single min/max trimmed-mean outlier rejection with Kahan summation.
  - Polled input reading (50 ms / 20 Hz) for immediate local motion update.
  - Rate-limited network transmissions (minimum 200 ms between mesh packet broadcasts).
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
void updateLocalServoFp4(uint16_t requestedUsFp4);
float readAnalogFiltered();

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

// Motion control state separation in fixed-point 1/16th microseconds (FP4)
// Default midpoint (~90 deg) = 1472 us * 16 = 23552
static uint16_t requestedServoUsFp4 = 23552;
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
 * High-Precision Analog Read:
 * 1. Takes ADC_OVERSAMPLE_COUNT samples with yields to ensure Wi-Fi PHY stability.
 * 2. Uses Kahan Summation algorithm to accumulate total without precision loss.
 * 3. Applies Outlier Rejection: subtracts exactly ONE minVal and ONE maxVal.
 * 4. Filters result through 1D Kalman Filter.
 */
float readAnalogFiltered() {
    uint16_t minVal = 1023;
    uint16_t maxVal = 0;
    float sum = 0.0f;
    float c = 0.0f; // Compensation variable for lost low-order bits

    // 1. Oversample ADC & accumulate with Kahan summation
    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        uint16_t val = analogRead(SENSOR_PIN);
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;

        float y = (float)val - c;
        float t = sum + y;
        c = (t - sum) - y;
        sum = t;

        optimistic_yield(1000); // Allow Wi-Fi stack & PHY tasks to process
    }

    // 2. Outlier Rejection: Subtract exactly ONE minVal and ONE maxVal
    float averageAdc;
    if (ADC_OVERSAMPLE_COUNT >= 4) {
        float trimmedSum = sum - (float)minVal - (float)maxVal;
        averageAdc = trimmedSum / (float)(ADC_OVERSAMPLE_COUNT - 2);
    } else {
        averageAdc = sum / (float)ADC_OVERSAMPLE_COUNT;
    }

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
    Serial.printf("ESP8266 Bi-directional Servo Mesh Node (Sub-Microsecond FP4 Precision)\n");
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
    // Keep painlessMesh and TaskScheduler running
    mesh.update();
}

/**
 * Polls inputs at 50 ms intervals.
 * Maps floating point Kalman ADC value to 1/16th us fixed-point (FP4) pulse width.
 * Updates local limit switch safety clamping immediately.
 * Enforces MIN_TX_INTERVAL_MS (200 ms) rate limiting on mesh broadcast transmissions.
 */
void checkAndTransmitInputs() {
    // Read high-precision filtered analog value (0.0f - 1023.0f)
    float filteredAdc = readAnalogFiltered();

    // Map high-precision ADC reading directly to sub-microsecond pulse duration in FP4 units (us * 16)
    float targetPulseUs = (float)SERVO_MIN_PULSE_WIDTH + (filteredAdc / 1023.0f) * (float)(SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH);
    float targetPulseFp4Float = targetPulseUs * 16.0f;

    uint16_t minUsFp4 = SERVO_MIN_PULSE_WIDTH * 16;
    uint16_t maxUsFp4 = SERVO_MAX_PULSE_WIDTH * 16;

    uint16_t currentUsFp4 = (uint16_t)constrain((int)roundf(targetPulseFp4Float), minUsFp4, maxUsFp4);

    uint8_t currentDigital = digitalRead(DIGITAL_INPUT_PIN);

    // Limit switches are active LOW (0 when pressed/active, 1 when open)
    uint8_t currentMinLimit = (digitalRead(MIN_LIMIT_PIN) == LOW) ? 1 : 0;
    uint8_t currentMaxLimit = (digitalRead(MAX_LIMIT_PIN) == LOW) ? 1 : 0;

    // Local limit switch reaction (updates local servo safety clamping immediately)
    if (currentMinLimit != lastMinLimit || currentMaxLimit != lastMaxLimit) {
        updateLocalServoFp4(requestedServoUsFp4);
    }

    // Determine if state changed significantly (delta >= 8 FP4 units = 0.5 us)
    bool pulseChanged = (abs((int)currentUsFp4 - (int)lastTransmittedUsFp4) >= PULSE_FP4_CHANGE_THRESHOLD);
    bool digitalChanged = (currentDigital != lastDigitalVal);
    bool minLimitChanged = (currentMinLimit != lastMinLimit);
    bool maxLimitChanged = (currentMaxLimit != lastMaxLimit);
    bool stateChanged = (pulseChanged || digitalChanged || minLimitChanged || maxLimitChanged);

    uint32_t now = millis();
    bool rateLimitElapsed = (now - lastTxTime >= MIN_TX_INTERVAL_MS);
    bool heartbeatElapsed = (now - lastTxTime >= HEARTBEAT_INTERVAL_MS);

    // Enforce 200 ms network rate limiting
    if ((stateChanged && rateLimitElapsed) || heartbeatElapsed) {
        // Construct binary payload with FP4 sub-microsecond precision
        ServoMeshMessage msg;
        msg.magic = MESSAGE_MAGIC;
        msg.sender_id = MY_NODE_ID;
        msg.target_id = TARGET_NODE_ID;
        msg.target_us_fp4 = currentUsFp4;
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
        lastTransmittedUsFp4 = currentUsFp4;
        lastDigitalVal = currentDigital;
        lastMinLimit = currentMinLimit;
        lastMaxLimit = currentMaxLimit;
        lastTxTime = now;

        Serial.printf("[TX #%u] Filtered ADC: %.2f | Target Pulse: %.2f us (FP4: %u) | Digital: %u | MinLim: %u | MaxLim: %u (Hex: %s)\n",
                      msg.seq, filteredAdc, (float)currentUsFp4 / 16.0f, currentUsFp4, currentDigital, currentMinLimit, currentMaxLimit, hexBuffer);
    }
}

/**
 * Calculates and updates local servo pulse width with FP4 sub-microsecond precision & directional limit clamping.
 *
 * Safety Model:
 * - MIN active: forbids movement toward MIN (targetUsFp4 < lastSafeUsFp4), allows movement toward MAX.
 * - MAX active: forbids movement toward MAX (targetUsFp4 > lastSafeUsFp4), allows movement toward MIN.
 * - Keeps separate tracking for requestedServoUsFp4, appliedServoUsFp4, and lastSafeUsFp4.
 */
void updateLocalServoFp4(uint16_t newRequestedUsFp4) {
    requestedServoUsFp4 = newRequestedUsFp4;

    uint16_t minUsFp4 = SERVO_MIN_PULSE_WIDTH * 16;
    uint16_t maxUsFp4 = SERVO_MAX_PULSE_WIDTH * 16;

    uint16_t targetUsFp4 = constrain(requestedServoUsFp4, minUsFp4, maxUsFp4);

    // Read current local limit switches (Active LOW)
    bool minActive = (digitalRead(MIN_LIMIT_PIN) == LOW);
    bool maxActive = (digitalRead(MAX_LIMIT_PIN) == LOW);

    // Apply directional limit switch protection
    if (minActive && targetUsFp4 < lastSafeUsFp4) {
        // Block further movement toward MIN, hold last safe position
        targetUsFp4 = lastSafeUsFp4;
    }

    if (maxActive && targetUsFp4 > lastSafeUsFp4) {
        // Block further movement toward MAX, hold last safe position
        targetUsFp4 = lastSafeUsFp4;
    }

    // Convert FP4 (1/16th us) to integer microseconds for myServo.writeMicroseconds
    uint16_t targetUsInt = (uint16_t)roundf((float)targetUsFp4 / 16.0f);

    // Drive servo with safe target pulse width
    myServo.writeMicroseconds(targetUsInt);

    appliedServoUsFp4 = targetUsFp4;
    lastSafeUsFp4 = targetUsFp4;

    Serial.printf("[SERVO FP4] Requested: %.2f us (%u) -> Applied: %u us (%u FP4) | LastSafe: %u FP4 (MinLim: %d, MaxLim: %d)\n",
                  (float)requestedServoUsFp4 / 16.0f, requestedServoUsFp4, targetUsInt, appliedServoUsFp4, lastSafeUsFp4, minActive, maxActive);
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

        // Update local servo with received sub-microsecond FP4 target
        updateLocalServoFp4(incoming.target_us_fp4);

        Serial.printf("[RX #%u] From Node: %u | Target Pulse: %.2f us (%u FP4) | Digital: %u | Remote MinLim: %u | Remote MaxLim: %u\n",
                      incoming.seq, incoming.sender_id, (float)incoming.target_us_fp4 / 16.0f, incoming.target_us_fp4,
                      incoming.digital_value, incoming.min_limit_active, incoming.max_limit_active);
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
