/*
  ESP8266 Bi-directional Sensor Mesh Node Firmware (High-Precision DSP)
  Uses painlessMesh to create an auto-organizing mesh network.
  Reads analog input pin (A0) with Kahan summation oversampling, outlier rejection,
  and 1D Kalman filtering every 1 second and transmits compact binary packed struct data
  to a paired node (configured via MY_NODE_ID and TARGET_NODE_ID).
  When receiving messages addressed to MY_NODE_ID, sets PWM output pin (D1) and
  digital output pin (D3).
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

// Task to read sensor and transmit data every 1 second (SEND_INTERVAL_MS)
Task taskSendSensorData(SEND_INTERVAL_MS, TASK_FOREVER, &sendSensorData);

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// Sequence counter
static uint32_t messageSequence = 0;

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
    Serial.printf("ESP8266 Bi-directional Sensor Mesh Node (DSP Enhanced)\n");
    Serial.printf("My Node ID: %u -> Target Node ID: %u\n", MY_NODE_ID, TARGET_NODE_ID);
    Serial.printf("Analog In: A0 (DSP Kahan+Kalman) | PWM Out: GPIO %d (D1)\n", PWM_PIN);
    Serial.printf("Digital In: GPIO %d (D2) | Digital Out: GPIO %d (D3)\n", DIGITAL_INPUT_PIN, DIGITAL_OUTPUT_PIN);
    Serial.println("==================================================");

    // Initialize hardware pins
    pinMode(SENSOR_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0); // Start PWM at 0% duty cycle

    pinMode(DIGITAL_INPUT_PIN, INPUT_PULLUP);
    pinMode(DIGITAL_OUTPUT_PIN, OUTPUT);
    digitalWrite(DIGITAL_OUTPUT_PIN, LOW); // Default digital output state

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
    // Keep painlessMesh and TaskScheduler running
    mesh.update();
}

/**
 * Reads DSP-filtered analog input and digital input, packs into binary struct,
 * hex-encodes it, and broadcasts over mesh.
 */
void sendSensorData() {
    // Read high-precision filtered analog value
    float filteredVal = readAnalogFiltered();
    uint16_t highResSensorVal = (uint16_t)constrain((int)roundf(filteredVal), 0, 1023);

    // Read digital pin (D2)
    uint8_t digitalVal = digitalRead(DIGITAL_INPUT_PIN);

    // Construct binary payload
    SensorMessage msg;
    msg.magic = MESSAGE_MAGIC;
    msg.sender_id = MY_NODE_ID;
    msg.target_id = TARGET_NODE_ID;
    msg.sensor_value = highResSensorVal;
    msg.digital_value = digitalVal;
    msg.seq = ++messageSequence;

    // Hex-encode struct to avoid 0x00 null byte truncation in painlessMesh JSON serialization
    const size_t structSize = sizeof(SensorMessage);
    const size_t hexLen = structSize * 2;
    char hexBuffer[hexLen + 1];
    const uint8_t* rawBytes = (const uint8_t*)&msg;

    for (size_t i = 0; i < structSize; i++) {
        sprintf(&hexBuffer[i * 2], "%02X", rawBytes[i]);
    }
    hexBuffer[hexLen] = '\0';

    mesh.sendBroadcast(String(hexBuffer));

    Serial.printf("[TX #%u] Filtered ADC: %.2f (Val: %u) | Digital D2: %u -> Target Node: %u (Hex: %s)\n",
                  msg.seq, filteredVal, highResSensorVal, digitalVal, TARGET_NODE_ID, hexBuffer);
}

/**
 * Callback when a mesh message is received.
 */
void receivedCallback(uint32_t from, String &msg) {
    const size_t expectedStructSize = sizeof(SensorMessage);
    const size_t expectedHexLen = expectedStructSize * 2;

    // Validate string length matches expected HEX payload size
    if (msg.length() != expectedHexLen) {
        return;
    }

    // Decode HEX string back into binary SensorMessage struct
    SensorMessage incoming;
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < expectedStructSize; i++) {
        char highNibble = msg.charAt(i * 2);
        char lowNibble = msg.charAt(i * 2 + 1);
        rawBytes[i] = (hexCharToNibble(highNibble) << 4) | hexCharToNibble(lowNibble);
    }

    // Verify magic byte
    if (incoming.magic != MESSAGE_MAGIC) {
        return;
    }

    // Check if message is directed to this node
    if (incoming.target_id == MY_NODE_ID) {
        // Map sensor value (0-1023) to PWM range (0-PWM_RANGE)
        uint16_t pwmValue = incoming.sensor_value;
        if (pwmValue > PWM_RANGE) {
            pwmValue = PWM_RANGE;
        }

        // Drive the PWM pin (D1)
        analogWrite(PWM_PIN, pwmValue);

        // Drive the digital output pin (D3)
        uint8_t digitalState = incoming.digital_value ? HIGH : LOW;
        digitalWrite(DIGITAL_OUTPUT_PIN, digitalState);

        Serial.printf("[RX #%u] From Node: %u | Analog: %u -> PWM Duty: %u/%d | Digital D2 -> D3: %u (Mesh NodeID: %u)\n",
                      incoming.seq, incoming.sender_id, incoming.sensor_value, pwmValue, PWM_RANGE, digitalState, from);
    } else {
        // Message is relayed automatically by painlessMesh to other nodes
        Serial.printf("[RELAY] From Node: %u to Target Node: %u (relayed via mesh node %u)\n",
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
