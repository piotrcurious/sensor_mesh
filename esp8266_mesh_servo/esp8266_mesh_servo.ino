/*
  ESP8266 Bi-directional Servo & Sensor Mesh Node Firmware
  Uses painlessMesh to create an auto-organizing mesh network.

  Features:
  - Event-driven transmission: sends updates immediately when input states change
    (analog input A0, digital input D2, min limit switch D6, max limit switch D7),
    plus periodic heartbeat transmissions.
  - Controls local Servo motor (D1) based on target node analog position.
  - Servo pulse-width range calibrated (544 us to 2400 us) for standard micro-servos (e.g. SG90/MG996R).
  - Local limit switch clamping: limit switches (D6, D7) clamp local servo movement
    to protect physical hardware.
  - Transmits local limit switch states to corresponding paired node.
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
void updateLocalServoAngle(uint16_t requestedAnalogVal);

// Polling task (checks for state changes every POLL_INTERVAL_MS)
Task taskPollInputs(POLL_INTERVAL_MS, TASK_FOREVER, &checkAndTransmitInputs);

// State tracking for change detection
static uint16_t lastAnalogVal = 0xFFFF;
static uint8_t  lastDigitalVal = 0xFF;
static uint8_t  lastMinLimit = 0xFF;
static uint8_t  lastMaxLimit = 0xFF;
static uint32_t lastTxTime = 0;
static uint32_t messageSequence = 0;

// Current requested analog position for local servo
static uint16_t currentTargetAnalogVal = 512;

// Helper: Convert uint8_t hex character ('0'-'9', 'A'-'F', 'a'-'f') to byte value
static uint8_t hexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("ESP8266 Bi-directional Servo & Sensor Mesh Node\n");
    Serial.printf("My Node ID: %u -> Target Node ID: %u\n", MY_NODE_ID, TARGET_NODE_ID);
    Serial.printf("Analog In: A0 | Servo Pin: GPIO %d (D1) [Pulse: %d - %d us]\n",
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

    // Attach Servo with calibrated pulse width range (544 to 2400 us)
    myServo.attach(SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    updateLocalServoAngle(currentTargetAnalogVal);

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
 * Checks all inputs for state changes and transmits if changed or if heartbeat interval elapsed.
 */
void checkAndTransmitInputs() {
    uint16_t currentAnalog = analogRead(SENSOR_PIN);
    uint8_t currentDigital = digitalRead(DIGITAL_INPUT_PIN);

    // Limit switches are active LOW (0 when pressed/active, 1 when open)
    uint8_t currentMinLimit = (digitalRead(MIN_LIMIT_PIN) == LOW) ? 1 : 0;
    uint8_t currentMaxLimit = (digitalRead(MAX_LIMIT_PIN) == LOW) ? 1 : 0;

    // Check if limit switch state changed locally -> update local servo clamping immediately
    if (currentMinLimit != lastMinLimit || currentMaxLimit != lastMaxLimit) {
        updateLocalServoAngle(currentTargetAnalogVal);
    }

    // Determine if state changed significantly
    bool analogChanged = (abs((int)currentAnalog - (int)lastAnalogVal) >= ANALOG_CHANGE_THRESHOLD);
    bool digitalChanged = (currentDigital != lastDigitalVal);
    bool minLimitChanged = (currentMinLimit != lastMinLimit);
    bool maxLimitChanged = (currentMaxLimit != lastMaxLimit);

    uint32_t now = millis();
    bool heartbeat = (now - lastTxTime >= HEARTBEAT_INTERVAL_MS);

    if (analogChanged || digitalChanged || minLimitChanged || maxLimitChanged || heartbeat) {
        // Construct binary payload
        ServoMeshMessage msg;
        msg.magic = MESSAGE_MAGIC;
        msg.sender_id = MY_NODE_ID;
        msg.target_id = TARGET_NODE_ID;
        msg.sensor_value = currentAnalog;
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

        // Update last state tracking
        lastAnalogVal = currentAnalog;
        lastDigitalVal = currentDigital;
        lastMinLimit = currentMinLimit;
        lastMaxLimit = currentMaxLimit;
        lastTxTime = now;

        Serial.printf("[TX #%u] Analog: %u | Digital: %u | MinLim: %u | MaxLim: %u -> Target Node: %u (Hex: %s)\n",
                      msg.seq, currentAnalog, currentDigital, currentMinLimit, currentMaxLimit, TARGET_NODE_ID, hexBuffer);
    }
}

/**
 * Calculates and updates local servo angle while respecting local limit switch clamping.
 */
void updateLocalServoAngle(uint16_t requestedAnalogVal) {
    currentTargetAnalogVal = requestedAnalogVal;

    // Map analog sensor range (0-1023) to servo angle (SERVO_MIN_ANGLE - SERVO_MAX_ANGLE)
    int targetAngle = map((int)requestedAnalogVal, 0, 1023, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    targetAngle = constrain(targetAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    // Read current local limit switches
    bool minLimitActive = (digitalRead(MIN_LIMIT_PIN) == LOW);
    bool maxLimitActive = (digitalRead(MAX_LIMIT_PIN) == LOW);

    int finalAngle = targetAngle;

    // Clamp servo movement if limit switches are triggered
    if (minLimitActive && finalAngle < (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) / 2) {
        finalAngle = SERVO_MIN_ANGLE;
    }
    if (maxLimitActive && finalAngle > (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) / 2) {
        finalAngle = SERVO_MAX_ANGLE;
    }

    myServo.write(finalAngle);

    Serial.printf("[SERVO] Requested Analog: %u -> Target Angle: %d deg | Clamped Angle: %d deg (MinLim: %d, MaxLim: %d)\n",
                  requestedAnalogVal, targetAngle, finalAngle, minLimitActive, maxLimitActive);
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

        // Update local servo angle with received analog position (respecting local limit clamping)
        updateLocalServoAngle(incoming.sensor_value);

        Serial.printf("[RX #%u] From Node: %u | Sensor: %u | Digital: %u | Remote MinLim: %u | Remote MaxLim: %u\n",
                      incoming.seq, incoming.sender_id, incoming.sensor_value, incoming.digital_value,
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
