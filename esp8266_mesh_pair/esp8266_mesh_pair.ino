/*
  ESP8266 Bi-directional Sensor Mesh Node Firmware
  Uses painlessMesh to create an auto-organizing mesh network.
  Reads analog input pin (A0) every 1 second and transmits compact binary packed struct data
  (HEX encoded to safely pass through JSON mesh frames) to a paired node
  (configured via MY_NODE_ID and TARGET_NODE_ID).
  When receiving messages addressed to MY_NODE_ID, sets PWM output pin.
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

// Task to read sensor and transmit data every 1 second (SEND_INTERVAL_MS)
Task taskSendSensorData(SEND_INTERVAL_MS, TASK_FOREVER, &sendSensorData);

// Sequence counter
static uint32_t messageSequence = 0;

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
    Serial.printf("ESP8266 Bi-directional Sensor Mesh Node\n");
    Serial.printf("My Node ID: %u -> Target Node ID: %u\n", MY_NODE_ID, TARGET_NODE_ID);
    Serial.printf("PWM Pin: GPIO %d | Sensor Pin: A0\n", PWM_PIN);
    Serial.println("==================================================");

    // Initialize hardware pins
    pinMode(SENSOR_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0); // Start PWM at 0% duty cycle

    // Enable painlessMesh debug messages (optional)
    // mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION);

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
 * Reads A0 analog input, packs into compact binary struct, hex-encodes it, and broadcasts over mesh.
 */
void sendSensorData() {
    // Read analog pin (0 - 1023)
    uint16_t rawSensorVal = analogRead(SENSOR_PIN);

    // Construct binary payload
    SensorMessage msg;
    msg.magic = MESSAGE_MAGIC;
    msg.sender_id = MY_NODE_ID;
    msg.target_id = TARGET_NODE_ID;
    msg.sensor_value = rawSensorVal;
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

    Serial.printf("[TX #%u] Sent Sensor Value: %u (Raw A0) -> Target Node: %u (Struct: %d bytes, Hex Payload: %s)\n",
                  msg.seq, rawSensorVal, TARGET_NODE_ID, (int)structSize, hexBuffer);
}

/**
 * Callback when a mesh message is received.
 */
void receivedCallback(uint32_t from, String &msg) {
    const size_t expectedStructSize = sizeof(SensorMessage);
    const size_t expectedHexLen = expectedStructSize * 2;

    // Validate string length matches expected HEX payload size
    if (msg.length() != expectedHexLen) {
        // Ignore messages of different size
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

        // Drive the PWM pin
        analogWrite(PWM_PIN, pwmValue);

        Serial.printf("[RX #%u] From Node: %u | Sensor Data: %u -> Set PWM Duty: %u/%d (Mesh NodeID: %u)\n",
                      incoming.seq, incoming.sender_id, incoming.sensor_value, pwmValue, PWM_RANGE, from);
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
