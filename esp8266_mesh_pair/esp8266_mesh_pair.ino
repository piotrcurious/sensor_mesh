/*
  ESP8266 Bi-directional Sensor Mesh Node Firmware (Traffic Separated)
  Uses painlessMesh to create an auto-organizing mesh network.

  Traffic Separation & Discovery Features:
  - CONTROL Traffic vs DISCOVERY Traffic Separation:
    * CONTROL / Sensor payloads are sent EXCLUSIVELY via targeted unicast (mesh.sendSingle).
    * Broadcasts are restricted solely to infrequent HELLO discovery packets during DISCOVERING state.
    * Eliminates broadcast network congestion / flooding on disconnected or multi-node mesh networks.
  - PeerState Enum: UNKNOWN, DISCOVERING, CONNECTED.
  - Active Handshake Discovery: sends HELLO broadcast packets independently of sensor motion.
  - Dynamic Topology Invalidation: on changedConnectionCallback(), checks if targetMeshNodeId
    remains connected in mesh.getNodeList(). If lost, state resets to DISCOVERING.
  - Paired Sender Validation & Sequence Verification.
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
void sendHelloDiscovery();
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
float readAnalogFiltered();
bool isNewerSequence(uint32_t incoming, uint32_t last);

// Task to read sensor and transmit data every 1 second (SEND_INTERVAL_MS)
Task taskSendSensorData(SEND_INTERVAL_MS, TASK_FOREVER, &sendSensorData);

// Discovery task (1000 ms retry while DISCOVERING)
Task taskDiscovery(DISCOVERY_INTERVAL_MS, TASK_FOREVER, &sendHelloDiscovery);

// Peer Discovery State Machine
static PeerState peerState = PeerState::UNKNOWN;
static uint32_t targetMeshNodeId = 0; // Discovered painlessMesh uint32_t node ID for TARGET_NODE_ID

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// Sequence tracking (Isolated to TARGET_NODE_ID)
static uint32_t messageSequence = 0;
static uint32_t discoverySequence = 0;
static uint32_t lastReceivedSeq = 0;
static bool     hasReceivedFirstPacket = false;

// Helper: Convert uint8_t hex character to byte
static uint8_t hexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

/**
 * Wraparound-safe 32-bit sequence comparison.
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
    Serial.printf("ESP8266 Bi-directional Sensor Mesh Node (Strict Unicast Control)\n");
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

    // Start in DISCOVERING state
    peerState = PeerState::DISCOVERING;

    // Enable Tasks
    userScheduler.addTask(taskSendSensorData);
    taskSendSensorData.enable();

    userScheduler.addTask(taskDiscovery);
    taskDiscovery.enable();
}

void loop() {
    mesh.update();
}

/**
 * Periodically broadcasts HELLO handshake packets ONLY when state is DISCOVERING / UNKNOWN.
 * Isolates broadcast traffic to infrequent discovery intervals.
 */
void sendHelloDiscovery() {
    if (peerState == PeerState::CONNECTED) {
        return;
    }

    HandshakeMessage helloMsg;
    helloMsg.magic = MSG_TYPE_HELLO;
    helloMsg.sender_id = MY_NODE_ID;
    helloMsg.target_id = TARGET_NODE_ID;
    helloMsg.seq = ++discoverySequence;

    const size_t structSize = sizeof(HandshakeMessage);
    const size_t hexLen = structSize * 2;
    char hexBuffer[hexLen + 1];
    const uint8_t* rawBytes = (const uint8_t*)&helloMsg;

    for (size_t i = 0; i < structSize; i++) {
        sprintf(&hexBuffer[i * 2], "%02X", rawBytes[i]);
    }
    hexBuffer[hexLen] = '\0';

    mesh.sendBroadcast(String(hexBuffer));

    Serial.printf("[DISCOVERY #%u] Sent HELLO broadcast for Target Node %u (State: DISCOVERING)\n",
                  helloMsg.seq, TARGET_NODE_ID);
}

/**
 * Sends a targeted HELLO_ACK response in reply to a received HELLO.
 */
void sendHelloAck(uint32_t destMeshId) {
    HandshakeMessage ackMsg;
    ackMsg.magic = MSG_TYPE_HELLO_ACK;
    ackMsg.sender_id = MY_NODE_ID;
    ackMsg.target_id = TARGET_NODE_ID;
    ackMsg.seq = ++discoverySequence;

    const size_t structSize = sizeof(HandshakeMessage);
    const size_t hexLen = structSize * 2;
    char hexBuffer[hexLen + 1];
    const uint8_t* rawBytes = (const uint8_t*)&ackMsg;

    for (size_t i = 0; i < structSize; i++) {
        sprintf(&hexBuffer[i * 2], "%02X", rawBytes[i]);
    }
    hexBuffer[hexLen] = '\0';

    String payload(hexBuffer);
    mesh.sendSingle(destMeshId, payload);

    Serial.printf("[DISCOVERY #%u] Sent HELLO_ACK to Target Node %u (MeshID: %u)\n",
                  ackMsg.seq, TARGET_NODE_ID, destMeshId);
}

/**
 * Reads DSP-filtered analog input and digital input, packs into binary struct,
 * hex-encodes it, and sends CONTROL payload EXCLUSIVELY via targeted UNICAST (sendSingle).
 */
void sendSensorData() {
    float filteredVal = readAnalogFiltered();
    uint16_t highResSensorVal = (uint16_t)constrain((int)roundf(filteredVal), 0, 1023);

    uint8_t digitalVal = digitalRead(DIGITAL_INPUT_PIN);

    // Suppress CONTROL packet transmission unless CONNECTED to target node
    if (peerState != PeerState::CONNECTED || targetMeshNodeId == 0 || !mesh.isConnected(targetMeshNodeId)) {
        return;
    }

    SensorMessage msg;
    msg.magic = MSG_TYPE_DATA;
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

    // Strict Unicast CONTROL Transmission
    bool sentDirect = mesh.sendSingle(targetMeshNodeId, payload);

    Serial.printf("[TX #%u] Filtered ADC: %.2f (Val: %u) | Digital D2: %u | Unicast Sent: %s (Dest MeshID: %u)\n",
                  msg.seq, filteredVal, highResSensorVal, digitalVal,
                  sentDirect ? "SUCCESS" : "FAILED", targetMeshNodeId);
}

/**
 * Callback when a mesh message is received.
 * Handles HELLO / HELLO_ACK discovery and DATA messages.
 */
void receivedCallback(uint32_t from, String &msg) {
    // 1. Check for Handshake Messages (HELLO / HELLO_ACK)
    if (msg.length() == sizeof(HandshakeMessage) * 2) {
        HandshakeMessage handshake;
        uint8_t* rawBytes = (uint8_t*)&handshake;
        for (size_t i = 0; i < sizeof(HandshakeMessage); i++) {
            char highNibble = msg.charAt(i * 2);
            char lowNibble = msg.charAt(i * 2 + 1);
            rawBytes[i] = (hexCharToNibble(highNibble) << 4) | hexCharToNibble(lowNibble);
        }

        if (handshake.sender_id == TARGET_NODE_ID && handshake.target_id == MY_NODE_ID) {
            if (handshake.magic == MSG_TYPE_HELLO) {
                targetMeshNodeId = from;
                peerState = PeerState::CONNECTED;
                Serial.printf("[HANDSHAKE] Received HELLO from Target Node %u (MeshID: %u). Transitioned to CONNECTED.\n",
                              TARGET_NODE_ID, from);
                sendHelloAck(from);
            } else if (handshake.magic == MSG_TYPE_HELLO_ACK) {
                targetMeshNodeId = from;
                peerState = PeerState::CONNECTED;
                Serial.printf("[HANDSHAKE] Received HELLO_ACK from Target Node %u (MeshID: %u). Transitioned to CONNECTED.\n",
                              TARGET_NODE_ID, from);
            }
        }
        return;
    }

    // 2. Check for Data Payload Messages
    if (msg.length() != sizeof(SensorMessage) * 2) {
        return;
    }

    SensorMessage incoming;
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < sizeof(SensorMessage); i++) {
        char highNibble = msg.charAt(i * 2);
        char lowNibble = msg.charAt(i * 2 + 1);
        rawBytes[i] = (hexCharToNibble(highNibble) << 4) | hexCharToNibble(lowNibble);
    }

    if (incoming.magic != MSG_TYPE_DATA) {
        return;
    }

    // Explicit Sender & Target Validation
    if (incoming.sender_id != TARGET_NODE_ID || incoming.target_id != MY_NODE_ID) {
        return;
    }

    // Auto-discover / verify target mesh ID
    targetMeshNodeId = from;
    if (peerState != PeerState::CONNECTED) {
        peerState = PeerState::CONNECTED;
        Serial.printf("[STATE] Learned Target MeshID %u from DATA payload. Transitioned to CONNECTED.\n", from);
    }

    // Single-source sequence check
    if (!isNewerSequence(incoming.seq, lastReceivedSeq)) {
        Serial.printf("[RX DROP #%u] Out-of-order or duplicate packet dropped (Last Seq: %u, From Sender: %u, MeshID: %u)\n",
                      incoming.seq, lastReceivedSeq, incoming.sender_id, from);
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
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("[MESH] New Connection, nodeId = %u (Local Mesh Node ID = %u)\n", nodeId, mesh.getNodeId());
}

/**
 * Handles topology changes: verifies if targetMeshNodeId is still present in current node list.
 * If disconnected, resets targetMeshNodeId and transitions peerState to DISCOVERING.
 */
void changedConnectionCallback() {
    Serial.printf("[MESH] Topology changed (Local Mesh Node ID = %u)\n", mesh.getNodeId());

    if (targetMeshNodeId != 0) {
        SimpleList<uint32_t> nodes = mesh.getNodeList();
        bool stillConnected = false;
        SimpleList<uint32_t>::iterator node = nodes.begin();
        while (node != nodes.end()) {
            if (*node == targetMeshNodeId) {
                stillConnected = true;
                break;
            }
            node++;
        }

        if (!stillConnected) {
            Serial.printf("[MESH] Target MeshID %u disconnected. Clearing peer state to DISCOVERING.\n", targetMeshNodeId);
            targetMeshNodeId = 0;
            peerState = PeerState::DISCOVERING;
            hasReceivedFirstPacket = false;
        }
    }
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("[MESH] System time adjusted: %d us\n", offset);
}
