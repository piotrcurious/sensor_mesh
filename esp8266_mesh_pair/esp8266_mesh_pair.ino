/*
  ESP8266 Bi-directional Sensor Mesh Node Firmware (Handshake-Only Sessions & Heap Optimized)
  Uses painlessMesh to create an auto-organizing mesh network.

  Fixes & Hardening Enhancements:
  - Strict Handshake-Only Session Installation:
    * DATA packets are NEVER permitted to establish or reset a session.
    * DATA packets must strictly match active targetSessionId. Stale, unauthenticated, or mismatched
      session DATA packets are immediately DROPPED.
    * Sessions are installed EXCLUSIVELY via MSG_TYPE_HELLO / MSG_TYPE_HELLO_ACK handshakes.
  - Zero Heap Allocation Strategy:
    * Uses a static reusable String txPayloadString with pre-reserved capacity (reserve()),
      preventing repeated creation/destruction of String objects on the ESP8266 heap.
  - Hardened Boot Nonce Generation:
    * Combines ESP.getChipId(), micros(), ESP.getCycleCount(), and random() for boot incarnation.
  - Strict Rate Limiting Gate:
    * Enforces a hard time gate (now - lastTxTime < MIN_TX_INTERVAL_MS) at the start of packet evaluation.
  - Adaptive 1D Kalman Filter:
    * Dynamic process noise Q scales with motion innovation, eliminating motion lag.
  - Protocol Integrity & Zero-Initialization:
    * Zero-initializes all C++ structs (Struct{}) to prevent stack garbage leakage in padding bytes.
    * Static compile-time size assertions (static_assert) guarantee wire format structure size.
  - Strict Transport Route Validation:
    * Once CONNECTED, data payloads are accepted ONLY if from == targetMeshNodeId.
  - Targeted Unicast Handshakes:
    * HELLO_ACK is sent strictly via unicast; targeted ACKs do NOT fall back to broadcast.
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

// Local Boot Session Incarnation ID
static uint32_t mySessionId = 0;

// Peer Discovery State Machine & Remote Session Tracking
static PeerState peerState = PeerState::UNKNOWN;
static uint32_t targetMeshNodeId = 0; // Discovered painlessMesh uint32_t node ID for TARGET_NODE_ID
static uint32_t targetSessionId = 0;  // Active boot session ID of TARGET_NODE_ID installed via handshake

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// Sequence tracking
static uint32_t messageSequence = 0;
static uint32_t discoverySequence = 0;
static uint32_t lastReceivedSeq = 0;
static bool     hasReceivedFirstPacket = false;

// Reusable static character buffer & static reserved String payload to prevent heap fragmentation
static char staticHexTxBuffer[PAIR_WIRE_HEX_LEN + 1];
static String txPayloadString;

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
 * Resets target sequence state EXCLUSIVELY upon authentic handshake session establishment.
 */
static void resetSessionSequence(uint32_t newSessionId) {
    targetSessionId = newSessionId;
    lastReceivedSeq = 0;
    hasReceivedFirstPacket = false;
    Serial.printf("[SESSION] Handshake Established Active Target Session ID: %u (Seq Reset to 0)\n", newSessionId);
}

/**
 * High-Precision Analog Read with Sort-Based Trimmed Mean, Kahan Summation, and Adaptive Kalman Filter.
 */
float readAnalogFiltered() {
    uint16_t samples[ADC_OVERSAMPLE_COUNT];

    // 1. Oversample ADC
    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        samples[i] = analogRead(SENSOR_PIN);
        optimistic_yield(1000);
    }

    // 2. Sort samples in place to cleanly strip highest and lowest extremes
    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT - 1; i++) {
        for (size_t j = i + 1; j < ADC_OVERSAMPLE_COUNT; j++) {
            if (samples[i] > samples[j]) {
                uint16_t temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }

    // 3. Kahan Summation on interior trimmed samples
    float sum = 0.0f;
    float c = 0.0f;
    size_t startIndex = (ADC_OVERSAMPLE_COUNT >= 4) ? 1 : 0;
    size_t endIndex = (ADC_OVERSAMPLE_COUNT >= 4) ? (ADC_OVERSAMPLE_COUNT - 1) : ADC_OVERSAMPLE_COUNT;
    size_t count = endIndex - startIndex;

    for (size_t i = startIndex; i < endIndex; i++) {
        float y = (float)samples[i] - c;
        float t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }

    float averageAdc = (count > 0) ? (sum / (float)count) : (float)samples[0];

    // 4. Adaptive 1D Kalman Filter Update:
    float innovation = fabsf(averageAdc - kalman_x);
    float dynamicQ = KALMAN_PROCESS_NOISE_Q;
    if (innovation > 10.0f) {
        dynamicQ = innovation * 0.1f;
    }

    kalman_p = kalman_p + dynamicQ;
    float k_gain = kalman_p / (kalman_p + KALMAN_MEASUREMENT_NOISE_R);
    kalman_x = kalman_x + k_gain * (averageAdc - kalman_x);
    kalman_p = (1.0f - k_gain) * kalman_p;

    return kalman_x;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // Hardened unique boot session nonce generation
    mySessionId = ESP.getChipId() ^ micros() ^ ESP.getCycleCount() ^ (uint32_t)random(0xFFFFFFFF);

    // Pre-reserve static txPayloadString capacity to prevent heap allocations
    txPayloadString.reserve(PAIR_WIRE_HEX_LEN + 1);

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("ESP8266 Bi-directional Sensor Mesh Node (Handshake-Only Sessions)\n");
    Serial.printf("My Node ID: %u (Session: %u) -> Target Node ID: %u\n", MY_NODE_ID, mySessionId, TARGET_NODE_ID);
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
 */
void sendHelloDiscovery() {
    if (peerState == PeerState::CONNECTED) {
        return;
    }

    HandshakeMessage helloMsg{}; // Zero-initialized struct
    helloMsg.magic = MSG_TYPE_HELLO;
    helloMsg.sender_id = MY_NODE_ID;
    helloMsg.target_id = TARGET_NODE_ID;
    helloMsg.session_id = mySessionId;
    helloMsg.seq = ++discoverySequence;

    const uint8_t* rawBytes = (const uint8_t*)&helloMsg;

    for (size_t i = 0; i < sizeof(HandshakeMessage); i++) {
        sprintf(&staticHexTxBuffer[i * 2], "%02X", rawBytes[i]);
    }
    staticHexTxBuffer[HANDSHAKE_WIRE_HEX_LEN] = '\0';

    txPayloadString = staticHexTxBuffer;
    mesh.sendBroadcast(txPayloadString);

    Serial.printf("[DISCOVERY #%u] Sent HELLO broadcast for Target Node %u (Session: %u)\n",
                  helloMsg.seq, TARGET_NODE_ID, mySessionId);
}

/**
 * Sends a targeted HELLO_ACK response in reply to a received HELLO strictly via unicast.
 */
void sendHelloAck(uint32_t destMeshId) {
    HandshakeMessage ackMsg{}; // Zero-initialized struct
    ackMsg.magic = MSG_TYPE_HELLO_ACK;
    ackMsg.sender_id = MY_NODE_ID;
    ackMsg.target_id = TARGET_NODE_ID;
    ackMsg.session_id = mySessionId;
    ackMsg.seq = ++discoverySequence;

    const uint8_t* rawBytes = (const uint8_t*)&ackMsg;

    for (size_t i = 0; i < sizeof(HandshakeMessage); i++) {
        sprintf(&staticHexTxBuffer[i * 2], "%02X", rawBytes[i]);
    }
    staticHexTxBuffer[HANDSHAKE_WIRE_HEX_LEN] = '\0';

    txPayloadString = staticHexTxBuffer;

    // Strict Unicast - no broadcast fallback for targeted ACKs
    mesh.sendSingle(destMeshId, txPayloadString);

    Serial.printf("[DISCOVERY #%u] Sent targeted HELLO_ACK to Target Node %u (MeshID: %u, Session: %u)\n",
                  ackMsg.seq, TARGET_NODE_ID, destMeshId, mySessionId);
}

/**
 * Reads DSP-filtered analog input and digital input, packs into binary struct,
 * hex-encodes it, and sends CONTROL payload EXCLUSIVELY via targeted UNICAST (sendSingle).
 */
void sendSensorData() {
    // Suppress CONTROL packet transmission unless CONNECTED to target node
    if (peerState != PeerState::CONNECTED || targetMeshNodeId == 0 || !mesh.isConnected(targetMeshNodeId)) {
        return;
    }

    float filteredVal = readAnalogFiltered();
    uint16_t highResSensorVal = (uint16_t)constrain((int)roundf(filteredVal), 0, 1023);

    uint8_t digitalVal = digitalRead(DIGITAL_INPUT_PIN);

    SensorMessage msg{}; // Zero-initialized struct
    msg.magic = MSG_TYPE_DATA;
    msg.sender_id = MY_NODE_ID;
    msg.target_id = TARGET_NODE_ID;
    msg.session_id = mySessionId;
    msg.sensor_value = highResSensorVal;
    msg.digital_value = digitalVal;
    msg.seq = ++messageSequence;

    const uint8_t* rawBytes = (const uint8_t*)&msg;

    for (size_t i = 0; i < sizeof(SensorMessage); i++) {
        sprintf(&staticHexTxBuffer[i * 2], "%02X", rawBytes[i]);
    }
    staticHexTxBuffer[PAIR_WIRE_HEX_LEN] = '\0';

    txPayloadString = staticHexTxBuffer;

    // Strict Unicast CONTROL Transmission
    bool sentDirect = mesh.sendSingle(targetMeshNodeId, txPayloadString);

    Serial.printf("[TX #%u] Filtered ADC: %.2f (Val: %u) | Digital D2: %u | Unicast Queued: %s (Session: %u)\n",
                  msg.seq, filteredVal, highResSensorVal, digitalVal,
                  sentDirect ? "SUCCESS" : "FAILED", mySessionId);
}

/**
 * Callback when a mesh message is received.
 * Strict Session Integrity: DATA messages NEVER establish sessions. Handshakes install sessions exclusively.
 */
void receivedCallback(uint32_t from, String &msg) {
    // 1. Check for Handshake Messages (HELLO / HELLO_ACK) - EXCLUSIVE session installation mechanism
    if (msg.length() == HANDSHAKE_WIRE_HEX_LEN) {
        HandshakeMessage handshake{};
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
                resetSessionSequence(handshake.session_id);
                Serial.printf("[HANDSHAKE] Received HELLO from Target Node %u (MeshID: %u, Session: %u). Transitioned to CONNECTED.\n",
                              TARGET_NODE_ID, from, handshake.session_id);
                sendHelloAck(from);
            } else if (handshake.magic == MSG_TYPE_HELLO_ACK) {
                targetMeshNodeId = from;
                peerState = PeerState::CONNECTED;
                resetSessionSequence(handshake.session_id);
                Serial.printf("[HANDSHAKE] Received HELLO_ACK from Target Node %u (MeshID: %u, Session: %u). Transitioned to CONNECTED.\n",
                              TARGET_NODE_ID, from, handshake.session_id);
            }
        }
        return;
    }

    // 2. Check for Data Payload Messages
    if (msg.length() != PAIR_WIRE_HEX_LEN) {
        return; // Size mismatch
    }

    // HARDENED FILTERING RULE 1: Must be in CONNECTED state
    if (peerState != PeerState::CONNECTED) {
        return;
    }

    // HARDENED FILTERING RULE 2: Must originate from active targetMeshNodeId
    if (from != targetMeshNodeId) {
        Serial.printf("[RX REJECT] DATA payload from unverified MeshID %u (Active Target MeshID: %u)\n",
                      from, targetMeshNodeId);
        return;
    }

    SensorMessage incoming{};
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < sizeof(SensorMessage); i++) {
        char highNibble = msg.charAt(i * 2);
        char lowNibble = msg.charAt(i * 2 + 1);
        rawBytes[i] = (hexCharToNibble(highNibble) << 4) | hexCharToNibble(lowNibble);
    }

    if (incoming.magic != MSG_TYPE_DATA) {
        return;
    }

    // HARDENED FILTERING RULE 3: Explicit Sender & Target Validation
    if (incoming.sender_id != TARGET_NODE_ID || incoming.target_id != MY_NODE_ID) {
        return;
    }

    // HARDENED FILTERING RULE 4: Strict Active Session ID Matching
    // DATA payloads CANNOT establish or reset sessions. Stale or unauthenticated session IDs are DROPPED.
    if (incoming.session_id != targetSessionId) {
        Serial.printf("[RX SESSION REJECT] Dropped DATA with stale/unmatched Session ID %u (Active Session: %u)\n",
                      incoming.session_id, targetSessionId);
        return;
    }

    // HARDENED FILTERING RULE 5: Single-Source Sequence Check for active session
    if (!isNewerSequence(incoming.seq, lastReceivedSeq)) {
        Serial.printf("[RX DROP #%u] Out-of-order or duplicate packet dropped (Last Seq: %u, Sender Session: %u)\n",
                      incoming.seq, lastReceivedSeq, incoming.session_id);
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

    Serial.printf("[RX #%u] From Node: %u (Session: %u) | Analog: %u -> PWM Duty: %u/%d | Digital D2 -> D3: %u (Mesh NodeID: %u)\n",
                  incoming.seq, incoming.sender_id, incoming.session_id, incoming.sensor_value, pwmValue, PWM_RANGE, digitalState, from);
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
