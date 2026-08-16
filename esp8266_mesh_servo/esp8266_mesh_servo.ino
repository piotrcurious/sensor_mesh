/*
  ESP8266 Bi-directional Servo & Sensor Mesh Node Firmware (Handshake-Only Sessions & Heap Optimized)
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
  - Sub-microsecond Fixed-Point Precision (FP4 = 1/16th us resolution).
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
void sendHelloDiscovery();
void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void updateLocalServoFp4(uint16_t requestedUsFp4);
float readAnalogFiltered();
bool isNewerSequence(uint32_t incoming, uint32_t last);

// Input polling task (50 ms)
Task taskPollInputs(POLL_INTERVAL_MS, TASK_FOREVER, &checkAndTransmitInputs);

// Discovery task (1000 ms retry while DISCOVERING)
Task taskDiscovery(DISCOVERY_INTERVAL_MS, TASK_FOREVER, &sendHelloDiscovery);

// Local Boot Session Incarnation ID
static uint32_t mySessionId = 0;

// Peer Discovery State Machine & Remote Session Tracking
static PeerState peerState = PeerState::UNKNOWN;
static uint32_t targetMeshNodeId = 0; // Discovered painlessMesh uint32_t node ID for TARGET_NODE_ID
static uint32_t targetSessionId = 0;  // Active boot session ID of TARGET_NODE_ID installed via handshake

// Kalman Filter State
static float kalman_x = 512.0f;
static float kalman_p = 1.0f;

// State tracking for change detection & network rate limiting
static uint16_t lastTransmittedUsFp4 = 0xFFFF;
static uint8_t  lastDigitalVal = 0xFF;
static uint8_t  lastMinLimit = 0xFF;
static uint8_t  lastMaxLimit = 0xFF;
static uint32_t lastTxTime = 0;
static uint32_t messageSequence = 0;
static uint32_t discoverySequence = 0;

// Sequence verification (Isolated to TARGET_NODE_ID and active targetSessionId)
static uint32_t lastReceivedSeq = 0;
static bool     hasReceivedFirstPacket = false;

// Motion control state separation in fixed-point 1/16th microseconds (FP4)
static uint16_t requestedServoUsFp4 = 23552; // 1472 us * 16
static uint16_t appliedServoUsFp4   = 23552;
static uint16_t lastSafeUsFp4       = 23552;

// Reusable static character buffer & static reserved String payload to prevent heap fragmentation
static char staticHexTxBuffer[SERVO_WIRE_HEX_LEN + 1];
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
    txPayloadString.reserve(SERVO_WIRE_HEX_LEN + 1);

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("ESP8266 Bi-directional Servo Mesh Node (Handshake-Only Sessions)\n");
    Serial.printf("My Node ID: %u (Session: %u) -> Target Node ID: %u\n", MY_NODE_ID, mySessionId, TARGET_NODE_ID);
    Serial.printf("Analog In: A0 (Adaptive Kahan+Kalman) | Servo Pin: GPIO %d (D1) [%d - %d us]\n",
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

    // Start in DISCOVERING state
    peerState = PeerState::DISCOVERING;

    // Enable Tasks
    userScheduler.addTask(taskPollInputs);
    taskPollInputs.enable();

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
        return; // Handshake complete, discovery task idle
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
    mesh.sendSingle(destMeshId, txPayloadString);

    Serial.printf("[DISCOVERY #%u] Sent targeted HELLO_ACK to Target Node %u (MeshID: %u, Session: %u)\n",
                  ackMsg.seq, TARGET_NODE_ID, destMeshId, mySessionId);
}

/**
 * Polls inputs at 50 ms intervals.
 * Updates local safety clamping immediately without waiting for network timers.
 * Transmits CONTROL payloads EXCLUSIVELY via targeted UNICAST (sendSingle) with a strict MIN_TX_INTERVAL_MS rate limit.
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

    // Local limit switch reaction: update local safety clamping immediately on poll
    static uint8_t lastLocalMinLimit = 0xFF;
    static uint8_t lastLocalMaxLimit = 0xFF;
    if (currentMinLimit != lastLocalMinLimit || currentMaxLimit != lastLocalMaxLimit) {
        lastLocalMinLimit = currentMinLimit;
        lastLocalMaxLimit = currentMaxLimit;
        updateLocalServoFp4(requestedServoUsFp4);
    }

    // Suppress CONTROL packet transmission unless CONNECTED to target node
    if (peerState != PeerState::CONNECTED || targetMeshNodeId == 0 || !mesh.isConnected(targetMeshNodeId)) {
        return;
    }

    // STRICT NETWORK RATE-LIMITING GATE:
    uint32_t now = millis();
    if (lastTxTime != 0 && (now - lastTxTime < MIN_TX_INTERVAL_MS)) {
        return;
    }

    // Compare current sampled state against LAST TRANSMITTED state
    bool pulseChanged = (abs((int)currentUsFp4 - (int)lastTransmittedUsFp4) >= PULSE_FP4_CHANGE_THRESHOLD);
    bool digitalChanged = (currentDigital != lastDigitalVal);
    bool minLimitChanged = (currentMinLimit != lastMinLimit);
    bool maxLimitChanged = (currentMaxLimit != lastMaxLimit);
    bool stateChanged = (pulseChanged || digitalChanged || minLimitChanged || maxLimitChanged);

    bool heartbeatElapsed = (now - lastTxTime >= HEARTBEAT_INTERVAL_MS);

    if (stateChanged || heartbeatElapsed) {
        ServoMeshMessage msg{}; // Zero-initialized struct
        msg.magic = MSG_TYPE_DATA;
        msg.sender_id = MY_NODE_ID;
        msg.target_id = TARGET_NODE_ID;
        msg.session_id = mySessionId;
        msg.target_us_fp4 = currentUsFp4;
        msg.digital_value = currentDigital;
        msg.min_limit_active = currentMinLimit;
        msg.max_limit_active = currentMaxLimit;
        msg.seq = ++messageSequence;

        const uint8_t* rawBytes = (const uint8_t*)&msg;

        for (size_t i = 0; i < sizeof(ServoMeshMessage); i++) {
            sprintf(&staticHexTxBuffer[i * 2], "%02X", rawBytes[i]);
        }
        staticHexTxBuffer[SERVO_WIRE_HEX_LEN] = '\0';

        lastTxTime = now;

        txPayloadString = staticHexTxBuffer;

        // Strict Unicast CONTROL Transmission
        bool sentDirect = mesh.sendSingle(targetMeshNodeId, txPayloadString);

        if (sentDirect) {
            lastTransmittedUsFp4 = currentUsFp4;
            lastDigitalVal = currentDigital;
            lastMinLimit = currentMinLimit;
            lastMaxLimit = currentMaxLimit;
        }

        Serial.printf("[TX #%u] Filtered ADC: %.2f | Target Pulse: %.2f us (FP4: %u) | Unicast Queued: %s (Session: %u)\n",
                      msg.seq, filteredAdc, (float)currentUsFp4 / 16.0f, currentUsFp4,
                      sentDirect ? "SUCCESS" : "FAILED", mySessionId);
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
 * Strict Session Integrity: DATA messages NEVER establish sessions. Handshakes installed sessions exclusively.
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
    if (msg.length() != SERVO_WIRE_HEX_LEN) {
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

    ServoMeshMessage incoming{};
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < sizeof(ServoMeshMessage); i++) {
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

    // Update local digital output (D3)
    digitalWrite(DIGITAL_OUTPUT_PIN, incoming.digital_value ? HIGH : LOW);

    // Update local servo
    updateLocalServoFp4(incoming.target_us_fp4);

    Serial.printf("[RX #%u] From Node: %u (Session: %u) | Target Pulse: %.2f us (%u FP4) | Digital: %u | MinLim: %u | MaxLim: %u\n",
                  incoming.seq, incoming.sender_id, incoming.session_id, (float)incoming.target_us_fp4 / 16.0f,
                  incoming.target_us_fp4, incoming.digital_value, incoming.min_limit_active, incoming.max_limit_active);
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
