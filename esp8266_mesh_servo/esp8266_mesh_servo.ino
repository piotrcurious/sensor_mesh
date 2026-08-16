/*
  ESP8266 Bi-directional Servo & Sensor Mesh Node Firmware (Session-Incarnation & Unicast Control)
  Uses painlessMesh to create an auto-organizing mesh network.

  Session Incarnation & Sequence Features:
  - Boot Session Incarnation Token (mySessionId):
    * Generated upon boot via (ESP.getChipId() ^ micros()).
    * Included in both Handshake (HELLO/HELLO_ACK) and Data (ServoMeshMessage) frames.
    * When a new session_id is received from TARGET_NODE_ID, sequence state (lastReceivedSeq)
      is automatically reset and re-synchronized.
  - CONTROL Traffic vs DISCOVERY Traffic Separation:
    * CONTROL / Sensor payloads are sent EXCLUSIVELY via targeted unicast (mesh.sendSingle).
    * Broadcasts are restricted solely to infrequent HELLO discovery packets during DISCOVERING state.
  - PeerState Enum: UNKNOWN, DISCOVERING, CONNECTED.
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
static uint32_t targetSessionId = 0;  // Active boot session ID of TARGET_NODE_ID

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
 * Resets target sequence state for a new session incarnation.
 */
static void resetSessionSequence(uint32_t newSessionId) {
    targetSessionId = newSessionId;
    lastReceivedSeq = 0;
    hasReceivedFirstPacket = false;
    Serial.printf("[SESSION] Established/Reset Target Session ID: %u (Seq Reset to 0)\n", newSessionId);
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

    // Generate unique session incarnation token for this boot
    mySessionId = ESP.getChipId() ^ micros() ^ (uint32_t)random(0xFFFF);

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("ESP8266 Bi-directional Servo Mesh Node (Session Incarnation Aware)\n");
    Serial.printf("My Node ID: %u (Session: %u) -> Target Node ID: %u\n", MY_NODE_ID, mySessionId, TARGET_NODE_ID);
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

    HandshakeMessage helloMsg;
    helloMsg.magic = MSG_TYPE_HELLO;
    helloMsg.sender_id = MY_NODE_ID;
    helloMsg.target_id = TARGET_NODE_ID;
    helloMsg.session_id = mySessionId;
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

    Serial.printf("[DISCOVERY #%u] Sent HELLO broadcast for Target Node %u (Session: %u)\n",
                  helloMsg.seq, TARGET_NODE_ID, mySessionId);
}

/**
 * Sends a targeted HELLO_ACK response in reply to a received HELLO.
 */
void sendHelloAck(uint32_t destMeshId) {
    HandshakeMessage ackMsg;
    ackMsg.magic = MSG_TYPE_HELLO_ACK;
    ackMsg.sender_id = MY_NODE_ID;
    ackMsg.target_id = TARGET_NODE_ID;
    ackMsg.session_id = mySessionId;
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

    Serial.printf("[DISCOVERY #%u] Sent HELLO_ACK to Target Node %u (MeshID: %u, Session: %u)\n",
                  ackMsg.seq, TARGET_NODE_ID, destMeshId, mySessionId);
}

/**
 * Polls inputs and transmits CONTROL payload data EXCLUSIVELY via targeted UNICAST (sendSingle).
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

    // Suppress CONTROL packet transmission unless CONNECTED to target node
    if (peerState != PeerState::CONNECTED || targetMeshNodeId == 0 || !mesh.isConnected(targetMeshNodeId)) {
        return;
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
        msg.magic = MSG_TYPE_DATA;
        msg.sender_id = MY_NODE_ID;
        msg.target_id = TARGET_NODE_ID;
        msg.session_id = mySessionId;
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

        // Strict Unicast CONTROL Transmission
        bool sentDirect = mesh.sendSingle(targetMeshNodeId, payload);

        lastTransmittedUsFp4 = currentUsFp4;
        lastDigitalVal = currentDigital;
        lastMinLimit = currentMinLimit;
        lastMaxLimit = currentMaxLimit;
        lastTxTime = now;

        Serial.printf("[TX #%u] Filtered ADC: %.2f | Target Pulse: %.2f us (FP4: %u) | Unicast Sent: %s (Session: %u)\n",
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
 * Handles HELLO / HELLO_ACK discovery and DATA messages with session incarnation tracking.
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
    if (msg.length() != sizeof(ServoMeshMessage) * 2) {
        return;
    }

    ServoMeshMessage incoming;
    uint8_t* rawBytes = (uint8_t*)&incoming;

    for (size_t i = 0; i < sizeof(ServoMeshMessage); i++) {
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

    // Auto-discover / verify target mesh ID & check session incarnation change
    targetMeshNodeId = from;
    if (peerState != PeerState::CONNECTED || incoming.session_id != targetSessionId) {
        peerState = PeerState::CONNECTED;
        resetSessionSequence(incoming.session_id);
        Serial.printf("[STATE] Synchronized Target MeshID %u with new Session ID %u. Transitioned to CONNECTED.\n",
                      from, incoming.session_id);
    }

    // Single-source sequence check for active session
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
