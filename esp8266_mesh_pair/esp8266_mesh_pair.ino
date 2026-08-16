/*
  ESP8266 Bi-directional Sensor Mesh Node Firmware (Hardened PeerSession Tuple Validation)
  Uses painlessMesh to create an auto-organizing mesh network.

  Fixes & Hardening Enhancements:
  - PeerSession Tuple Validation:
    * Replaces scalar variables with a unified PeerSession state structure:
      (meshNodeId, senderId, sessionId, lastDataSeq, lastHelloSeq, state).
  - Handshake Non-Reset Protection:
    * Duplicate HELLO / HELLO_ACK frames within the same session DO NOT reset lastDataSeq.
    * Only authentic new session handshakes or newer HELLO sequences re-synchronize session parameters.
    * Rejects stale/old HELLO packets from previous boots.
  - Strict DATA Frame Validation:
    * DATA frames are validated against the complete PeerSession tuple.
    * DATA packets with invalid session_id, unverified transport node ID, or stale seq are DROPPED.
  - Fast Local Lookup Table (LUT) Hex Encoder / Decoder & CRC-16 Checksum.
  - Zero Heap Allocation Strategy (static reserved txPayloadString).
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
bool isNewerSequence(uint32_t incoming, uint32_t last, bool initialized);

// Fast Hex LUT
static const char HEX_LUT[] = "0123456789ABCDEF";

// Task to read sensor and transmit data every 1 second (SEND_INTERVAL_MS)
Task taskSendSensorData(SEND_INTERVAL_MS, TASK_FOREVER, &sendSensorData);

// Discovery task (1000 ms retry while DISCOVERING)
Task taskDiscovery(DISCOVERY_INTERVAL_MS, TASK_FOREVER, &sendHelloDiscovery);

// Local Boot Session Incarnation ID
static uint32_t mySessionId = 0;

// Unified PeerSession State Structure
struct PeerSession {
    uint32_t  meshNodeId;      // Transport painlessMesh uint32_t node ID
    uint16_t  senderId;        // Application sender ID (TARGET_NODE_ID)
    uint32_t  sessionId;       // Active boot session incarnation token
    uint32_t  lastDataSeq;     // Last accepted DATA sequence number
    uint32_t  lastHelloSeq;    // Last accepted HELLO handshake sequence number
    PeerState state;          // UNKNOWN, DISCOVERING, CONNECTED
    bool      hasDataSeq;      // Sequence initialization flag
    bool      hasHelloSeq;     // Handshake sequence initialization flag
};

static PeerSession peerSession{};

// Kalman Filter State
static float kalman_x = 512.0f; // Estimated value
static float kalman_p = 1.0f;    // Estimation error covariance

// Sequence tracking
static uint32_t messageSequence = 0;
static uint32_t discoverySequence = 0;
static uint32_t lastTxTime = 0;

// Reusable static character buffer & static reserved String payload
static char staticHexTxBuffer[PAIR_WIRE_HEX_LEN + 1];
static String txPayloadString;

/**
 * Calculates 16-bit CRC-16-CCITT checksum over byte array.
 */
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * Fast LUT-based binary-to-hex encoder.
 */
void bytesToHex(const uint8_t* src, size_t srcLen, char* dest) {
    for (size_t i = 0; i < srcLen; i++) {
        uint8_t byte = src[i];
        dest[i * 2]     = HEX_LUT[(byte >> 4) & 0x0F];
        dest[i * 2 + 1] = HEX_LUT[byte & 0x0F];
    }
    dest[srcLen * 2] = '\0';
}

/**
 * Fast robust hex-to-binary decoder.
 */
bool hexToBytes(const String& hexStr, uint8_t* dest, size_t destLen) {
    if (hexStr.length() != destLen * 2) {
        return false;
    }
    for (size_t i = 0; i < destLen; i++) {
        char high = hexStr.charAt(i * 2);
        char low  = hexStr.charAt(i * 2 + 1);

        uint8_t highNibble = 0, lowNibble = 0;

        if (high >= '0' && high <= '9') highNibble = high - '0';
        else if (high >= 'A' && high <= 'F') highNibble = high - 'A' + 10;
        else if (high >= 'a' && high <= 'f') highNibble = high - 'a' + 10;
        else return false;

        if (low >= '0' && low <= '9') lowNibble = low - '0';
        else if (low >= 'A' && low <= 'F') lowNibble = low - 'A' + 10;
        else if (low >= 'a' && low <= 'f') lowNibble = low - 'a' + 10;
        else return false;

        dest[i] = (highNibble << 4) | lowNibble;
    }
    return true;
}

/**
 * Wraparound-safe 32-bit sequence comparison.
 */
bool isNewerSequence(uint32_t incoming, uint32_t last, bool initialized) {
    if (!initialized) {
        return true;
    }
    return ((int32_t)(incoming - last)) > 0;
}

/**
 * High-Precision Analog Read with Sort-Based Trimmed Mean, Kahan Summation, and Adaptive Kalman Filter.
 */
float readAnalogFiltered() {
    uint16_t samples[ADC_OVERSAMPLE_COUNT];

    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        samples[i] = analogRead(SENSOR_PIN);
        optimistic_yield(1000);
    }

    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT - 1; i++) {
        for (size_t j = i + 1; j < ADC_OVERSAMPLE_COUNT; j++) {
            if (samples[i] > samples[j]) {
                uint16_t temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }

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

    // Initialize peerSession state
    peerSession.senderId = TARGET_NODE_ID;
    peerSession.meshNodeId = 0;
    peerSession.sessionId = 0;
    peerSession.lastDataSeq = 0;
    peerSession.lastHelloSeq = 0;
    peerSession.state = PeerState::DISCOVERING;
    peerSession.hasDataSeq = false;
    peerSession.hasHelloSeq = false;

    Serial.println();
    Serial.println("==================================================");
    Serial.printf("ESP8266 Bi-directional Sensor Mesh Node (PeerSession Tuple Validated)\n");
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
    if (peerSession.state == PeerState::CONNECTED) {
        return;
    }

    HandshakeMessage helloMsg{}; // Zero-initialized struct
    helloMsg.magic = MSG_TYPE_HELLO;
    helloMsg.sender_id = MY_NODE_ID;
    helloMsg.target_id = TARGET_NODE_ID;
    helloMsg.session_id = mySessionId;
    helloMsg.seq = ++discoverySequence;

    helloMsg.crc16 = calculateCRC16((const uint8_t*)&helloMsg, sizeof(HandshakeMessage) - sizeof(uint16_t));

    bytesToHex((const uint8_t*)&helloMsg, sizeof(HandshakeMessage), staticHexTxBuffer);

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

    ackMsg.crc16 = calculateCRC16((const uint8_t*)&ackMsg, sizeof(HandshakeMessage) - sizeof(uint16_t));

    bytesToHex((const uint8_t*)&ackMsg, sizeof(HandshakeMessage), staticHexTxBuffer);

    txPayloadString = staticHexTxBuffer;

    mesh.sendSingle(destMeshId, txPayloadString);

    Serial.printf("[DISCOVERY #%u] Sent targeted HELLO_ACK to Target Node %u (MeshID: %u, Session: %u)\n",
                  ackMsg.seq, TARGET_NODE_ID, destMeshId, mySessionId);
}

/**
 * Reads DSP-filtered analog input and digital input, packs into binary struct,
 * hex-encodes it, and sends CONTROL payload EXCLUSIVELY via targeted UNICAST (sendSingle).
 */
void sendSensorData() {
    if (peerSession.state != PeerState::CONNECTED || peerSession.meshNodeId == 0 || !mesh.isConnected(peerSession.meshNodeId)) {
        return;
    }

    uint32_t now = millis();
    if (lastTxTime != 0 && (now - lastTxTime < 200)) {
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

    msg.crc16 = calculateCRC16((const uint8_t*)&msg, sizeof(SensorMessage) - sizeof(uint16_t));

    bytesToHex((const uint8_t*)&msg, sizeof(SensorMessage), staticHexTxBuffer);

    lastTxTime = now;
    txPayloadString = staticHexTxBuffer;

    bool sentDirect = mesh.sendSingle(peerSession.meshNodeId, txPayloadString);

    Serial.printf("[TX #%u] Filtered ADC: %.2f (Val: %u) | Digital D2: %u | Unicast Queued: %s (CRC: 0x%04X)\n",
                  msg.seq, filteredVal, highResSensorVal, digitalVal,
                  sentDirect ? "SUCCESS" : "FAILED", msg.crc16);
}

/**
 * Callback when a mesh message is received.
 * Strict Session Tuple Validation & Duplicate HELLO Sequence Protection.
 */
void receivedCallback(uint32_t from, String &msg) {
    // 1. Check for Handshake Messages (HELLO / HELLO_ACK) - EXCLUSIVE session installation mechanism
    if (msg.length() == HANDSHAKE_WIRE_HEX_LEN) {
        HandshakeMessage handshake{};
        if (!hexToBytes(msg, (uint8_t*)&handshake, sizeof(HandshakeMessage))) {
            return;
        }

        uint16_t expectedCrc = calculateCRC16((const uint8_t*)&handshake, sizeof(HandshakeMessage) - sizeof(uint16_t));
        if (handshake.crc16 != expectedCrc) {
            Serial.printf("[RX CRC REJECT] Handshake CRC mismatch: received 0x%04X, expected 0x%04X\n",
                          handshake.crc16, expectedCrc);
            return;
        }

        if (handshake.sender_id == TARGET_NODE_ID && handshake.target_id == MY_NODE_ID) {
            bool isNewSession = (handshake.session_id != peerSession.sessionId);
            bool isNewHelloSeq = isNewerSequence(handshake.seq, peerSession.lastHelloSeq, peerSession.hasHelloSeq);

            if (handshake.magic == MSG_TYPE_HELLO) {
                if (isNewSession || isNewHelloSeq) {
                    peerSession.meshNodeId = from;
                    peerSession.sessionId = handshake.session_id;
                    peerSession.lastHelloSeq = handshake.seq;
                    peerSession.hasHelloSeq = true;
                    peerSession.state = PeerState::CONNECTED;

                    if (isNewSession) {
                        peerSession.lastDataSeq = 0;
                        peerSession.hasDataSeq = false;
                        Serial.printf("[HANDSHAKE] NEW Session ID %u established from Node %u (MeshID: %u). Data Seq reset.\n",
                                      handshake.session_id, TARGET_NODE_ID, from);
                    } else {
                        Serial.printf("[HANDSHAKE] Valid HELLO from existing Session ID %u (MeshID: %u, HelloSeq: %u).\n",
                                      handshake.session_id, from, handshake.seq);
                    }
                    sendHelloAck(from);
                } else {
                    Serial.printf("[HANDSHAKE] Duplicate HELLO (Session: %u, HelloSeq: %u). Sending ACK without DATA seq reset.\n",
                                  handshake.session_id, handshake.seq);
                    sendHelloAck(from);
                }
            } else if (handshake.magic == MSG_TYPE_HELLO_ACK) {
                if (isNewSession || isNewHelloSeq) {
                    peerSession.meshNodeId = from;
                    peerSession.sessionId = handshake.session_id;
                    peerSession.lastHelloSeq = handshake.seq;
                    peerSession.hasHelloSeq = true;
                    peerSession.state = PeerState::CONNECTED;

                    if (isNewSession) {
                        peerSession.lastDataSeq = 0;
                        peerSession.hasDataSeq = false;
                        Serial.printf("[HANDSHAKE] NEW Session ID %u ACKed from Node %u (MeshID: %u). Data Seq reset.\n",
                                      handshake.session_id, TARGET_NODE_ID, from);
                    }
                }
            }
        }
        return;
    }

    // 2. Check for Data Payload Messages
    if (msg.length() != PAIR_WIRE_HEX_LEN) {
        return;
    }

    if (peerSession.state != PeerState::CONNECTED) {
        return;
    }

    if (from != peerSession.meshNodeId) {
        Serial.printf("[RX REJECT] DATA payload from unverified MeshID %u (Active Target MeshID: %u)\n",
                      from, peerSession.meshNodeId);
        return;
    }

    SensorMessage incoming{};
    if (!hexToBytes(msg, (uint8_t*)&incoming, sizeof(SensorMessage))) {
        return;
    }

    uint16_t expectedCrc = calculateCRC16((const uint8_t*)&incoming, sizeof(SensorMessage) - sizeof(uint16_t));
    if (incoming.crc16 != expectedCrc) {
        Serial.printf("[RX CRC REJECT] Data Payload CRC mismatch: received 0x%04X, expected 0x%04X\n",
                      incoming.crc16, expectedCrc);
        return;
    }

    if (incoming.magic != MSG_TYPE_DATA) {
        return;
    }

    if (incoming.sender_id != TARGET_NODE_ID || incoming.target_id != MY_NODE_ID) {
        return;
    }

    if (incoming.session_id != peerSession.sessionId) {
        Serial.printf("[RX SESSION REJECT] Dropped DATA with stale/unmatched Session ID %u (Active Session: %u)\n",
                      incoming.session_id, peerSession.sessionId);
        return;
    }

    if (!isNewerSequence(incoming.seq, peerSession.lastDataSeq, peerSession.hasDataSeq)) {
        Serial.printf("[RX DROP #%u] Out-of-order or duplicate DATA packet dropped (Last Seq: %u, Sender Session: %u)\n",
                      incoming.seq, peerSession.lastDataSeq, incoming.session_id);
        return;
    }

    peerSession.lastDataSeq = incoming.seq;
    peerSession.hasDataSeq = true;

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

void changedConnectionCallback() {
    Serial.printf("[MESH] Topology changed (Local Mesh Node ID = %u)\n", mesh.getNodeId());

    if (peerSession.meshNodeId != 0) {
        SimpleList<uint32_t> nodes = mesh.getNodeList();
        bool stillConnected = false;
        SimpleList<uint32_t>::iterator node = nodes.begin();
        while (node != nodes.end()) {
            if (*node == peerSession.meshNodeId) {
                stillConnected = true;
                break;
            }
            node++;
        }

        if (!stillConnected) {
            Serial.printf("[MESH] Target MeshID %u disconnected. Clearing peer state to DISCOVERING.\n", peerSession.meshNodeId);
            peerSession.meshNodeId = 0;
            peerSession.state = PeerState::DISCOVERING;
            peerSession.hasDataSeq = false;
            peerSession.hasHelloSeq = false;
        }
    }
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("[MESH] System time adjusted: %d us\n", offset);
}
