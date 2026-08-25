#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <painlessMesh.h>
#include <ESP32Servo.h>

#include "config.h"

// ============================================================================
// Global objects
// ============================================================================

Scheduler userScheduler;
painlessMesh mesh;
Servo myServo;

// ============================================================================
// Function declarations
// ============================================================================

void checkAndTransmitInputs();
void sendHelloDiscovery();

#ifdef ENABLE_SIREN_OUTPUT
void updateSirenAudio();
#endif

void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);

void updateLocalServoFp4(uint16_t requestedUsFp4);
float readAnalogFiltered();

bool isNewerSequence(
    uint32_t incoming,
    uint32_t last,
    bool initialized
);

uint16_t calculateCRC16(
    const uint8_t *data,
    size_t length
);

void bytesToHex(
    const uint8_t *src,
    size_t srcLen,
    char *dest
);

bool hexToBytes(
    const String &hexStr,
    uint8_t *dest,
    size_t destLen
);

// ============================================================================
// Tasks
// ============================================================================

Task taskPollInputs(
    POLL_INTERVAL_MS,
    TASK_FOREVER,
    &checkAndTransmitInputs
);

Task taskDiscovery(
    DISCOVERY_INTERVAL_MS,
    TASK_FOREVER,
    &sendHelloDiscovery
);

#ifdef ENABLE_SIREN_OUTPUT

Task taskSiren(
    SIREN_TICK_MS,
    TASK_FOREVER,
    &updateSirenAudio
);

static uint16_t currentSirenFreq = SIREN_FREQ_LOW;
static bool sirenSweepRising = true;

#endif

// ============================================================================
// Session
// ============================================================================

static uint32_t mySessionId = 0;

// ============================================================================
// Sticky alarm
// ============================================================================

static bool latchedDigitalOutputState = false;

// ============================================================================
// Peer session
// ============================================================================

struct PeerSession {
    uint32_t meshNodeId;
    uint16_t senderId;

    uint32_t sessionId;

    uint32_t lastDataSeq;
    uint32_t lastHelloSeq;

    PeerState state;

    bool hasDataSeq;
    bool hasHelloSeq;
};

static PeerSession peerSession{};

// ============================================================================
// Kalman state
// ============================================================================

static float kalman_x = 512.0f;
static float kalman_p = 1.0f;

// ============================================================================
// Transmission state
// ============================================================================

static uint16_t lastTransmittedUsFp4 = 0xFFFF;
static uint8_t lastDigitalVal = 0xFF;

#ifndef NO_ENDSTOPS
static uint8_t lastMinLimit = 0xFF;
static uint8_t lastMaxLimit = 0xFF;
#endif

static uint32_t lastTxTime = 0;
static uint32_t messageSequence = 0;
static uint32_t discoverySequence = 0;

// ============================================================================
// Servo state
// ============================================================================

static uint16_t requestedServoUsFp4 = 23552; // 1472 us * 16
static uint16_t appliedServoUsFp4   = 23552;
static uint16_t lastSafeUsFp4       = 23552;

// ============================================================================
// Static communication buffers
// ============================================================================

static const char HEX_LUT[] = "0123456789ABCDEF";

static char staticHexTxBuffer[
    SERVO_WIRE_HEX_LEN + 1
];

static String txPayloadString;

// ============================================================================
// CRC16-CCITT
// ============================================================================

uint16_t calculateCRC16(
    const uint8_t *data,
    size_t length
) {
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

// ============================================================================
// Binary -> HEX
// ============================================================================

void bytesToHex(
    const uint8_t *src,
    size_t srcLen,
    char *dest
) {
    for (size_t i = 0; i < srcLen; i++) {

        uint8_t byte = src[i];

        dest[i * 2] =
            HEX_LUT[(byte >> 4) & 0x0F];

        dest[i * 2 + 1] =
            HEX_LUT[byte & 0x0F];
    }

    dest[srcLen * 2] = '\0';
}

// ============================================================================
// HEX -> Binary
// ============================================================================

bool hexToBytes(
    const String &hexStr,
    uint8_t *dest,
    size_t destLen
) {
    if (hexStr.length() != destLen * 2) {
        return false;
    }

    for (size_t i = 0; i < destLen; i++) {

        char high = hexStr.charAt(i * 2);
        char low  = hexStr.charAt(i * 2 + 1);

        uint8_t highNibble;
        uint8_t lowNibble;

        if (high >= '0' && high <= '9') {
            highNibble = high - '0';
        }
        else if (high >= 'A' && high <= 'F') {
            highNibble = high - 'A' + 10;
        }
        else if (high >= 'a' && high <= 'f') {
            highNibble = high - 'a' + 10;
        }
        else {
            return false;
        }

        if (low >= '0' && low <= '9') {
            lowNibble = low - '0';
        }
        else if (low >= 'A' && low <= 'F') {
            lowNibble = low - 'A' + 10;
        }
        else if (low >= 'a' && low <= 'f') {
            lowNibble = low - 'a' + 10;
        }
        else {
            return false;
        }

        dest[i] =
            (highNibble << 4) | lowNibble;
    }

    return true;
}

// ============================================================================
// Siren
// ============================================================================

#ifdef ENABLE_SIREN_OUTPUT

void updateSirenAudio() {

    if (digitalRead(DIGITAL_OUTPUT_PIN) == HIGH) {

        if (sirenSweepRising) {

            currentSirenFreq += SIREN_STEP_HZ;

            if (currentSirenFreq >= SIREN_FREQ_HIGH) {

                currentSirenFreq = SIREN_FREQ_HIGH;
                sirenSweepRising = false;
            }

        } else {

            if (currentSirenFreq <=
                SIREN_FREQ_LOW + SIREN_STEP_HZ) {

                currentSirenFreq = SIREN_FREQ_LOW;
                sirenSweepRising = true;

            } else {

                currentSirenFreq -= SIREN_STEP_HZ;
            }
        }

        tone(
            SIREN_PIN,
            currentSirenFreq
        );

    } else {

        noTone(SIREN_PIN);
        digitalWrite(SIREN_PIN, LOW);

        currentSirenFreq = SIREN_FREQ_LOW;
        sirenSweepRising = true;
    }
}

#endif

// ============================================================================
// Sequence comparison
// ============================================================================

bool isNewerSequence(
    uint32_t incoming,
    uint32_t last,
    bool initialized
) {
    if (!initialized) {
        return true;
    }

    return ((int32_t)(incoming - last)) > 0;
}

// ============================================================================
// ADC + trimmed mean + Kahan + adaptive Kalman
// ============================================================================

float readAnalogFiltered() {

    uint16_t samples[ADC_OVERSAMPLE_COUNT];

    for (size_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {

        samples[i] = analogRead(SENSOR_PIN);

        // ESP32 equivalent of ESP8266 optimistic_yield()
        yield();
    }

    // ------------------------------------------------------------------------
    // Sort
    // ------------------------------------------------------------------------

    for (
        size_t i = 0;
        i < ADC_OVERSAMPLE_COUNT - 1;
        i++
    ) {

        for (
            size_t j = i + 1;
            j < ADC_OVERSAMPLE_COUNT;
            j++
        ) {

            if (samples[i] > samples[j]) {

                uint16_t temp = samples[i];

                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Trim one sample from each end
    // ------------------------------------------------------------------------

    float sum = 0.0f;
    float c = 0.0f;

    size_t startIndex =
        (ADC_OVERSAMPLE_COUNT >= 4) ? 1 : 0;

    size_t endIndex =
        (ADC_OVERSAMPLE_COUNT >= 4)
            ? ADC_OVERSAMPLE_COUNT - 1
            : ADC_OVERSAMPLE_COUNT;

    size_t count =
        endIndex - startIndex;

    // ------------------------------------------------------------------------
    // Kahan summation
    // ------------------------------------------------------------------------

    for (
        size_t i = startIndex;
        i < endIndex;
        i++
    ) {

        float y =
            (float)samples[i] - c;

        float t =
            sum + y;

        c =
            (t - sum) - y;

        sum = t;
    }

    float averageAdc =
        (count > 0)
            ? sum / (float)count
            : (float)samples[0];

    // ------------------------------------------------------------------------
    // Adaptive Kalman
    // ------------------------------------------------------------------------

    float innovation =
        fabsf(averageAdc - kalman_x);

    float dynamicQ =
        KALMAN_PROCESS_NOISE_Q;

    if (innovation > 10.0f) {
        dynamicQ = innovation * 0.1f;
    }

    kalman_p += dynamicQ;

    float k_gain =
        kalman_p /
        (kalman_p + KALMAN_MEASUREMENT_NOISE_R);

    kalman_x +=
        k_gain * (averageAdc - kalman_x);

    kalman_p =
        (1.0f - k_gain) * kalman_p;

    return kalman_x;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {

    Serial.begin(115200);

    delay(500);

 
    // ESP32 defaults to 12-bit / 0..4095.
 
    // ADC1 + WiFi safe pin.
    //
    // GPIO34 is ADC1_CH6 on classic ESP32.
    // ------------------------------------------------------------------------

    analogSetPinAttenuation(
        SENSOR_PIN,
        ADC_11db
    );

    // ------------------------------------------------------------------------
    // Generate unique boot session
    // ------------------------------------------------------------------------

    uint64_t efuseMac =
        ESP.getEfuseMac();

    uint32_t macLow =
        (uint32_t)(efuseMac & 0xFFFFFFFFULL);

    uint32_t macHigh =
        (uint32_t)(efuseMac >> 32);

    mySessionId =
        macLow ^
        macHigh ^
        micros() ^
        esp_random();

    // ------------------------------------------------------------------------
    // Reserve payload buffer
    // ------------------------------------------------------------------------

    txPayloadString.reserve(
        SERVO_WIRE_HEX_LEN + 1
    );

    // ------------------------------------------------------------------------
    // Peer state
    // ------------------------------------------------------------------------

    peerSession.senderId =
        TARGET_NODE_ID;

    peerSession.meshNodeId = 0;

    peerSession.sessionId = 0;

    peerSession.lastDataSeq = 0;
    peerSession.lastHelloSeq = 0;

    peerSession.state =
        PeerState::DISCOVERING;

    peerSession.hasDataSeq = false;
    peerSession.hasHelloSeq = false;

    // ------------------------------------------------------------------------
    // Startup information
    // ------------------------------------------------------------------------

    Serial.println();
    Serial.println(
        "=================================================="
    );

    Serial.println(
        "ESP32 Bi-directional Servo Mesh Node"
    );

    Serial.printf(
        "My Node ID: %u (Session: %u) -> Target Node ID: %u\n",
        MY_NODE_ID,
        mySessionId,
        TARGET_NODE_ID
    );

    Serial.printf(
        "Analog In: GPIO %d (10-bit Adaptive Kahan+Kalman)\n",
        SENSOR_PIN
    );

    Serial.printf(
        "Servo Pin: GPIO %d [%d - %d us]\n",
        SERVO_PIN,
        SERVO_MIN_PULSE_WIDTH,
        SERVO_MAX_PULSE_WIDTH
    );

    Serial.printf(
        "Digital In: GPIO %d | Digital Out: GPIO %d\n",
        DIGITAL_INPUT_PIN,
        DIGITAL_OUTPUT_PIN
    );

#ifdef ENABLE_SIREN_OUTPUT

    Serial.printf(
        "Siren Pin: GPIO %d\n",
        SIREN_PIN
    );

#endif

#ifdef NO_ENDSTOPS

    Serial.printf(
        "Reset Alarm Pin: GPIO %d | Endstops DISABLED\n",
        RESET_ALARM_PIN
    );

#else

    Serial.printf(
        "Min Limit: GPIO %d | Max Limit: GPIO %d | Reset: GPIO %d\n",
        MIN_LIMIT_PIN,
        MAX_LIMIT_PIN,
        RESET_ALARM_PIN
    );

#endif

#ifdef LATCH_DIGITAL_OUTPUT_HIGH

    Serial.println(
        "Sticky Alarm: ENABLED"
    );

#else

    Serial.println(
        "Sticky Alarm: DISABLED"
    );

#endif

#ifdef ENABLE_SIREN_OUTPUT

    Serial.println(
        "Siren: ENABLED"
    );

#else

    Serial.println(
        "Siren: DISABLED"
    );

#endif

#ifdef DEBUG_CONNECTION

    Serial.println(
        "Connection Debug: ENABLED"
    );

#endif

    Serial.println(
        "=================================================="
    );

    // ------------------------------------------------------------------------
    // GPIO
    // ------------------------------------------------------------------------

    pinMode(
        SENSOR_PIN,
        INPUT
    );

    pinMode(
        DIGITAL_INPUT_PIN,
        INPUT_PULLUP
    );

    pinMode(
        DIGITAL_OUTPUT_PIN,
        OUTPUT
    );

    digitalWrite(
        DIGITAL_OUTPUT_PIN,
        LOW
    );

#ifdef ENABLE_SIREN_OUTPUT

    pinMode(
        SIREN_PIN,
        OUTPUT
    );

    digitalWrite(
        SIREN_PIN,
        LOW
    );

#endif

#ifndef NO_ENDSTOPS

    pinMode(
        MIN_LIMIT_PIN,
        INPUT_PULLUP
    );

    pinMode(
        MAX_LIMIT_PIN,
        INPUT_PULLUP
    );

#endif

    pinMode(
        RESET_ALARM_PIN,
        INPUT_PULLUP
    );

    // ------------------------------------------------------------------------
    // Initial Kalman state
    // ------------------------------------------------------------------------

    kalman_x =
        (float)analogRead(SENSOR_PIN);

    // ------------------------------------------------------------------------
    // Servo
    // ------------------------------------------------------------------------

    myServo.attach(
        SERVO_PIN,
        SERVO_MIN_PULSE_WIDTH,
        SERVO_MAX_PULSE_WIDTH
    );

    updateLocalServoFp4(
        requestedServoUsFp4
    );

    // ------------------------------------------------------------------------
    // painlessMesh
    // ------------------------------------------------------------------------

    mesh.init(
        MESH_PREFIX,
        MESH_PASSWORD,
        &userScheduler,
        MESH_PORT
    );

    mesh.onReceive(
        &receivedCallback
    );

    mesh.onNewConnection(
        &newConnectionCallback
    );

    mesh.onChangedConnections(
        &changedConnectionCallback
    );

    mesh.onNodeTimeAdjusted(
        &nodeTimeAdjustedCallback
    );

    delay(500);

    // ------------------------------------------------------------------------
    // ESP32 WiFi configuration
    // ------------------------------------------------------------------------

    //
    // ESP32 maximum selectable Arduino TX power is normally exposed as
    // WIFI_POWER_19_5dBm rather than the ESP8266 20.5 dBm setting.
    //

    WiFi.setTxPower(
        WIFI_POWER_19_5dBm
    );

    //
    // Disable modem sleep to minimize mesh latency.
    //

    WiFi.setSleep(false);

    //
    // Force 802.11b only.
    //
    // painlessMesh uses the native ESP WiFi networking stack on ESP32.
    //

    esp_err_t phySta =
        esp_wifi_set_protocol(
            WIFI_IF_STA,
            WIFI_PROTOCOL_11B
        );

    esp_err_t phyAp =
        esp_wifi_set_protocol(
            WIFI_IF_AP,
            WIFI_PROTOCOL_11B
        );

    Serial.printf(
        "ESP32 PHY setup: STA=%s AP=%s\n",
        phySta == ESP_OK ? "OK" : "FAILED",
        phyAp == ESP_OK ? "OK" : "FAILED"
    );

    // ------------------------------------------------------------------------
    // Tasks
    // ------------------------------------------------------------------------

    userScheduler.addTask(
        taskPollInputs
    );

    taskPollInputs.enable();

    userScheduler.addTask(
        taskDiscovery
    );

    taskDiscovery.enable();

#ifdef ENABLE_SIREN_OUTPUT

    userScheduler.addTask(
        taskSiren
    );

    taskSiren.enable();

#endif
}

// ============================================================================
// Main loop
// ============================================================================

void loop() {

    mesh.update();

    yield();
}

// ============================================================================
// HELLO Discovery
// ============================================================================

void sendHelloDiscovery() {

    if (
        peerSession.state ==
        PeerState::CONNECTED
    ) {
        return;
    }

    HandshakeMessage helloMsg{};

    helloMsg.magic =
        MSG_TYPE_HELLO;

    helloMsg.sender_id =
        MY_NODE_ID;

    helloMsg.target_id =
        TARGET_NODE_ID;

    helloMsg.session_id =
        mySessionId;

    helloMsg.seq =
        ++discoverySequence;

    helloMsg.crc16 =
        calculateCRC16(
            (const uint8_t *)&helloMsg,
            sizeof(HandshakeMessage) -
            sizeof(uint16_t)
        );

    bytesToHex(
        (const uint8_t *)&helloMsg,
        sizeof(HandshakeMessage),
        staticHexTxBuffer
    );

    txPayloadString =
        staticHexTxBuffer;

    mesh.sendBroadcast(
        txPayloadString
    );

    Serial.printf(
        "[DISCOVERY #%u] Sent HELLO broadcast for Target Node %u (Session: %u)\n",
        helloMsg.seq,
        TARGET_NODE_ID,
        mySessionId
    );
}

// ============================================================================
// HELLO ACK
// ============================================================================

void sendHelloAck(
    uint32_t destMeshId
) {

    HandshakeMessage ackMsg{};

    ackMsg.magic =
        MSG_TYPE_HELLO_ACK;

    ackMsg.sender_id =
        MY_NODE_ID;

    ackMsg.target_id =
        TARGET_NODE_ID;

    ackMsg.session_id =
        mySessionId;

    ackMsg.seq =
        ++discoverySequence;

    ackMsg.crc16 =
        calculateCRC16(
            (const uint8_t *)&ackMsg,
            sizeof(HandshakeMessage) -
            sizeof(uint16_t)
        );

    bytesToHex(
        (const uint8_t *)&ackMsg,
        sizeof(HandshakeMessage),
        staticHexTxBuffer
    );

    txPayloadString =
        staticHexTxBuffer;

    mesh.sendSingle(
        destMeshId,
        txPayloadString
    );

    Serial.printf(
        "[DISCOVERY #%u] HELLO_ACK -> MeshID %u\n",
        ackMsg.seq,
        destMeshId
    );
}

// ============================================================================
// Input polling + TX
// ============================================================================

void checkAndTransmitInputs() {

    // ------------------------------------------------------------------------
    // Alarm reset
    // ------------------------------------------------------------------------

    if (
        digitalRead(RESET_ALARM_PIN) ==
        LOW
    ) {

        if (
            latchedDigitalOutputState ||
            digitalRead(DIGITAL_OUTPUT_PIN) == HIGH
        ) {

            latchedDigitalOutputState =
                false;

            digitalWrite(
                DIGITAL_OUTPUT_PIN,
                LOW
            );

#ifdef ENABLE_SIREN_OUTPUT

            noTone(SIREN_PIN);

            digitalWrite(
                SIREN_PIN,
                LOW
            );

#endif

            Serial.println(
                "[ALARM RESET] Output memory cleared."
            );
        }
    }

    // ------------------------------------------------------------------------
    // Filtered analog signal
    // ------------------------------------------------------------------------

    float filteredAdc =
        readAnalogFiltered();

    // ------------------------------------------------------------------------
    // Convert 0..4095 ADC -> servo pulse
    // ------------------------------------------------------------------------

    float targetPulseUs =
        (float)SERVO_MIN_PULSE_WIDTH +
        (filteredAdc / ADC_RESOLUTION) *
        (float)(
            SERVO_MAX_PULSE_WIDTH -
            SERVO_MIN_PULSE_WIDTH
        );

    float targetPulseFp4Float =
        targetPulseUs * 16.0f;

    uint16_t minUsFp4 =
        SERVO_MIN_PULSE_WIDTH * 16;

    uint16_t maxUsFp4 =
        SERVO_MAX_PULSE_WIDTH * 16;

    uint16_t currentUsFp4 =
        (uint16_t)constrain(
            (int)roundf(targetPulseFp4Float),
            minUsFp4,
            maxUsFp4
        );

    // ------------------------------------------------------------------------
    // Digital input
    // ------------------------------------------------------------------------

    uint8_t currentDigital =
        digitalRead(DIGITAL_INPUT_PIN);

    // ------------------------------------------------------------------------
    // Endstops
    // ------------------------------------------------------------------------

#ifndef NO_ENDSTOPS

    uint8_t currentMinLimit =
        (digitalRead(MIN_LIMIT_PIN) == LOW)
            ? 1
            : 0;

    uint8_t currentMaxLimit =
        (digitalRead(MAX_LIMIT_PIN) == LOW)
            ? 1
            : 0;

    static uint8_t lastLocalMinLimit = 0xFF;
    static uint8_t lastLocalMaxLimit = 0xFF;

    if (
        currentMinLimit != lastLocalMinLimit ||
        currentMaxLimit != lastLocalMaxLimit
    ) {

        lastLocalMinLimit =
            currentMinLimit;

        lastLocalMaxLimit =
            currentMaxLimit;

        updateLocalServoFp4(
            requestedServoUsFp4
        );
    }

    bool minLimitChanged =
        currentMinLimit != lastMinLimit;

    bool maxLimitChanged =
        currentMaxLimit != lastMaxLimit;

#else

    uint8_t currentMinLimit = 0;
    uint8_t currentMaxLimit = 0;

    bool minLimitChanged = false;
    bool maxLimitChanged = false;

#endif

    // ------------------------------------------------------------------------
    // Must be connected first
    // ------------------------------------------------------------------------

    if (
        peerSession.state !=
        PeerState::CONNECTED ||
        peerSession.meshNodeId == 0 ||
        !mesh.isConnected(
            peerSession.meshNodeId
        )
    ) {

        return;
    }

    // ------------------------------------------------------------------------
    // Rate limiting
    // ------------------------------------------------------------------------

    uint32_t now =
        millis();

    if (
        lastTxTime != 0 &&
        (now - lastTxTime <
         MIN_TX_INTERVAL_MS)
    ) {

        return;
    }

    // ------------------------------------------------------------------------
    // State changes
    // ------------------------------------------------------------------------

    bool pulseChanged =
        abs(
            (int)currentUsFp4 -
            (int)lastTransmittedUsFp4
        ) >= PULSE_FP4_CHANGE_THRESHOLD;

    bool digitalChanged =
        currentDigital !=
        lastDigitalVal;

    bool stateChanged =
        pulseChanged ||
        digitalChanged ||
        minLimitChanged ||
        maxLimitChanged;

    bool heartbeatElapsed =
        (now - lastTxTime) >=
        HEARTBEAT_INTERVAL_MS;

    if (
        stateChanged ||
        heartbeatElapsed
    ) {

        ServoMeshMessage msg{};

        msg.magic =
            MSG_TYPE_DATA;

        msg.sender_id =
            MY_NODE_ID;

        msg.target_id =
            TARGET_NODE_ID;

        msg.session_id =
            mySessionId;

        msg.target_us_fp4 =
            currentUsFp4;

        msg.digital_value =
            currentDigital;

        msg.min_limit_active =
            currentMinLimit;

        msg.max_limit_active =
            currentMaxLimit;

        msg.seq =
            ++messageSequence;

        msg.crc16 =
            calculateCRC16(
                (const uint8_t *)&msg,
                sizeof(ServoMeshMessage) -
                sizeof(uint16_t)
            );

        bytesToHex(
            (const uint8_t *)&msg,
            sizeof(ServoMeshMessage),
            staticHexTxBuffer
        );

        lastTxTime =
            now;

        txPayloadString =
            staticHexTxBuffer;

        bool sentDirect =
            mesh.sendSingle(
                peerSession.meshNodeId,
                txPayloadString
            );

        if (sentDirect) {

            lastTransmittedUsFp4 =
                currentUsFp4;

            lastDigitalVal =
                currentDigital;

#ifndef NO_ENDSTOPS

            lastMinLimit =
                currentMinLimit;

            lastMaxLimit =
                currentMaxLimit;

#endif
        }

        const char *txTypeStr =
            heartbeatElapsed
                ? "HEARTBEAT"
                : "EVENT";

        Serial.printf(
            "[TX %s #%u] ADC: %.2f | Pulse: %.2f us (FP4:%u) | %s | CRC:0x%04X",
            txTypeStr,
            msg.seq,
            filteredAdc,
            (float)currentUsFp4 / 16.0f,
            currentUsFp4,
            sentDirect ? "SUCCESS" : "FAILED",
            msg.crc16
        );

#ifdef DEBUG_CONNECTION

        int8_t rssi =
            WiFi.RSSI();

        SimpleList<uint32_t> nodeList =
            mesh.getNodeList();

        size_t totalMeshNodes =
            nodeList.size();

        uint32_t myMeshId =
            mesh.getNodeId();

        Serial.printf(
            " | RSSI:%d dBm | Peer:%u | Me:%u | Nodes:%zu",
            rssi,
            peerSession.meshNodeId,
            myMeshId,
            totalMeshNodes
        );

#endif

        Serial.println();
    }
}

// ============================================================================
// Local servo update
// ============================================================================

void updateLocalServoFp4(
    uint16_t newRequestedUsFp4
) {

    requestedServoUsFp4 =
        newRequestedUsFp4;

    uint16_t minUsFp4 =
        SERVO_MIN_PULSE_WIDTH * 16;

    uint16_t maxUsFp4 =
        SERVO_MAX_PULSE_WIDTH * 16;

    uint16_t targetUsFp4 =
        constrain(
            requestedServoUsFp4,
            minUsFp4,
            maxUsFp4
        );

#ifndef NO_ENDSTOPS

    bool minActive =
        digitalRead(MIN_LIMIT_PIN) == LOW;

    bool maxActive =
        digitalRead(MAX_LIMIT_PIN) == LOW;

    if (
        minActive &&
        targetUsFp4 < lastSafeUsFp4
    ) {
        targetUsFp4 =
            lastSafeUsFp4;
    }

    if (
        maxActive &&
        targetUsFp4 > lastSafeUsFp4
    ) {
        targetUsFp4 =
            lastSafeUsFp4;
    }

#endif

    uint16_t targetUsInt =
        (uint16_t)roundf(
            (float)targetUsFp4 /
            16.0f
        );

    myServo.writeMicroseconds(
        targetUsInt
    );

    appliedServoUsFp4 =
        targetUsFp4;

    lastSafeUsFp4 =
        targetUsFp4;

    Serial.printf(
        "[SERVO] Requested: %.2f us -> Applied: %u us (%u FP4)\n",
        (float)requestedServoUsFp4 / 16.0f,
        targetUsInt,
        appliedServoUsFp4
    );
}

// ============================================================================
// RX callback
// ============================================================================

void receivedCallback(
    uint32_t from,
    String &msg
) {

    // ========================================================================
    // HELLO / HELLO_ACK
    // ========================================================================

    if (
        msg.length() ==
        HANDSHAKE_WIRE_HEX_LEN
    ) {

        HandshakeMessage handshake{};

        if (
            !hexToBytes(
                msg,
                (uint8_t *)&handshake,
                sizeof(HandshakeMessage)
            )
        ) {
            return;
        }

        uint16_t expectedCrc =
            calculateCRC16(
                (const uint8_t *)&handshake,
                sizeof(HandshakeMessage) -
                sizeof(uint16_t)
            );

        if (
            handshake.crc16 !=
            expectedCrc
        ) {

            Serial.printf(
                "[RX CRC REJECT] Handshake: RX=0x%04X expected=0x%04X\n",
                handshake.crc16,
                expectedCrc
            );

            return;
        }

        if (
            handshake.sender_id ==
                TARGET_NODE_ID &&
            handshake.target_id ==
                MY_NODE_ID
        ) {

            bool isNewSession =
                handshake.session_id !=
                peerSession.sessionId;

            bool isNewHelloSeq =
                isNewerSequence(
                    handshake.seq,
                    peerSession.lastHelloSeq,
                    peerSession.hasHelloSeq
                );

            // ---------------------------------------------------------------
            // HELLO
            // ---------------------------------------------------------------

            if (
                handshake.magic ==
                MSG_TYPE_HELLO
            ) {

                if (
                    isNewSession ||
                    isNewHelloSeq
                ) {

                    peerSession.meshNodeId =
                        from;

                    peerSession.sessionId =
                        handshake.session_id;

                    peerSession.lastHelloSeq =
                        handshake.seq;

                    peerSession.hasHelloSeq =
                        true;

                    peerSession.state =
                        PeerState::CONNECTED;

                    if (isNewSession) {

                        peerSession.lastDataSeq =
                            0;

                        peerSession.hasDataSeq =
                            false;

                        Serial.printf(
                            "[HANDSHAKE] NEW session=%u MeshID=%u\n",
                            handshake.session_id,
                            from
                        );
                    }

                    sendHelloAck(from);

                } else {

                    sendHelloAck(from);
                }
            }

            // ---------------------------------------------------------------
            // HELLO ACK
            // ---------------------------------------------------------------

            else if (
                handshake.magic ==
                MSG_TYPE_HELLO_ACK
            ) {

                if (
                    isNewSession ||
                    isNewHelloSeq
                ) {

                    peerSession.meshNodeId =
                        from;

                    peerSession.sessionId =
                        handshake.session_id;

                    peerSession.lastHelloSeq =
                        handshake.seq;

                    peerSession.hasHelloSeq =
                        true;

                    peerSession.state =
                        PeerState::CONNECTED;

                    if (isNewSession) {

                        peerSession.lastDataSeq =
                            0;

                        peerSession.hasDataSeq =
                            false;

                        Serial.printf(
                            "[HANDSHAKE] Session %u acknowledged.\n",
                            handshake.session_id
                        );
                    }
                }
            }
        }

        return;
    }

    // ========================================================================
    // DATA
    // ========================================================================

    if (
        msg.length() !=
        SERVO_WIRE_HEX_LEN
    ) {
        return;
    }

    if (
        peerSession.state !=
        PeerState::CONNECTED
    ) {
        return;
    }

    if (
        from !=
        peerSession.meshNodeId
    ) {

        Serial.printf(
            "[RX REJECT] Unverified MeshID %u\n",
            from
        );

        return;
    }

    ServoMeshMessage incoming{};

    if (
        !hexToBytes(
            msg,
            (uint8_t *)&incoming,
            sizeof(ServoMeshMessage)
        )
    ) {
        return;
    }

    uint16_t expectedCrc =
        calculateCRC16(
            (const uint8_t *)&incoming,
            sizeof(ServoMeshMessage) -
            sizeof(uint16_t)
        );

    if (
        incoming.crc16 !=
        expectedCrc
    ) {

        Serial.printf(
            "[RX CRC REJECT] RX=0x%04X expected=0x%04X\n",
            incoming.crc16,
            expectedCrc
        );

        return;
    }

    if (
        incoming.magic !=
        MSG_TYPE_DATA
    ) {
        return;
    }

    if (
        incoming.sender_id !=
            TARGET_NODE_ID ||
        incoming.target_id !=
            MY_NODE_ID
    ) {
        return;
    }

    if (
        incoming.session_id !=
        peerSession.sessionId
    ) {

        Serial.printf(
            "[RX SESSION REJECT] Session %u != active %u\n",
            incoming.session_id,
            peerSession.sessionId
        );

        return;
    }

    if (
        !isNewerSequence(
            incoming.seq,
            peerSession.lastDataSeq,
            peerSession.hasDataSeq
        )
    ) {

        Serial.printf(
            "[RX DROP #%u] Duplicate/out-of-order packet\n",
            incoming.seq
        );

        return;
    }

    peerSession.lastDataSeq =
        incoming.seq;

    peerSession.hasDataSeq =
        true;

    // ========================================================================
    // Digital output
    // ========================================================================

#ifdef LATCH_DIGITAL_OUTPUT_HIGH

    if (incoming.digital_value) {
        latchedDigitalOutputState = true;
    }

    digitalWrite(
        DIGITAL_OUTPUT_PIN,
        latchedDigitalOutputState
            ? HIGH
            : LOW
    );

#else

    digitalWrite(
        DIGITAL_OUTPUT_PIN,
        incoming.digital_value
            ? HIGH
            : LOW
    );

#endif

    // ========================================================================
    // Servo
    // ========================================================================

    updateLocalServoFp4(
        incoming.target_us_fp4
    );

    Serial.printf(
        "[RX #%u] Node:%u Session:%u Pulse:%.2f us FP4:%u Digital:%u Min:%u Max:%u\n",
        incoming.seq,
        incoming.sender_id,
        incoming.session_id,
        (float)incoming.target_us_fp4 / 16.0f,
        incoming.target_us_fp4,
        incoming.digital_value,
        incoming.min_limit_active,
        incoming.max_limit_active
    );
}

// ============================================================================
// Mesh callbacks
// ============================================================================

void newConnectionCallback(
    uint32_t nodeId
) {

    Serial.printf(
        "[MESH] New connection nodeId=%u local=%u\n",
        nodeId,
        mesh.getNodeId()
    );
}

void changedConnectionCallback() {

    Serial.printf(
        "[MESH] Topology changed local=%u\n",
        mesh.getNodeId()
    );

    if (
        peerSession.meshNodeId != 0
    ) {

        SimpleList<uint32_t> nodes =
            mesh.getNodeList();

        bool stillConnected = false;

        SimpleList<uint32_t>::iterator node =
            nodes.begin();

        while (
            node != nodes.end()
        ) {

            if (
                *node ==
                peerSession.meshNodeId
            ) {

                stillConnected = true;
                break;
            }

            node++;
        }

        if (!stillConnected) {

            Serial.printf(
                "[MESH] Peer %u disconnected. Rediscovering.\n",
                peerSession.meshNodeId
            );

            peerSession.meshNodeId =
                0;

            peerSession.state =
                PeerState::DISCOVERING;

            peerSession.hasDataSeq =
                false;

            peerSession.hasHelloSeq =
                false;
        }
    }
}

void nodeTimeAdjustedCallback(
    int32_t offset
) {

    Serial.printf(
        "[MESH] System time adjusted: %d us\n",
        offset
    );
}
