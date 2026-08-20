# ESP8266 Bi-Directional Sensor & Servo Mesh (DSP & LUT Hardened)

An auto-organizing ESP8266 mesh network built using `painlessMesh` featuring non-blocking sweeping siren audio output (`ENABLE_SIREN_OUTPUT` on D4), sticky alarm digital output latching (`LATCH_DIGITAL_OUTPUT_HIGH`), local alarm reset pin (`RESET_ALARM_PIN` / D5), unified `PeerSession` tuple validation, handshake sequence tracking, fast Lookup-Table (LUT) hex encoding/decoding (`bytesToHex` / `hexToBytes`), CRC-16-CCITT checksum verification, handshake-only session installation (`session_id`), zero heap fragmentation, strict rate-limited unicast transmission, adaptive Kalman DSP filtering, traffic separation (unicast control vs. discovery broadcasts), active peer discovery handshakes (`HELLO`/`HELLO_ACK`), state machine target management, paired-sender packet filtering, sequence verification, sub-microsecond fixed-point resolution, directional limit switch protection, and microsecond-level actuator control. This repository provides two firmware variants:

1. **`esp8266_mesh_pair`**: PWM & Digital IO version (transmits 1/sec periodic updates).
2. **`esp8266_mesh_servo`**: Servo version (50ms input polling, 200ms strict network rate limiting, sub-microsecond FP4 fixed-point pulse transmission, directional limit switch safety clamping).

Both versions allow creating paired ESP8266 nodes (configured with simple compile-time node IDs) that communicate bi-directionally over an ad-hoc Wi-Fi mesh network using compact hex-encoded binary packed structs.

---

## 1. Overview & Firmware Comparison

| Feature | `esp8266_mesh_pair` (PWM Version) | `esp8266_mesh_servo` (Servo Version) |
|---|---|---|
| **Input Polling Rate** | 1000 ms (1 Hz) | 50 ms (20 Hz local sampling, Wi-Fi PHY safe) |
| **Network Transmission Rate** | Periodic (1/sec = 1000 ms) | Rate-limited (max 1 packet per 200 ms / 5 Hz) |
| **Siren Sound Output Option** | Optional `#define ENABLE_SIREN_OUTPUT` on `D4` | Optional `#define ENABLE_SIREN_OUTPUT` on `D4` |
| **Sticky Alarm Output Option**| Optional `#define LATCH_DIGITAL_OUTPUT_HIGH` with Reset Pin D5 | Optional `#define LATCH_DIGITAL_OUTPUT_HIGH` with Reset Pin D5 |
| **Session Tracking** | Unified `PeerSession` tuple validation + Handshake Sequence Tracking | Unified `PeerSession` tuple validation + Handshake Sequence Tracking |
| **Duplicate HELLO Handling** | ACK sent; DATA sequence counter (`lastDataSeq`) is NOT reset | ACK sent; DATA sequence counter (`lastDataSeq`) is NOT reset |
| **Hex Serialization Engine** | Fast local Lookup Table (`HEX_LUT[]`) | Fast local Lookup Table (`HEX_LUT[]`) |
| **Checksum Verification** | 16-bit CRC-16-CCITT (`crc16`) | 16-bit CRC-16-CCITT (`crc16`) |
| **Session Incarnation Policy** | Handshake-Only Session Installation (`MSG_TYPE_HELLO` / `HELLO_ACK`) | Handshake-Only Session Installation (`MSG_TYPE_HELLO` / `HELLO_ACK`) |
| **Heap Memory Optimization** | Zero dynamic heap allocations (pre-reserved `txPayloadString` & static hex buffers) | Zero dynamic heap allocations (pre-reserved `txPayloadString` & static hex buffers) |
| **Route Locking** | Strict route validation (`from == targetMeshNodeId` when connected) | Strict route validation (`from == targetMeshNodeId` when connected) |
| **Control Traffic Transport** | Strict Targeted Unicast `mesh.sendSingle()` | Strict Targeted Unicast `mesh.sendSingle()` |
| **Discovery Traffic Transport**| Infrequent Broadcast `MSG_TYPE_HELLO` | Infrequent Broadcast `MSG_TYPE_HELLO` |
| **Handshake Response** | Unicast `HELLO_ACK` (no broadcast fallback) | Unicast `HELLO_ACK` (no broadcast fallback) |
| **Peer State Machine** | `UNKNOWN`, `DISCOVERING`, `CONNECTED` | `UNKNOWN`, `DISCOVERING`, `CONNECTED` |
| **Sender Filtering & Sequence** | Paired sender validation (`sender_id == TARGET_NODE_ID`) & wraparound sequence tracking | Paired sender validation (`sender_id == TARGET_NODE_ID`) & wraparound sequence tracking |
| **Analog Input Processing** | Sort-based Trimmed Mean + Kahan Oversampling + Adaptive 1D Kalman Filter | Sort-based Trimmed Mean + Kahan Oversampling + Adaptive 1D Kalman Filter |
| **Actuator Drive Resolution**| PWM Output on `D1` (GPIO 5, 0-1023) | Sub-microsecond FP4 Fixed-Point (1/16th $\mu s$) via `writeMicroseconds()` |
| **Digital IO Pin** | `D2` In $\rightarrow$ `D3` Out | `D2` In $\rightarrow$ `D3` Out |
| **Limit Switch Support** | N/A | `D6` (Min Limit) & `D7` (Max Limit) |
| **Limit Switch Safety**| N/A | Directional motion blocking (forbids movement toward triggered limit, allows reverse) |
| **Struct Size** | 18 bytes (36 hex characters) | 20 bytes (40 hex characters) |

---

## 2. Siren Sound Output & Sticky Alarm Logic

### Siren Sound Output Mode (`#define ENABLE_SIREN_OUTPUT`)
Outputs a sweeping dual-tone siren sound on `SIREN_PIN` (`D4` / GPIO 2) whenever `DIGITAL_OUTPUT_PIN` (`D3`) is `HIGH`:
- **Default**: Disabled (`ENABLE_SIREN_OUTPUT` commented out in `config.h`), freeing up pin `D4` for general GPIO use.
- **Siren Enabled**: Uncomment `#define ENABLE_SIREN_OUTPUT` in `config.h`. A non-blocking `TaskScheduler` task modulates tone frequency between `600 Hz` and `1200 Hz` on `D4` whenever `D3` is `HIGH`.
- **Guarded Macros**: All siren tasks, functions, variables, and pin initializations are strictly guarded inside `#ifdef ENABLE_SIREN_OUTPUT` blocks.
- **Automatic Silencing**: When `D3` drops `LOW` or when `RESET_ALARM_PIN` (D5) is pulled `LOW`, `noTone(SIREN_PIN)` immediately silences `D4`.

### Sticky Alarm Mode (`#define LATCH_DIGITAL_OUTPUT_HIGH`)
For door/window switches where opening events (D2 `HIGH`) must remain latched even after closing:
- **Sticky Alarm Enabled**: Uncomment `#define LATCH_DIGITAL_OUTPUT_HIGH` in `config.h`. When a HIGH digital input arrives, `DIGITAL_OUTPUT_PIN` (D3) latches `HIGH` permanently.
- **Alarm Reset Pin (`RESET_ALARM_PIN` / D5)**: Pulling `RESET_ALARM_PIN` (D5) `LOW` clears latched memory, forcing `D3` and Siren `D4` to `LOW`.

---

## 3. Hardware Requirements & Wiring

### Pinout Summary
| Function | ESP8266 Pin | Notes |
|---|---|---|
| **Analog Sensor Input** | `A0` | ESP8266 ADC input pin (0 to 1.0V/3.3V, 10-bit resolution, DSP filtered) |
| **PWM Output / Servo Signal** | `D1` (GPIO 5) | PWM output in `esp8266_mesh_pair` or Microsecond Servo signal in `esp8266_mesh_servo` (544–2400 $\mu s$) |
| **Digital Input** | `D2` (GPIO 4) | Input pin read by sending node (`INPUT_PULLUP`) |
| **Digital Output** | `D3` (GPIO 0) | Output pin driven by received paired node digital state (`digitalWrite`) |
| **Siren Sound Output** | `D4` (GPIO 2) | Sweeping dual-tone audio output whenever `D3` is `HIGH` (Guarded by `#ifdef ENABLE_SIREN_OUTPUT`, freed when disabled) |
| **Alarm Reset Input** | `D5` (GPIO 14) | Resets sticky latched alarm output state to LOW (`INPUT_PULLUP`, Active LOW) |
| **Min Limit Switch** | `D6` (GPIO 12) | Min position limit switch in `esp8266_mesh_servo` (Active LOW `INPUT_PULLUP`) |
| **Max Limit Switch** | `D7` (GPIO 13) | Max position limit switch in `esp8266_mesh_servo` (Active LOW `INPUT_PULLUP`) |
| **GND / VCC** | GND / 3V3 / 5V | Common power and ground (power Servos from external 5V if required) |

---

## 4. Firmware Setup & Compilation

```bash
# Compile Node 1 (MY_NODE_ID=1, TARGET_NODE_ID=2, with siren output & sticky alarm enabled)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=1 -DTARGET_NODE_ID=2 -DENABLE_SIREN_OUTPUT -DLATCH_DIGITAL_OUTPUT_HIGH" \
  esp8266_mesh_servo

# Upload to Node 1
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_servo

# Compile Node 2 (MY_NODE_ID=2, TARGET_NODE_ID=1)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=2 -DTARGET_NODE_ID=1" \
  esp8266_mesh_servo

# Upload to Node 2
arduino-cli upload -p /dev/ttyUSB1 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_servo
```

---

## 5. Operation & Diagnostics

```text
==================================================
ESP8266 Bi-directional Servo Mesh Node
My Node ID: 1 (Session: 3849201) -> Target Node ID: 2
Analog In: A0 (Adaptive Kahan+Kalman) | Servo Pin: GPIO 5 (D1) [544 - 2400 us]
Digital In: GPIO 4 (D2) | Digital Out: GPIO 0 (D3) | Siren Pin: GPIO 2 (D4)
Min Limit Pin: GPIO 12 (D6) | Max Limit Pin: GPIO 13 (D7) | Reset Alarm Pin: GPIO 14 (D5)
Option: LATCH_DIGITAL_OUTPUT_HIGH ENABLED (Sticky Alarm Mode)
Option: ENABLE_SIREN_OUTPUT ENABLED (Siren Audio on D4 when D3 HIGH)
==================================================
[DISCOVERY #1] Sent HELLO broadcast for Target Node 2 (Session: 3849201, CRC: 0xA3F1)
[HANDSHAKE] Received HELLO_ACK from Target Node 2 (MeshID: 312847102, Session: 9812402). Transitioned to CONNECTED.
[SESSION] Handshake Established Active Target Session ID: 9812402 (Seq Reset to 0)
[TX #1] Filtered ADC: 512.35 | Target Pulse: 1472.25 us (FP4: 23556) | Unicast Queued: SUCCESS (CRC: 0xB4E2)
[RX #1] From Node: 2 (Session: 9812402) | Target Pulse: 1950.12 us (FP4: 31202) | Digital: 1 | MinLim: 0 | MaxLim: 0
[SERVO FP4] Requested: 1950.12 us (31202) -> Applied: 1950 us (31202 FP4) | LastSafe: 31202 FP4 (MinLim: 0, MaxLim: 0)
[ALARM RESET] Local Reset Pin D5 pulled LOW -> Output memory cleared, D3 & Siren forced LOW.
```
