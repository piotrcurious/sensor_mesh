# ESP8266 Bi-Directional Sensor & Servo Mesh (DSP Enhanced)

An auto-organizing ESP8266 mesh network built using `painlessMesh` featuring traffic separation (unicast control vs. discovery broadcasts), active peer discovery handshakes (`HELLO`/`HELLO_ACK`), state machine target management, paired-sender packet filtering, sequence verification, high-precision Digital Signal Processing (DSP) for analog inputs, sub-microsecond fixed-point resolution, directional limit switch protection, microsecond-level actuator control, and network rate limiting. This repository provides two firmware variants:

1. **`esp8266_mesh_pair`**: PWM & Digital IO version (transmits 1/sec periodic updates).
2. **`esp8266_mesh_servo`**: Servo version (50ms input polling, 200ms network rate limiting, sub-microsecond FP4 fixed-point pulse transmission, directional limit switch safety clamping).

Both versions allow creating paired ESP8266 nodes (configured with simple compile-time node IDs) that communicate bi-directionally over an ad-hoc Wi-Fi mesh network using compact hex-encoded binary packed structs.

---

## 1. Overview & Firmware Comparison

| Feature | `esp8266_mesh_pair` (PWM Version) | `esp8266_mesh_servo` (Servo Version) |
|---|---|---|
| **Input Polling Rate** | 1000 ms (1 Hz) | 50 ms (20 Hz local sampling, Wi-Fi PHY safe) |
| **Network Transmission Rate** | Periodic (1/sec = 1000 ms) | Rate-limited (max 1 packet per 200 ms / 5 Hz) |
| **Control Traffic Transport** | Strict Targeted Unicast `mesh.sendSingle()` | Strict Targeted Unicast `mesh.sendSingle()` |
| **Discovery Traffic Transport**| Infrequent Broadcast `MSG_TYPE_HELLO` | Infrequent Broadcast `MSG_TYPE_HELLO` |
| **Peer State Machine** | `UNKNOWN`, `DISCOVERING`, `CONNECTED` | `UNKNOWN`, `DISCOVERING`, `CONNECTED` |
| **Sender Filtering & Sequence** | Paired sender validation (`sender_id == TARGET_NODE_ID`) & wraparound sequence tracking | Paired sender validation (`sender_id == TARGET_NODE_ID`) & wraparound sequence tracking |
| **Analog Input Processing** | Kahan Oversampling + Single Min/Max Outlier Rejection + 1D Kalman Filter | Kahan Oversampling + Single Min/Max Outlier Rejection + 1D Kalman Filter |
| **Actuator Drive Resolution**| PWM Output on `D1` (GPIO 5, 0-1023) | Sub-microsecond FP4 Fixed-Point (1/16th $\mu s$) via `writeMicroseconds()` |
| **Digital IO Pin** | `D2` In $\rightarrow$ `D3` Out | `D2` In $\rightarrow$ `D3` Out |
| **Limit Switch Support** | N/A | `D6` (Min Limit) & `D7` (Max Limit) |
| **Limit Switch Safety**| N/A | Directional motion blocking (forbids movement toward triggered limit, allows reverse) |
| **Struct Size** | 12 bytes (24 hex characters) | 14 bytes (28 hex characters) |

---

## 2. Traffic Separation, Discovery State Machine & Safety

### Traffic Separation (Control vs. Discovery)
To prevent network channel saturation and broadcast storm degradation on growing painlessMesh networks:
- **CONTROL Traffic**: Sensor and servo motion payload frames are transmitted **EXCLUSIVELY via targeted unicast** (`mesh.sendSingle`). If disconnected, control payloads are suppressed.
- **DISCOVERY Traffic**: Broadcasts are strictly isolated to infrequent `HandshakeMessage` (`MSG_TYPE_HELLO`) frames sent only while in the `DISCOVERING` state.

### Active Handshake Discovery Protocol (`HELLO` / `HELLO_ACK`)
- Nodes run an active discovery task during startup / reconnects (`DISCOVERING` state) broadcasting lightweight `HandshakeMessage` (`MSG_TYPE_HELLO`).
- When the target node receives a `HELLO`, it records `targetMeshNodeId`, transitions to `CONNECTED`, and replies with a targeted `MSG_TYPE_HELLO_ACK`.

### Dynamic Topology Invalidation
- On mesh connection changes (`changedConnectionCallback()`), the node inspects `mesh.getNodeList()`.
- If the paired `targetMeshNodeId` is no longer connected in the mesh topology, `targetMeshNodeId` is cleared and `peerState` reverts to `DISCOVERING`.

### Paired Sender Filtering & Sequence Tracking
- Filters incoming packets to explicitly enforce `incoming.sender_id == TARGET_NODE_ID` and `incoming.target_id == MY_NODE_ID`.
- Evaluates incoming sequence counters using signed 32-bit arithmetic to safely handle integer wraparound and reject stale, duplicate, or reordered packets.

### Directional Limit Switch Safety Model (`esp8266_mesh_servo`)
- **MIN limit switch active (`D6`)**: Forbids further movement toward MIN (`targetUsFp4 < lastSafeUsFp4`), holding `lastSafeUsFp4`, while permitting movement toward MAX (`targetUsFp4 > lastSafeUsFp4`).
- **MAX limit switch active (`D7`)**: Forbids further movement toward MAX (`targetUsFp4 > lastSafeUsFp4`), holding `lastSafeUsFp4`, while permitting movement toward MIN (`targetUsFp4 < lastSafeUsFp4`).

---

## 3. Hardware Requirements & Wiring

### Pinout Summary
| Function | ESP8266 Pin | Notes |
|---|---|---|
| **Analog Sensor Input** | `A0` | ESP8266 ADC input pin (0 to 1.0V/3.3V, 10-bit resolution, DSP filtered) |
| **PWM Output / Servo Signal** | `D1` (GPIO 5) | PWM output in `esp8266_mesh_pair` or Microsecond Servo signal in `esp8266_mesh_servo` (544–2400 $\mu s$) |
| **Digital Input** | `D2` (GPIO 4) | Input pin read by sending node (`INPUT_PULLUP`) |
| **Digital Output** | `D3` (GPIO 0) | Output pin driven by received paired node digital state (`digitalWrite`) |
| **Min Limit Switch** | `D6` (GPIO 12) | Min position limit switch in `esp8266_mesh_servo` (Active LOW `INPUT_PULLUP`) |
| **Max Limit Switch** | `D7` (GPIO 13) | Max position limit switch in `esp8266_mesh_servo` (Active LOW `INPUT_PULLUP`) |
| **GND / VCC** | GND / 3V3 / 5V | Common power and ground (power Servos from external 5V if required) |

---

## 4. Firmware Setup & Compilation

```bash
# Compile Node 1 (MY_NODE_ID=1, TARGET_NODE_ID=2)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=1 -DTARGET_NODE_ID=2" \
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
ESP8266 Bi-directional Servo Mesh Node (Strict Unicast Control)
My Node ID: 1 -> Target Node ID: 2
Analog In: A0 (DSP Kahan+Kalman) | Servo Pin: GPIO 5 (D1) [544 - 2400 us]
Digital In: GPIO 4 (D2) | Digital Out: GPIO 0 (D3)
Min Limit Pin: GPIO 12 (D6) | Max Limit Pin: GPIO 13 (D7)
==================================================
[DISCOVERY #1] Sent HELLO broadcast for Target Node 2 (State: DISCOVERING)
[HANDSHAKE] Received HELLO_ACK from Target Node 2 (MeshID: 312847102). Transitioned to CONNECTED.
[TX #1] Filtered ADC: 512.35 | Target Pulse: 1472.25 us (FP4: 23556) | Unicast Sent: SUCCESS (Dest MeshID: 312847102)
[RX #1] From Node: 2 | Target Pulse: 1950.12 us (FP4: 31202) | Digital: 1 | Remote MinLim: 0 | Remote MaxLim: 0
[SERVO FP4] Requested: 1950.12 us (31202) -> Applied: 1950 us (31202 FP4) | LastSafe: 31202 FP4 (MinLim: 0, MaxLim: 0)
```
