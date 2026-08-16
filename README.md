# ESP8266 Bi-Directional Sensor & Servo Mesh (DSP Enhanced)

An auto-organizing ESP8266 mesh network built using `painlessMesh` featuring sequence verification, unicast transport optimization, high-precision Digital Signal Processing (DSP) for analog inputs, sub-microsecond fixed-point resolution, directional limit switch protection, microsecond-level actuator control, and network rate limiting. This repository provides two firmware variants:

1. **`esp8266_mesh_pair`**: PWM & Digital IO version (transmits 1/sec periodic updates).
2. **`esp8266_mesh_servo`**: Servo version (50ms input polling, 200ms network rate limiting, sub-microsecond FP4 fixed-point pulse transmission, directional limit switch safety clamping).

Both versions allow creating paired ESP8266 nodes (configured with simple compile-time node IDs) that communicate bi-directionally over an ad-hoc Wi-Fi mesh network using compact hex-encoded binary packed structs.

---

## 1. Overview & Firmware Comparison

| Feature | `esp8266_mesh_pair` (PWM Version) | `esp8266_mesh_servo` (Servo Version) |
|---|---|---|
| **Input Polling Rate** | 1000 ms (1 Hz) | 50 ms (20 Hz local sampling, Wi-Fi PHY safe) |
| **Network Transmission Rate** | Periodic (1/sec = 1000 ms) | Rate-limited (max 1 packet per 200 ms / 5 Hz) |
| **Mesh Transport Mode** | Unicast `sendSingle()` (with broadcast discovery fallback) | Unicast `sendSingle()` (with broadcast discovery fallback) |
| **Packet Sequence Verification**| Wraparound-safe 32-bit sequence tracking & out-of-order rejection | Wraparound-safe 32-bit sequence tracking & out-of-order rejection |
| **Analog Input Processing** | Kahan Oversampling + Single Min/Max Outlier Rejection + 1D Kalman Filter | Kahan Oversampling + Single Min/Max Outlier Rejection + 1D Kalman Filter |
| **Actuator Drive Resolution**| PWM Output on `D1` (GPIO 5, 0-1023) | Sub-microsecond FP4 Fixed-Point (1/16th $\mu s$) via `writeMicroseconds()` |
| **Digital IO Pin** | `D2` In $\rightarrow$ `D3` Out | `D2` In $\rightarrow$ `D3` Out |
| **Limit Switch Support** | N/A | `D6` (Min Limit) & `D7` (Max Limit) |
| **Limit Switch Safety**| N/A | Directional motion blocking (forbids movement toward triggered limit, allows reverse) |
| **Struct Size** | 12 bytes (24 hex characters) | 14 bytes (28 hex characters) |

---

## 2. Unicast Routing, Sequence Control & Directional Safety

### Unicast `sendSingle()` Routing Optimization & Node ID Mapping
- Maps logical application IDs (`MY_NODE_ID`, `TARGET_NODE_ID`) to transport-level painlessMesh 32-bit node IDs (`mesh.getNodeId()`).
- Automatically learns the paired node's transport ID upon receiving incoming packets.
- Transmits using targeted unicast `mesh.sendSingle(targetMeshNodeId, payload)` once connected, drastically reducing mesh channel saturation compared to broadcast flooding.

### Wraparound-Safe Sequence Tracking
- Transmits an incrementing 32-bit sequence counter (`seq`).
- Evaluates incoming packets using signed 32-bit arithmetic (`isNewerSequence`).
- Drops old, duplicate, or reordered packets to prevent backward jumping or actuator jitter.

### Sub-Microsecond FP4 Fixed-Point Resolution
- Target pulse durations are represented in fixed-point **FP4 units** ($1\text{ unit} = 1/16\text{th }\mu s$).
- Preserves fractional ADC precision gained through Kahan oversampling and Kalman filtering across network transmissions.

### Directional Limit Switch Safety Model (`esp8266_mesh_servo`)
- **MIN limit switch active (`D6`)**: Forbids further movement toward MIN (`targetUsFp4 < lastSafeUsFp4`), holding `lastSafeUsFp4`, while permitting movement toward MAX (`targetUsFp4 > lastSafeUsFp4`).
- **MAX limit switch active (`D7`)**: Forbids further movement toward MAX (`targetUsFp4 > lastSafeUsFp4`), holding `lastSafeUsFp4`, while permitting movement toward MIN (`targetUsFp4 < lastSafeUsFp4`).
- Explicitly separates `requestedServoUsFp4` (desired position) from `appliedServoUsFp4` / `lastSafeUsFp4` (actually commanded hardware position).

---

## 3. Hardware Requirements & Wiring

### Hardware
- **2 or more ESP8266 boards** (NodeMCU V2, Wemos D1 Mini, ESP-12E, etc.)
- **Analog Sensors / Potentiometers** (connected to `A0`)
- **PWM Actuator / LED** (for PWM version on `D1`) OR **Servo Motor** (for Servo version on `D1`)
- **Digital Input Switch** (connected to `D2`)
- **Digital Output LED / Relay** (connected to `D3`)
- **Limit Switches** (for Servo version: `D6` = Min Limit, `D7` = Max Limit, active LOW with internal pull-ups)

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

### Required Arduino Libraries
- **Painless Mesh** (`painlessMesh` v1.5.7)
- **TaskScheduler** (`TaskScheduler` v3.8.5)
- **ArduinoJson** (`ArduinoJson` v6.21.3)
- **ESP Async TCP** (`ESPAsyncTCP` v2.0.0)
- **Servo** (`Servo` - included with ESP8266 Arduino Core)

### Option A: Using Arduino IDE
1. Open `esp8266_mesh_pair/esp8266_mesh_pair.ino` or `esp8266_mesh_servo/esp8266_mesh_servo.ino` in Arduino IDE.
2. Select **NodeMCU 1.0 (ESP-12E Module)** or your specific ESP8266 board.
3. Open `config.h` in the respective sketch folder and configure pairing IDs (`MY_NODE_ID`, `TARGET_NODE_ID`).
4. Upload to the respective ESP8266 nodes.

---

### Option B: Using `arduino-cli`

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

1. **Unicast & Sequence Verification Logs**:
   ```text
   ==================================================
   ESP8266 Bi-directional Servo Mesh Node
   My Node ID: 1 -> Target Node ID: 2
   Analog In: A0 (DSP Kahan+Kalman) | Servo Pin: GPIO 5 (D1) [544 - 2400 us]
   Digital In: GPIO 4 (D2) | Digital Out: GPIO 0 (D3)
   Min Limit Pin: GPIO 12 (D6) | Max Limit Pin: GPIO 13 (D7)
   ==================================================
   [MESH] New Connection, nodeId = 312847102 (Local Mesh Node ID = 312847101)
   [TX #1] Filtered ADC: 512.35 | Target Pulse: 1472.25 us (FP4: 23556) | Transport: UNICAST (sendSingle) (Dest MeshID: 312847102)
   [RX #1] From Node: 2 | Target Pulse: 1950.12 us (FP4: 31202) | Digital: 1 | Remote MinLim: 0 | Remote MaxLim: 0
   [SERVO FP4] Requested: 1950.12 us (31202) -> Applied: 1950 us (31202 FP4) | LastSafe: 31202 FP4 (MinLim: 0, MaxLim: 0)
   ```
