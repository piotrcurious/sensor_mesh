# ESP8266 Bi-Directional Sensor & Servo Mesh (DSP Enhanced)

An auto-organizing ESP8266 mesh network built using `painlessMesh` featuring high-precision Digital Signal Processing (DSP) for analog inputs, directional limit switch protection, microsecond-level actuator control, and network rate limiting. This repository provides two firmware variants:

1. **`esp8266_mesh_pair`**: PWM & Digital IO version (transmits 1/sec periodic updates).
2. **`esp8266_mesh_servo`**: Servo version (50ms input polling, 200ms network rate limiting, directional limit switch safety clamping, microsecond pulse control).

Both versions allow creating paired ESP8266 nodes (configured with simple compile-time node IDs) that communicate bi-directionally over an ad-hoc Wi-Fi mesh network using compact hex-encoded binary packed structs.

---

## 1. Overview & Firmware Comparison

| Feature | `esp8266_mesh_pair` (PWM Version) | `esp8266_mesh_servo` (Servo Version) |
|---|---|---|
| **Input Polling Rate** | 1000 ms (1 Hz) | 50 ms (20 Hz local sampling, Wi-Fi PHY safe) |
| **Network Transmission Rate** | Periodic (1/sec = 1000 ms) | Rate-limited (max 1 packet per 200 ms / 5 Hz) |
| **Analog Input Processing** | Kahan Oversampling + Single Min/Max Outlier Rejection + 1D Kalman Filter | Kahan Oversampling + Single Min/Max Outlier Rejection + 1D Kalman Filter |
| **Primary Actuator / Drive** | PWM Output on `D1` (GPIO 5, 0-1023) | Servo Motor via `writeMicroseconds()` on `D1` (544–2400 $\mu s$) |
| **Digital IO Pin** | `D2` In $\rightarrow$ `D3` Out | `D2` In $\rightarrow$ `D3` Out |
| **Limit Switch Support** | N/A | `D6` (Min Limit) & `D7` (Max Limit) |
| **Limit Switch Safety**| N/A | Directional motion blocking (forbids movement toward triggered limit, allows reverse) |
| **Struct Size** | 12 bytes (24 hex characters) | 14 bytes (28 hex characters) |

---

## 2. Directional Limit Switch Safety & Advanced DSP Pipeline

### Wi-Fi PHY & Mesh Stability
ESP8266 uses a shared multiplexed SAR ADC (`system_adc_read()`) that briefly pauses Wi-Fi radio reception during conversion. To ensure painlessMesh auto-discovery, AP/STA topology changes, and packet receptions function without RF dropouts:
- ADC oversampling uses 4 samples per read cycle interleaved with `optimistic_yield()` calls to give time back to the ESP8266 Wi-Fi stack.
- Input polling is scheduled at 50 ms (20 Hz) to allow seamless mesh network topology updates.

### Directional Limit Switch Safety Model (`esp8266_mesh_servo`)
Rather than jumping to extreme pulse limits when a limit switch triggers (which can push the actuator into physical end-stops), the firmware enforces **directional motion protection**:
- **MIN limit switch active (`D6`)**: Forbids further movement toward MIN (`targetUs < lastSafeUs`), holding `lastSafeUs`, while permitting movement toward MAX (`targetUs > lastSafeUs`).
- **MAX limit switch active (`D7`)**: Forbids further movement toward MAX (`targetUs > lastSafeUs`), holding `lastSafeUs`, while permitting movement toward MIN (`targetUs < lastSafeUs`).
- Explicitly separates `requestedServoUs` (desired position) from `appliedServoUs` / `lastSafeUs` (actually commanded hardware position).

### Advanced DSP Pipeline
To eliminate analog signal noise, ADC jitter, and floating-point accumulation drift while managing network bandwidth:

1. **Kahan Summation Oversampling**:
   - Takes 4 raw ADC samples per measurement cycle using the **Kahan Summation algorithm** to compensate for numerical floating-point precision loss.

2. **Accurate Trimmed-Mean Outlier Rejection**:
   - Subtracts exactly **one minimum** sample value and **one maximum** sample value from the accumulated sum and divides by 2 (4 - 2). This eliminates single-sample electrical spikes without biasing discrete ADC distributions.

3. **1D Kalman Filtering**:
   - Passes the averaged oversampled reading through a 1-dimensional Kalman Filter (`Q = 0.05`, `R = 4.0`) to produce ultra-smooth continuous motion control.

4. **Microsecond Resolution Servo Control (`writeMicroseconds`)**:
   - Directly maps high-precision filtered analog readings to target pulse durations in microseconds (`544 µs` to `2400 µs`).
   - Drives servos using `myServo.writeMicroseconds()`, unlocking maximum angular resolution.

5. **Compact Packed Binary Structs (Hex Encoded)**:
   - Data is packed into byte-aligned structs and hex-encoded to guarantee that null bytes (`0x00`) within multi-byte integers do not truncate the string during painlessMesh JSON transport.

   **Servo Mesh Struct (`ServoMeshMessage`):**
   ```cpp
   struct __attribute__((__packed__)) ServoMeshMessage {
       uint8_t  magic;            // Magic byte (0xB8)
       uint16_t sender_id;        // Sender node ID
       uint16_t target_id;        // Target node ID
       uint16_t target_us;        // Target pulse width in microseconds (544 - 2400 us)
       uint8_t  digital_value;    // Digital input state (0 or 1 from D2)
       uint8_t  min_limit_active; // Min limit switch state (1 = active/pressed from D6)
       uint8_t  max_limit_active; // Max limit switch state (1 = active/pressed from D7)
       uint32_t seq;              // Sequence counter
   };
   ```

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

### Folder Structure
```
.
├── esp8266_mesh_pair/       # PWM & Digital IO Mesh Firmware (DSP Enhanced)
│   ├── config.h
│   └── esp8266_mesh_pair.ino
├── esp8266_mesh_servo/      # Servo, Directional Safety & DSP Firmware
│   ├── config.h
│   └── esp8266_mesh_servo.ino
└── README.md
```

### Option A: Using Arduino IDE

1. Open `esp8266_mesh_pair/esp8266_mesh_pair.ino` or `esp8266_mesh_servo/esp8266_mesh_servo.ino` in Arduino IDE.
2. Select **NodeMCU 1.0 (ESP-12E Module)** or your specific ESP8266 board.
3. Open `config.h` in the respective sketch folder and configure pairing IDs for **Node 1**:
   ```cpp
   #define MY_NODE_ID 1
   #define TARGET_NODE_ID 2
   ```
4. Upload to the first ESP8266.
5. Swap the node IDs in `config.h` for **Node 2**:
   ```cpp
   #define MY_NODE_ID 2
   #define TARGET_NODE_ID 1
   ```
6. Upload to the second ESP8266.

---

### Option B: Using `arduino-cli`

You can compile and flash without modifying source files by supplying build flags:

#### Compiling the Servo Version (`esp8266_mesh_servo`):

```bash
# Compile Node 1 (MY_NODE_ID=1, TARGET_NODE_ID=2)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=1 -DTARGET_NODE_ID=2" \
  esp8266_mesh_servo

# Upload to Node 1
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_servo

# Compile Node 2 (MY_NODE_ID=2, TARGET_NODE_ID=1)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=2 -DTARGET_NODE_ID=2" \
  esp8266_mesh_servo

# Upload to Node 2
arduino-cli upload -p /dev/ttyUSB1 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_servo
```

---

## 5. Operation & Diagnostics

1. **Directional Safety Control**:
   - If the actuator moves toward minimum and triggers the MIN limit switch (`D6`), commands attempting to decrease pulse width further (`targetUs < lastSafeUs`) are blocked and held at `lastSafeUs`. Motion toward maximum (`targetUs > lastSafeUs`) remains permitted.

2. **Serial Monitor Logs (115200 baud)**:
   ```text
   ==================================================
   ESP8266 Bi-directional Servo Mesh Node (Wi-Fi PHY Safe)
   My Node ID: 1 -> Target Node ID: 2
   Analog In: A0 (DSP Kahan+Kalman) | Servo Pin: GPIO 5 (D1) [544 - 2400 us]
   Digital In: GPIO 4 (D2) | Digital Out: GPIO 0 (D3)
   Min Limit Pin: GPIO 12 (D6) | Max Limit Pin: GPIO 13 (D7)
   ==================================================
   [MESH] New Connection, nodeId = 312847102
   [TX #1] Filtered ADC: 512.35 | Target Pulse: 1472 us | Digital: 1 | MinLim: 0 | MaxLim: 0 (Hex: B801000200C00501000001000000)
   [RX #1] From Node: 2 | Target Pulse: 1950 us | Digital: 1 | Remote MinLim: 0 | Remote MaxLim: 0
   [SERVO us] Requested: 1950 us -> Applied: 1950 us | LastSafe: 1950 us (MinLim: 0, MaxLim: 0)
   ```
