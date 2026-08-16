# ESP8266 Bi-Directional Sensor & Servo Mesh

An auto-organizing ESP8266 mesh network built using `painlessMesh`. This repository provides two firmware variants:

1. **`esp8266_mesh_pair`**: PWM & Digital IO version (transmits 1/sec periodic updates).
2. **`esp8266_mesh_servo`**: Servo version (event-driven updates on state changes, limit switch support, and local motion clamping).

Both versions allow creating paired ESP8266 nodes (configured with simple compile-time node IDs) that communicate bi-directionally over an ad-hoc Wi-Fi mesh network using compact hex-encoded binary packed structs.

---

## 1. Overview & Firmware Comparison

| Feature | `esp8266_mesh_pair` (PWM Version) | `esp8266_mesh_servo` (Servo Version) |
|---|---|---|
| **Transmission Trigger** | Periodic timer (1/sec = 1000 ms) | Event-driven (immediate on input change) + 5s heartbeat |
| **Analog Input Pin** | `A0` (0–1023) | `A0` (0–1023) |
| **Primary Actuator** | PWM Output on `D1` (GPIO 5) | Servo Motor on `D1` (GPIO 5, 0–180°) |
| **Digital IO Pin** | `D2` In $\rightarrow$ `D3` Out | `D2` In $\rightarrow$ `D3` Out |
| **Limit Switch Support** | N/A | `D6` (Min Limit) & `D7` (Max Limit) |
| **Limit Switch Behavior**| N/A | Transmitted to paired node & clamps local servo movement |
| **Struct Size** | 12 bytes (24 hex characters) | 14 bytes (28 hex characters) |

---

## 2. Key Features

1. **Auto-Organizing Mesh Network**:
   - Built on `painlessMesh`, nodes dynamically form an ad-hoc Wi-Fi mesh network without requiring a router or central access point.
   - Any node automatically relays messages intended for other nodes across the mesh.

2. **Compile-Time Node Pairing**:
   - Nodes are configured with `MY_NODE_ID` and `TARGET_NODE_ID` via `#define` or build flags.
   - Example:
     - **Node 1**: `MY_NODE_ID = 1`, `TARGET_NODE_ID = 2`
     - **Node 2**: `MY_NODE_ID = 2`, `TARGET_NODE_ID = 1`

3. **Compact Packed Binary Message Struct (Hex Encoded)**:
   - Data is packed into byte-aligned structs and hex-encoded to guarantee that null bytes (`0x00`) within multi-byte integers do not truncate the string during painlessMesh JSON transport.

   **Servo Mesh Struct (`ServoMeshMessage`):**
   ```cpp
   struct __attribute__((__packed__)) ServoMeshMessage {
       uint8_t  magic;            // Magic byte (0xB7)
       uint16_t sender_id;        // Sender node ID
       uint16_t target_id;        // Target node ID
       uint16_t sensor_value;     // Raw analog input (0 - 1023 from A0)
       uint8_t  digital_value;    // Digital input state (0 or 1 from D2)
       uint8_t  min_limit_active; // Min limit switch state (1 = active/pressed from D6)
       uint8_t  max_limit_active; // Max limit switch state (1 = active/pressed from D7)
       uint32_t seq;              // Sequence counter
   };
   ```

4. **Event-Driven Transmission & Local Limit Switch Clamping**:
   - The Servo version monitors inputs every 50 ms and sends packet updates instantly whenever an analog value changes beyond `ANALOG_CHANGE_THRESHOLD` (8 counts), or whenever digital inputs / limit switch states change.
   - Limit switches on `D6` (Min Limit) and `D7` (Max Limit) actively clamp the local servo angle (preventing over-rotation past physical end-stops) while transmitting switch states across the mesh to the remote node.

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
| **Analog Sensor Input** | `A0` | ESP8266 ADC input pin (0 to 1.0V/3.3V, 10-bit resolution: 0–1023) |
| **PWM Output / Servo Signal** | `D1` (GPIO 5) | PWM output in `esp8266_mesh_pair` or Servo PWM signal in `esp8266_mesh_servo` |
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
├── esp8266_mesh_pair/       # PWM & Digital IO Mesh Firmware
│   ├── config.h
│   └── esp8266_mesh_pair.ino
├── esp8266_mesh_servo/      # Servo, Limit Switch & Event-Driven Mesh Firmware
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
  --build-property "build.extra_flags=-DMY_NODE_ID=2 -DTARGET_NODE_ID=1" \
  esp8266_mesh_servo

# Upload to Node 2
arduino-cli upload -p /dev/ttyUSB1 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_servo
```

---

## 5. Operation & Diagnostics

1. **Event-Driven Updates**:
   - Turn the potentiometer connected to `A0` on Node 1: Node 1 detects the change immediately and transmits an updated hex-encoded frame.
   - Node 2 receives the frame and sets its Servo output pin `D1` accordingly.
   - Toggle switch `D2` on Node 1: Node 1 sends an update immediately, setting output `D3` on Node 2.

2. **Limit Switch Clamping**:
   - When the physical motion reaches the minimum limit switch (`D6`), pressing the switch forces local servo angle to `0°` regardless of requested higher inputs.
   - When the maximum limit switch (`D7`) is pressed, local servo angle is clamped to `180°`.
   - Limit switch activation events trigger an instant mesh update to inform the paired node.

3. **Serial Monitor Logs (115200 baud)**:
   ```text
   ==================================================
   ESP8266 Bi-directional Servo & Sensor Mesh Node
   My Node ID: 1 -> Target Node ID: 2
   Analog In: A0 | Servo Pin: GPIO 5 (D1)
   Digital In: GPIO 4 (D2) | Digital Out: GPIO 0 (D3)
   Min Limit Pin: GPIO 12 (D6) | Max Limit Pin: GPIO 13 (D7)
   ==================================================
   [MESH] New Connection, nodeId = 312847102
   [TX #1] Analog: 512 | Digital: 1 | MinLim: 0 | MaxLim: 0 -> Target Node: 2 (Hex: B701000200000201000001000000)
   [RX #1] From Node: 2 | Sensor: 780 | Digital: 1 | Remote MinLim: 0 | Remote MaxLim: 0
   [SERVO] Requested Analog: 780 -> Target Angle: 137 deg | Clamped Angle: 137 deg (MinLim: 0, MaxLim: 0)
   ```
