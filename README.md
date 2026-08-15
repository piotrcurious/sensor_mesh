# ESP8266 Bi-Directional Sensor & Digital Mesh

An auto-organizing ESP8266 mesh network built using `painlessMesh`. This firmware allows creating paired ESP8266 nodes (configured with simple compile-time node IDs). Each node reads its analog sensor pin (`A0`) and digital input pin (`D2`) once every second (1/sec transmission rate) and transmits the reading across the mesh network using a compact binary packed struct encoded as hex. The paired receiving node uses the received values to drive a PWM output pin (`D1`) and a digital output pin (`D3`). The communication is fully bi-directional.

---

## Key Features

1. **Auto-Organizing Mesh Network**:
   - Built on `painlessMesh`, nodes dynamically form an ad-hoc Wi-Fi mesh network without requiring a router or central access point.
   - Any node automatically relays messages intended for other nodes across the mesh.

2. **Compile-Time Node Pairing**:
   - Nodes are configured with `MY_NODE_ID` and `TARGET_NODE_ID` via `#define` or build flags.
   - Example:
     - **Node 1**: `MY_NODE_ID = 1`, `TARGET_NODE_ID = 2`
     - **Node 2**: `MY_NODE_ID = 2`, `TARGET_NODE_ID = 1`

3. **Compact Packed Binary Message Struct (Hex Encoded)**:
   - Data is packed into a byte-aligned struct:
     ```cpp
     struct __attribute__((__packed__)) SensorMessage {
         uint8_t  magic;          // Magic byte (0xA5) for integrity validation
         uint16_t sender_id;      // Sending node ID
         uint16_t target_id;      // Target node ID
         uint16_t sensor_value;   // Raw analog sensor value (0 - 1023 from A0)
         uint8_t  digital_value;  // Digital input state (0 or 1 from D2)
         uint32_t seq;            // Message sequence number
     };
     ```
   - Binary size is **12 bytes** (24 ASCII hex characters). Hex encoding guarantees that null bytes (`0x00`) within multi-byte integers do not truncate the string during painlessMesh JSON transport.

4. **1 Hz Transmission Rate**:
   - Managed via non-blocking `TaskScheduler` (`taskSendSensorData`) executing every 1000 ms.

5. **Bi-directional Analog-to-PWM & Digital IO Control**:
   - **Analog Input (`A0`) $\rightarrow$ PWM Output (`D1`)**: Reads `A0` (0–1023) and drives PWM duty cycle on the paired receiving node's `D1` (GPIO 5).
   - **Digital Input (`D2`) $\rightarrow$ Digital Output (`D3`)**: Reads `D2` (with pull-up) and sets digital output state on the paired receiving node's `D3` (GPIO 0).

---

## Hardware Requirements & Wiring

### Hardware
- **2 or more ESP8266 boards** (NodeMCU V2, Wemos D1 Mini, ESP-12E, etc.)
- **Analog Sensors / Potentiometers** (connected to `A0`)
- **Actuators / LEDs / Transistors** (connected to PWM pin `D1` / GPIO 5)
- **Switches / Buttons / Sensors** (connected to Digital Input `D2` / GPIO 4)
- **Digital Devices / Relay / LEDs** (connected to Digital Output `D3` / GPIO 0)

### Pinout
| Function | ESP8266 Pin | Notes |
|---|---|---|
| **Analog Sensor Input** | `A0` | ESP8266 ADC input pin (0 to 1.0V or 3.3V depending on board divider, 10-bit resolution: 0–1023) |
| **PWM Output** | `D1` (GPIO 5) | Output pin driven by received paired node sensor value (`analogWrite`) |
| **Digital Input** | `D2` (GPIO 4) | Input pin read by sending node (`INPUT_PULLUP`) |
| **Digital Output** | `D3` (GPIO 0) | Output pin driven by received paired node digital state (`digitalWrite`) |
| **GND / VCC** | GND / 3V3 or 5V | Common power and ground |

---

## Firmware Setup & Compilation

### Required Arduino Libraries
- **Painless Mesh** (`painlessMesh` v1.5.7)
- **TaskScheduler** (`TaskScheduler` v3.8.5)
- **ArduinoJson** (`ArduinoJson` v6.21.3)
- **ESP Async TCP** (`ESPAsyncTCP` v2.0.0)

### Folder Structure
```
.
├── esp8266_mesh_pair/
│   ├── config.h               # Node configuration & packed binary struct
│   └── esp8266_mesh_pair.ino  # Main sketch file with painlessMesh logic
└── README.md                  # System documentation
```

### Option A: Using Arduino IDE

1. Open `esp8266_mesh_pair/esp8266_mesh_pair.ino` in Arduino IDE.
2. Select **NodeMCU 1.0 (ESP-12E Module)** or your specific ESP8266 board.
3. Open `config.h` and configure the pairing IDs for **Node 1**:
   ```cpp
   #define MY_NODE_ID 1
   #define TARGET_NODE_ID 2
   ```
4. Upload to the first ESP8266.
5. In `config.h`, swap the node IDs for **Node 2**:
   ```cpp
   #define MY_NODE_ID 2
   #define TARGET_NODE_ID 1
   ```
6. Upload to the second ESP8266.

---

### Option B: Using `arduino-cli`

You can compile and upload without editing files by using build flags:

```bash
# Compile Node 1 (MY_NODE_ID=1, TARGET_NODE_ID=2)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=1 -DTARGET_NODE_ID=2" \
  esp8266_mesh_pair

# Upload to Node 1 (replace /dev/ttyUSB0 with your port)
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_pair

# Compile Node 2 (MY_NODE_ID=2, TARGET_NODE_ID=1)
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 \
  --build-property "build.extra_flags=-DMY_NODE_ID=2 -DTARGET_NODE_ID=1" \
  esp8266_mesh_pair

# Upload to Node 2 (replace /dev/ttyUSB1 with your port)
arduino-cli upload -p /dev/ttyUSB1 --fqbn esp8266:esp8266:nodemcuv2 esp8266_mesh_pair
```

---

## Mesh Network Operation & Diagnostics

1. **Powering On**:
   - Both nodes power up and initialize painlessMesh with `MESH_PREFIX` ("`ESP8266_SensorMesh`") and `MESH_PASSWORD`.
   - The nodes auto-discover each other and form the mesh network automatically.

2. **Data Flow**:
   - **Node 1** reads its potentiometer on `A0` and switch state on `D2` every 1 second.
   - **Node 1** hex-encodes the 12-byte `SensorMessage` containing `sender_id = 1`, `target_id = 2`, `sensor_value`, and `digital_value`.
   - **Node 2** receives the packet, hex-decodes the struct, recognizes `target_id == 2`, adjusts PWM output on `D1`, and updates digital output state on `D3`.
   - Simultaneously, **Node 2** reads its sensor (`A0`) and digital input (`D2`) and transmits to **Node 1** (`target_id = 1`), which updates PWM on `D1` and digital output on `D3`.

3. **Multi-hop / Mesh Relay**:
   - If additional nodes exist in the mesh, painlessMesh automatically routes and relays packets through intermediate nodes if Node 1 and Node 2 are out of direct range.

4. **Serial Monitor Logs (115200 baud)**:
   ```text
   ==================================================
   ESP8266 Bi-directional Sensor Mesh Node
   My Node ID: 1 -> Target Node ID: 2
   Analog In: A0 | PWM Out: GPIO 5 (D1)
   Digital In: GPIO 4 (D2) | Digital Out: GPIO 0 (D3)
   ==================================================
   [MESH] New Connection, nodeId = 312847102
   [TX #1] Analog A0: 512 | Digital D2: 1 -> Target Node: 2 (Struct: 12 bytes, Hex Payload: A50100020000020101000000)
   [RX #1] From Node: 2 | Analog: 780 -> PWM Duty: 780/1023 | Digital D2 -> D3: 1 (Mesh NodeID: 312847102)
   ```
