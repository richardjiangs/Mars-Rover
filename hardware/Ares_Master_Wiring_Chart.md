# PROJECT ARES V2.2 - MASTER WIRING CHART

Read First:

- `[ 5V BUS ]`: This means tie the wire to your central 5V power line (Output of the LM2596).
- `[ 7.4V BUS ]`: This means tie the wire to your raw battery power line (After the switch).
- `[ GND BUS ]`: This means tie the wire to your common Ground line (All GNDs must connect together!).

### 1. POWER SYSTEM (The Heart)

Wire this section first using thicker wires to handle the current.

| Source Component | Source Pin | Wire Color | Destination Component | Destination Pin / GPIO | Function |
| --- | --- | --- | --- | --- | --- |
| Solar Panels (x3) | + (Positive) | 🔴 Red | 2S BMS Module | IN+ | Solar Charging Power |
| Solar Panels (x3) | - (Negative) | ⚫ Black | 2S BMS Module | IN- | Solar Ground |
| 18650 Battery Bay | + (7.4V) | 🔴 Red | 2S BMS Module | B+ | Battery Positive |
| 18650 Battery Bay | - (GND) | ⚫ Black | 2S BMS Module | B- | Battery Negative |
| 2S BMS Module | P+ (Out) | 🔴 Red | Master Switch | Pin 1 | Main Power Line |
| 2S BMS Module | P- (GND) | ⚫ Black | [ GND BUS ] | Common GND | Main System Ground |
| Master Switch | Pin 2 | 🔴 Red | [ 7.4V BUS ] | Common 7.4V | Switched Raw Power |
| [ 7.4V BUS ] | Common 7.4V | 🔴 Red | LM2596 Buck | IN+ | Feed power to Step-Down |
| [ GND BUS ] | Common GND | ⚫ Black | LM2596 Buck | IN- | Ground for Step-Down |
| LM2596 Buck | OUT+ (5V) | 🟠 Orange | [ 5V BUS ] | Common 5V | Provides Safe 5V Power |
| LM2596 Buck | OUT- (GND) | ⚫ Black | [ GND BUS ] | Common GND | Ground |

### 2. MOTOR CONTROL (The Legs)

Controls the 6-wheel skid steering.

| Source Component | Source Pin | Wire Color | Destination Component | Destination Pin / GPIO | Function |
| --- | --- | --- | --- | --- | --- |
| L298N (LEFT) | 12V IN | 🔴 Red | [ 7.4V BUS ] | Common 7.4V | Motor Power Left |
| L298N (LEFT) | GND | ⚫ Black | [ GND BUS ] | Common GND | Motor Ground Left |
| ESP32 (Main) | GPIO 32 | 🟣 Purple | L298N (LEFT) | IN1 | Left Forward Signal |
| ESP32 (Main) | GPIO 33 | 🟣 Purple | L298N (LEFT) | IN2 | Left Reverse Signal |
| Left Motors (x3) | Terminal 1 | 🟤 Brown | L298N (LEFT) | OUT1 | Power to Left Wheels |
| Left Motors (x3) | Terminal 2 | 🟤 Brown | L298N (LEFT) | OUT2 | Power to Left Wheels |
| L298N (RIGHT) | 12V IN | 🔴 Red | [ 7.4V BUS ] | Common 7.4V | Motor Power Right |
| L298N (RIGHT) | GND | ⚫ Black | [ GND BUS ] | Common GND | Motor Ground Right |
| ESP32 (Main) | GPIO 25 | 🟣 Purple | L298N (RIGHT) | IN3 | Right Forward Signal |
| ESP32 (Main) | GPIO 26 | 🟣 Purple | L298N (RIGHT) | IN4 | Right Reverse Signal |
| Right Motors (x3) | Terminal 1 | 🟤 Brown | L298N (RIGHT) | OUT3 | Power to Right Wheels |
| Right Motors (x3) | Terminal 2 | 🟤 Brown | L298N (RIGHT) | OUT4 | Power to Right Wheels |

### 3. SENSORS (The Eyes & Ears)

Powers the Gyroscope and the Radar Array.

| Source Component | Source Pin | Wire Color | Destination Component | Destination Pin / GPIO | Function |
| --- | --- | --- | --- | --- | --- |
| MPU6050 (Gyro) | VCC | 🟠 Orange | [ 5V BUS ] | Common 5V | Sensor Power |
| MPU6050 (Gyro) | GND | ⚫ Black | [ GND BUS ] | Common GND | Sensor Ground |
| MPU6050 (Gyro) | SDA | 🔵 Blue | ESP32 (Main) | GPIO 21 | I2C Data Line |
| MPU6050 (Gyro) | SCL | 🔵 Blue | ESP32 (Main) | GPIO 22 | I2C Clock Line |
| HC-SR04 (Radars) | VCC | 🟠 Orange | [ 5V BUS ] | Common 5V | Radar Power |
| HC-SR04 (Radars) | GND | ⚫ Black | [ GND BUS ] | Common GND | Radar Ground |
| HC-SR04 (Radars) | Trig | 🟢 Green | ESP32 (Main) | GPIO 19 | Radar Pulse Out |
| HC-SR04 (Radars) | Echo | 🟢 Green | ESP32 (Main) | GPIO 18 | Radar Echo Return |

> Note: Wire all 3 HC-SR04 sensors in parallel to the exact same pins. The ESP32 will read them as one giant front bumper.

### 4. DEPLOYABLE SERVOS & 3D CAMERA (The Advanced Payload)

Powers the folding solar wings, the 3D scanning servo, and the camera communication.

| Source Component | Source Pin | Wire Color | Destination Component | Destination Pin / GPIO | Function |
| --- | --- | --- | --- | --- | --- |
| Left Wing Servo | VCC / GND | 🟠/⚫ | [ 5V & GND BUS] | Common Buses | Servo Power |
| Left Wing Servo | PWM Signal | 🟡 Yellow | ESP32 (Main) | GPIO 13 | Angle Control |
| Right Wing Servo | VCC / GND | 🟠/⚫ | [ 5V & GND BUS] | Common Buses | Servo Power |
| Right Wing Servo | PWM Signal | 🟡 Yellow | ESP32 (Main) | GPIO 12 | Angle Control |
| Scanner Pan Servo | VCC / GND | 🟠/⚫ | [ 5V & GND BUS] | Common Buses | Servo Power |
| Scanner Pan Servo | PWM Signal | 🟡 Yellow | ESP32 (Main) | GPIO 4 | Pan Control (Sweep) |
| ESP32-CAM (Cam) | 5V | 🟠 Orange | [ 5V BUS ] | Common 5V | Camera Power |
| ESP32-CAM (Cam) | GND | ⚫ Black | [ GND BUS ] | Common GND | Camera Ground |
| ESP32-CAM (Cam) | TX | 🟣 Magenta | ESP32 (Main) | GPIO 16 (RX2) | Data TO Main Brain |
| ESP32-CAM (Cam) | RX | 🟣 Magenta | ESP32 (Main) | GPIO 17 (TX2) | Commands FROM Main |

### 5. MAIN BRAIN POWER

| Source Component | Source Pin | Wire Color | Destination Component | Destination Pin / GPIO | Function |
| --- | --- | --- | --- | --- | --- |
| ESP32 (Main) | VIN or 5V | 🟠 Orange | [ 5V BUS ] | Common 5V | Powers the Main Brain |
| ESP32 (Main) | GND | ⚫ Black | [ GND BUS ] | Common GND | Brain Ground |

### 6. OPTIONAL THERMAL MONITORING & INSULATION NOTES

Passive insulation has no electrical wiring, but the optional temperature sensor can be wired as follows.

| Source Component | Source Pin | Wire Color | Destination Component | Destination Pin / GPIO | Function |
| --- | --- | --- | --- | --- | --- |
| DS18B20 Temp Sensor | VCC | 🟠 Orange | ESP32 (Main) | 3V3 | Sensor Power, use 3.3V not 5V |
| DS18B20 Temp Sensor | GND | ⚫ Black | [ GND BUS ] | Common GND | Sensor Ground |
| DS18B20 Temp Sensor | DATA | 🟡 Yellow | ESP32 (Main) | GPIO 27 | OneWire Temperature Data |
| 4.7kΩ Resistor | One End | 🟡 Yellow | ESP32 (Main) | GPIO 27 | Pull-up for DS18B20 DATA |
| 4.7kΩ Resistor | Other End | 🟠 Orange | ESP32 (Main) | 3V3 | Pull-up to 3.3V |

Thermal construction notes:

- Put the ESP32, battery bay, and camera-related connectors inside a central insulated electronics box.
- Use EVA foam, thin sponge, foam board, or corrugated cardboard as insulation.
- Do not let aluminum foil, metallic tape, screws, or wire ends touch battery terminals or PCB solder joints.
- Do not fully wrap the L298N heat sink or LM2596 in foam during motor tests; they need a path to release heat.
- No active heater is included in the baseline wiring. If a heater is added later, it must have independent protection, a fuse, temperature control, and explicit user approval.

