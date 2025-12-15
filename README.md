# Bioreactor Simulator

A real-time bioreactor control and monitoring system built for UCL Engineering Challenges 2025. This system provides comprehensive control of stirring, temperature, and pH parameters through a dual ESP32 architecture with cloud-based monitoring.

(If you are a UCL student checking this out, hello there. You're in for a wild ride.)

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Components](#software-components)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Control Parameters](#control-parameters)
- [Data Flow](#data-flow)
- [Project Structure](#project-structure)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

This bioreactor simulator implements a complete control system for managing critical bioreactor parameters:

- **Stirring Speed**: 0-1000 RPM with bang-bang control
- **Temperature**: 20-35°C with thermistor sensing and heater control
- **pH Level**: 4.0-10.0 with dual pump control (acid/base dosing)

The system features a dual-board architecture with local display and cloud-based web dashboard for real-time monitoring and control.

## System Architecture

```
┌─────────────────┐
│  ESP32 DevKit   │  ← Main Controller
│  (Control Loop) │     • Sensors & Actuators
│                 │     • Bang-Bang Controllers
│                 │     • 10ms Control Loop
└────────┬────────┘
         │ UART (115200 baud)
         │ Serial2 @ 100ms
         ▼
┌─────────────────┐
│  ESP32 TTGO     │  ← Gateway & Display
│  (T-Display)    │     • TFT Display
│                 │     • MQTT Bridge
│                 │     • WiFi Gateway
└────────┬────────┘
         │ MQTT over WiFi
         │ HiveMQ Cloud
         ▼
┌─────────────────┐
│   Streamlit     │  ← Web Dashboard
│   Dashboard     │     • Real-time Charts
│                 │     • Parameter Control
│                 │     • Data Logging
└─────────────────┘
```

## Features

### ESP32 DevKit (Main Controller)
- 10ms control loop for precise parameter regulation
- Bang-bang control with hysteresis for all three systems
- Hall sensor-based RPM measurement (70 pulses/revolution)
- Thermistor temperature sensing with Steinhart-Hart conversion
- Analog pH sensor with voltage-based calibration
- Dual peristaltic pump control (acid/base)
- Automatic pump priming sequence at startup
- Rolling average filters for sensor noise reduction
- UART data transmission to TTGO display

### ESP32 TTGO (Gateway & Display)
- 135×240 pixel TFT color display
- Real-time parameter visualization
- MQTT client with HiveMQ Cloud integration
- WiFi connectivity management
- Bidirectional communication (DevKit ↔ Cloud)
- Command reception from dashboard
- 5-second MQTT publish interval

### Streamlit Dashboard
- Multi-page web interface
- Real-time data visualization with Plotly charts
- Independent control pages for each parameter
- Realistic value smoothing with configurable time constants
- Background data logging (up to 10,000 entries)
- JSON export functionality
- Network and MQTT status monitoring
- Session-based data persistence

## Hardware Components

### ESP32 Nano Components
- **ESP32-WROOM-32** microcontroller
- **Hall Effect Sensor**: Stirring motor RPM measurement
- **Thermistor (10kΩ)**: Temperature sensing with voltage divider
- **pH Sensor**: Analog voltage output (0-3.3V)
- **Motor Driver**: PWM control for stirring motor (GPIO 27)
- **Heater Element**: PWM control at 39% max power (GPIO 25)
- **Peristaltic Pumps**: Acid pump (GPIO 9), Base pump (GPIO 8)

### ESP32 TTGO Components
- **ESP32-WROOM-32** with integrated TFT display
- **ST7789 TFT Display**: 135×240 RGB 16-bit color
- **WiFi Module**: 2.4GHz 802.11 b/g/n

### Pin Configuration

#### ESP32 Nano
```
GPIO 34  → Hall Sensor (ADC1_CH6)
GPIO 35  → Thermistor (ADC1_CH7)
GPIO 3   → pH Sensor (ADC1_CH3)
GPIO 27  → Stirring Motor (PWM)
GPIO 25  → Heater Element (PWM)
GPIO 9   → Acid Pump (Digital Out)
GPIO 8   → Base Pump (Digital Out)
GPIO 16  → UART2 TX (to TTGO)
GPIO 17  → UART2 RX (from TTGO)
```

## Software Components

### ESP32 Nano Firmware
- **Language**: C++ (Arduino Framework)
- **IDE**: Arduino IDE / PlatformIO
- **Key Libraries**:
  - `Arduino.h` - Core functionality
  - `HardwareSerial.h` - UART communication
- **Control Algorithms**:
  - Bang-bang control with hysteresis
  - Steinhart-Hart temperature conversion
  - Rolling average filters (100 samples for RPM/temp)
  - Voltage-to-pH calibration

### ESP32 TTGO Firmware
- **Language**: C++ (Arduino Framework)
- **Key Libraries**:
  - `TFT_eSPI.h` - Display driver
  - `WiFi.h` - WiFi connectivity
  - `PubSubClient.h` - MQTT client
  - `ArduinoJson.h` - JSON parsing
- **Features**:
  - TFT graphics rendering
  - MQTT publish/subscribe
  - WiFi auto-reconnect
  - Command parsing

### Streamlit Dashboard
- **Language**: Python 3.12+
- **Framework**: Streamlit
- **Key Libraries**:
  - `streamlit` - Web framework
  - `paho-mqtt` - MQTT client
  - `plotly` - Interactive charts
  - `pandas` - Data manipulation
- **Pages**:
  1. Introduction - System overview
  2. Stirring System - RPM control
  3. Heating System - Temperature control
  4. pH System - pH control
  5. Data Logging - Export functionality

## Installation

### Prerequisites
- Arduino IDE 2.0+ or PlatformIO
- Python 3.12 or higher
- HiveMQ Cloud account (free tier)
- Git

### 1. Clone Repository
```bash
git clone https://github.com/your-username/Bioreactor-Simulator.git
cd Bioreactor-Simulator
```

### 2. ESP32 DevKit Setup

#### Install Required Libraries
In Arduino IDE:
- Go to **Sketch → Include Library → Manage Libraries**
- Search and install: (None required - uses built-in libraries)

#### Configure Credentials
1. Copy credentials template:
```bash
cd esp32_devkit
cp credentials_template.h credentials.h
```

2. Edit `credentials.h`:
```cpp
// No WiFi credentials needed for DevKit
// This board only uses UART communication
```

#### Upload Firmware
1. Open `esp32_devkit/esp32_devkit.ino` in Arduino IDE
2. Select **Tools → Board → ESP32 Dev Module**
3. Select correct COM port
4. Click **Upload**

### 3. ESP32 TTGO Setup

#### Install Required Libraries
In Arduino IDE:
- TFT_eSPI
- PubSubClient
- ArduinoJson

#### Configure TFT_eSPI
Edit `Arduino/libraries/TFT_eSPI/User_Setup_Select.h`:
```cpp
#include <User_Setups/Setup25_TTGO_T_Display.h>
```

#### Configure Credentials
1. Copy credentials template:
```bash
cd esp32_ttgo
cp credentials_template.h credentials.h
```

2. Edit `credentials.h`:
```cpp
// WiFi credentials
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

// HiveMQ Cloud credentials
const char* mqtt_server = "your-cluster.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "your_username";
const char* mqtt_password = "your_password";

// MQTT topics
const char* topic_publish = "path/to/publish/topic/";
const char* topic_subscribe = "path/to/subscribe/topic";
```

#### Upload Firmware
1. Open `esp32_ttgo/esp32_ttgo.ino` in Arduino IDE
2. Select **Tools → Board → ESP32 Dev Module**
3. Select correct COM port
4. Click **Upload**

### 4. Streamlit Dashboard Setup

#### Create Virtual Environment
```bash
cd dashboard
python -m venv .venv

# Activate virtual environment
# Windows:
.venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate
```

#### Install Dependencies
```bash
pip install -r requirements.txt
```

#### Configure Secrets
Create `.streamlit/secrets.toml`:
```toml
[hivemq]
broker = "your-cluster.hivemq.cloud"
port = 8883
username = "your_username"
password = "your_password"
topic_subscribe = "path/to/subscribe/topic"
topic_publish = "path/to/publish/topic"
```

#### Run Dashboard
```bash
streamlit run app.py
```

Dashboard will open at `http://localhost:8501`

## Configuration

### Control Parameters

#### Stirring System
```cpp
// esp32_devkit/esp32_devkit.ino
int targetRPM = 500;              // Default: 500 RPM
const int hysteresisRPM = 20;     // ±20 RPM deadband
const int motorPin = 27;          // PWM output
const int hallSensorPin = 34;     // Hall sensor input
```

#### Heating System
```cpp
// esp32_devkit/heating.h
float targetTemp = 35.0;          // Default: 35°C
const float hysteresis = 0.5;     // ±0.5°C deadband
const int heaterPin = 25;         // PWM output
const int heaterLimit = 550;      // 39% max power (550/1023)
```

#### pH System
```cpp
// esp32_devkit/ph_control.h
float targetPH = 5.0;             // Default: pH 5.0
const float voltageTolerance = 0.003;  // ±0.003V deadband
const unsigned long acidPrimeDuration = 76000;   // 76 seconds
const unsigned long basePrimeDuration = 14000;   // 14 seconds
```

### MQTT Configuration

#### Publish Interval (TTGO)
```cpp
// esp32_ttgo/esp32_ttgo.ino
const unsigned long publishInterval = 5000;  // 5 seconds
```

#### Data Format
Published JSON (TTGO → Dashboard):
```json
{
  "rpm": 500,
  "temp": 35.2,
  "ph": 5.05
}
```

Command JSON (Dashboard → TTGO):
```json
{
  "target_rpm": 600,
  "target_temp": 37.0,
  "target_pH": 5.5
}
```

## Usage

### Starting the System

1. **Power on ESP32 DevKit**
   - Pump priming sequence runs automatically:
     - Phase 0 (0-76s): Acid pump priming
     - Phase 1 (76-90s): Base pump priming
     - Phase 2 (90s+): Normal operation
   - Control loops start immediately

2. **Power on ESP32 TTGO**
   - Connects to WiFi
   - Establishes MQTT connection
   - Displays real-time parameters on TFT screen

3. **Open Dashboard**
   ```bash
   cd dashboard
   streamlit run app.py
   ```
   - Navigate to `http://localhost:8501`
   - View Introduction page for system overview
   - Use sidebar to access individual control pages

### Controlling Parameters

#### From Dashboard
1. Navigate to desired parameter page (Stirring/Heating/pH)
2. Adjust slider to set target value
3. Click **SEND** button
4. Dashboard sends MQTT command to TTGO
5. TTGO forwards command via UART to DevKit
6. DevKit updates target setpoint

#### Response Times (Dashboard Smoothing)
- **Stirring**: Reaches target in ~25 seconds
- **Temperature**: Reaches target in ~2.5 minutes
- **pH**: Reaches target in ~2.5 minutes

### Data Logging

Data is automatically logged in the background on all pages:

1. Navigate to **Data Logging** page
2. View real-time log statistics
3. Click **Download as JSON** to export data
4. Click **Clear All Logs** to reset logging

Log entry format:
```json
{
  "timestamp": "2024-12-15 21:30:45.123",
  "unix_time": 1702675845.123,
  "rpm": 498.5,
  "temperature": 35.2,
  "ph": 5.03
}
```

## Control Parameters

### Stirring System
| Parameter | Value | Description |
|-----------|-------|-------------|
| Range | 0-1000 RPM | Bang-bang control |
| Hysteresis | ±20 RPM | Deadband to prevent oscillation |
| PWM Resolution | 10-bit (0-1023) | Motor speed control |
| PWM Frequency | 5 kHz | Motor driver frequency |
| Sensor | Hall effect | 70 pulses/revolution |
| Control Loop | 10 ms | 100 Hz update rate |

### Heating System
| Parameter | Value | Description |
|-----------|-------|-------------|
| Range | 20-45°C | Safe operating range |
| Hysteresis | ±0.5°C | Deadband to prevent oscillation |
| PWM Resolution | 10-bit (0-1023) | Heater power control |
| Power Limit | 39% (550/1023) | Safety limit |
| Sensor | 10kΩ Thermistor | Steinhart-Hart conversion |
| Control Loop | 10 ms | 100 Hz update rate |

### pH System
| Parameter | Value | Description |
|-----------|-------|-------------|
| Range | 4.0-10.0 pH | Safe operating range |
| Deadband | ±0.2 pH | Prevents overdosing |
| Pump Pulse | 300 ms | Dosing pulse duration |
| Cooldown | 5 seconds | Time between doses |
| Sensor | Analog pH probe | Voltage-based calibration |
| Calibration | pH = (V + 1.4) / 0.47 | Voltage to pH conversion |

## Data Flow

### Real-time Control Loop

```
DevKit (10ms loop):
  1. Read sensors (Hall, Thermistor, pH)
  2. Apply rolling average filters
  3. Run bang-bang controllers
  4. Update PWM outputs
  5. Control pump pulses

DevKit → TTGO (100ms, UART):
  Format: "RPM,TEMP,PH\n"
  Example: "498.5,35.2,5.03\n"

TTGO → Cloud (5s, MQTT):
  Topic: ucl/ec2/group5/ttgo/data
  Payload: {"rpm":498,"temp":35.2,"ph":5.03}

Cloud → TTGO (on command, MQTT):
  Topic: ucl/ec2/group5/ttgo/command
  Payload: {"target_rpm":600}

TTGO → DevKit (on command, UART):
  Format: "CMD:RPM:600\n"
  DevKit updates targetRPM variable
```

### Dashboard Smoothing

The dashboard applies first-order low-pass filtering to create realistic system dynamics:

```python
# Exponential Moving Average
α = Δt / (τ + Δt)
filtered_value = α × (setpoint + noise) + (1 - α) × previous_value

# Time Constants (τ)
Stirring:    τ = 5 seconds
Temperature: τ = 30 seconds
pH:          τ = 30 seconds

# Noise Ranges
Stirring:    ±10 RPM
Temperature: ±0.5°C
pH:          ±0.15 pH
```