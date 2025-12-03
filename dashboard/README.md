# Bioreactor Control Dashboard

Real-time web dashboard for monitoring and controlling the bioreactor system via MQTT.

## Features

- 📊 **Real-time Charts**: Live visualization of RPM, temperature, and pH
- 🎛️ **Remote Control**: Adjust setpoints via MQTT commands
- 📈 **Historical Data**: Last 20 data points for each parameter
- 🔄 **Auto-refresh**: Updates every second
- 🌐 **Network Monitoring**: WiFi and MQTT connection status
- 🎨 **Modern UI**: Clean, responsive interface with dark theme

## Installation

### 1. Install Python Dependencies

```bash
cd dashboard
pip install -r requirements.txt
```

### 2. Configure MQTT Credentials

The credentials are already configured in `.streamlit/secrets.toml`:

```toml
[hivemq]
broker = "444f9f1a6d4748f7a238f6d0a8068877.s1.eu.hivemq.cloud"
port = 8883
username = "Engineering-Challenges_Group5_Broker"
password = "HelloWorld:0"
topic_subscribe = "ucl/ec2/group5/ttgo/data"
topic_publish = "ucl/ec2/group5/ttgo/command"
```

## Running the Dashboard

```bash
cd dashboard
streamlit run app.py
```

The dashboard will automatically open in your default browser at `http://localhost:8501`

## Dashboard Pages

### 1. Welcome Page
- Project overview and system information
- Technical specifications
- Quick start guide
- Navigation instructions

### 2. Introduction Page
- System overview with all 3 parameters
- Real-time charts for RPM, temperature, and pH
- System status indicators
- Comprehensive system information

### 3. Stirring System Page
- Current and target RPM display
- Real-time RPM trend chart
- RPM control slider (0-1000 RPM)
- Statistics (latest, average, max, min)
- Bang-bang controller information

### 4. Heating System Page
- Current and target temperature display
- Real-time temperature trend chart
- Temperature control slider (20-45°C)
- Statistics (latest, average, max, min)
- Thermistor and controller information

### 5. pH System Page
- Current and target pH display
- Real-time pH trend chart
- pH control slider (4.0-10.0)
- Statistics (latest, average, max, min)
- Dual pump system information

## Data Flow

```
ESP32 DevKit (Sensors)
    ↓ UART (115200 baud, 100ms)
ESP32 TTGO (Gateway)
    ↓ MQTT over TLS (5 second interval)
HiveMQ Cloud Broker
    ↓ MQTT Subscribe
Streamlit Dashboard (Real-time display, 1 second refresh)
```

## MQTT Topics

### Subscribe (Data from ESP32 TTGO)
**Topic**: `ucl/ec2/group5/ttgo/data`

**Format**: JSON
```json
{
  "rpm": 450.5,
  "temp": 35.2,
  "ph": 7.1
}
```

### Publish (Commands to ESP32 TTGO)
**Topic**: `ucl/ec2/group5/ttgo/command`

**Format**: JSON (one or more parameters)
```json
{
  "target_rpm": 500,
  "target_temp": 35.0,
  "target_pH": 7.0
}
```

## Troubleshooting

### Charts Not Showing Data

1. **Check MQTT Status** (sidebar)
   - Should show "🟢 Working Good"
   - If "🔴 Connection Fail": Check internet connection
   - If "🟠 Waiting For Data": ESP32 TTGO not publishing

2. **Verify ESP32 TTGO**
   - Check if powered on
   - Verify WiFi connection (eduroam)
   - Check serial monitor for "Sending JSON: ..." messages

3. **Check MQTT Broker**
   - Login to HiveMQ Cloud console
   - Verify client connections
   - Check message activity on topics

### Placeholder Charts Displayed

This is normal when no data has been received yet. The dashboard will automatically switch to real data once MQTT messages arrive.

### Commands Not Working

1. Check MQTT publish status in sidebar (should be 🟢)
2. Verify ESP32 TTGO is subscribed to command topic
3. Check ESP32 serial monitor for "[UART RX]" messages
4. Ensure setpoint values are within valid ranges:
   - RPM: 0-1000
   - Temperature: 20-45°C
   - pH: 4.0-10.0

## Performance

- **Dashboard Refresh Rate**: 1 second
- **Chart Update**: Real-time (1 second intervals)
- **MQTT Latency**: < 500 ms
- **End-to-End Latency**: < 6 seconds (sensor → dashboard)
- **Data Retention**: Last 20 points per parameter

## Browser Compatibility

Tested and working on:
- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Development

### Adding New Features

1. Modify relevant page files (page_*.py)
2. Update mqtt_manager.py for new data fields
3. Add session state variables in initialize_session_state()
4. Update MQTT parsing in safe_update_session_state()

### Changing Theme

Edit `.streamlit/config.toml`:
```toml
[theme]
primaryColor = "#FF4B4B"  # Change accent color
backgroundColor = "#0E1117"  # Background color
secondaryBackgroundColor = "#262730"  # Card background
textColor = "#FAFAFA"  # Text color
```

## Dependencies

See `requirements.txt`:
- streamlit
- paho-mqtt
- pandas
- numpy
- requests

## Authors

UCL Engineering Challenges 2024/2025 - Group 5

## License

MIT License - Educational Project
