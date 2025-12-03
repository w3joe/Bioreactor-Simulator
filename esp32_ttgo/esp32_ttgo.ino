#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
TFT_eSprite face = TFT_eSprite(&tft);

// Display dimensions
#define FACE_W 240
#define FACE_H 135

// Color definitions
#define TEXT_FG   TFT_WHITE
#define TEXT_BG   TFT_BLACK
#define LABEL_FG  TFT_CYAN
#define VALUE_FG  TFT_YELLOW
#define ERROR_FG  TFT_RED

// Time variables
float time_secs = 0;

// Load header after time_secs global variable has been created so it is in scope
#include "NTP_Time.h" // Attached to this sketch, see that tab for library needs

// ================= Load Credentials =================
#include "credentials.h"
#include "esp_eap_client.h"

// MQTT Clients
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Time for next screen update
uint32_t targetTime = 0;
#define UPDATE_INTERVAL_MS 100  // Update display every 100ms

// UART Communication for bioreactor data
// Connect ESP32_DevKit GPIO11 (TX) to ESP32_TTGO GPIO25 (RX)
// Connect ESP32_DevKit GPIO12 (RX) to ESP32_TTGO GPIO26 (TX)
// Connect GND of both boards together
#define UART_RX_PIN 25   // RX pin for Serial2 (receives sensor data from DevKit)
#define UART_TX_PIN 26   // TX pin for Serial2 (sends setpoints to DevKit)
#define UART_BAUD 115200

// Setpoints (updated from MQTT or default values)
float SETPOINT_RPM = 500.0;
float SETPOINT_TEMP = 35.0;
float SETPOINT_PH = 7.0;
bool setpointsUpdated = false;  // Flag to indicate new setpoints from MQTT

// Notification system (non-blocking)
bool showNotification = false;
unsigned long notificationStartTime = 0;
#define NOTIFICATION_DURATION_MS 2000  // Show notification for 2 seconds

#define MQTT_PUBLISH_INTERVAL_MS 5000   // Publish sensor data to MQTT every 5 seconds

// Bioreactor data received from ESP32 DevKit
struct BioreactorData {
  float targetRPM = 0.0;
  float measuredRPM = 0.0;
  float error = 0.0;
  int motorPWM = 0;
  float pH = 7.0;
  float pHError = 0.1;
  float temperature = 37.0;
  float tempError = 0.5;
  int heaterPWM = 0;
  bool dataValid = false;
  unsigned long lastUpdate = 0;
};

BioreactorData bioreactorData;
String uartBuffer = "";  // Buffer for incoming UART data

// UART debug variables
unsigned long lastUARTActivity = 0;
unsigned long lastDebugPrint = 0;
unsigned long bytesReceived = 0;
unsigned long packetsReceived = 0;
unsigned long parseErrors = 0;
unsigned long setpointsSent = 0;
unsigned long lastMqttPublish = 0;
unsigned long mqttPublishCount = 0;
#define DEBUG_INTERVAL_MS 5000  // Print debug info every 5 seconds
#define MAX_BYTES_PER_LOOP 128  // Process max 128 bytes per loop iteration (increased to prevent buffer overflow)

// =========================================================================
// WiFi Connection Functions
// =========================================================================
// =========================================================================
// MQTT Callback - Receives setpoint commands from MQTT
// Expected JSON format: {"target_rpm": 500, "target_temp": 35, "target_pH": 7}
// =========================================================================
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("[MQTT] Received command [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("[MQTT] JSON parse error: ");
    Serial.println(error.f_str());
    return;
  }

  // Update setpoints from MQTT commands
  bool updated = false;

  if (doc.containsKey("target_rpm")) {
    float newRPM = doc["target_rpm"];
    if (newRPM >= 0.0 && newRPM <= 1000.0) {
      SETPOINT_RPM = newRPM;
      Serial.print("[MQTT] ✓ Updated RPM setpoint: ");
      Serial.println(SETPOINT_RPM);
      updated = true;
    }
  }

  if (doc.containsKey("target_temp")) {
    float newTemp = doc["target_temp"];
    if (newTemp >= 20.0 && newTemp <= 45.0) {
      SETPOINT_TEMP = newTemp;
      Serial.print("[MQTT] ✓ Updated TEMP setpoint: ");
      Serial.println(SETPOINT_TEMP);
      updated = true;
    }
  }

  if (doc.containsKey("target_pH")) {
    float newPH = doc["target_pH"];
    if (newPH >= 0.0 && newPH <= 14.0) {
      SETPOINT_PH = newPH;
      Serial.print("[MQTT] ✓ Updated pH setpoint: ");
      Serial.println(SETPOINT_PH);
      updated = true;
    }
  }

  // If any setpoint was updated, set flag to trigger UART transmission
  if (updated) {
    setpointsUpdated = true;

    // Trigger non-blocking notification
    showNotification = true;
    notificationStartTime = millis();

    // Display update notification on screen
    tft.fillRect(0, 110, 240, 25, TFT_BLUE);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(5, 115);
    tft.setTextSize(1);
    tft.print("MQTT: Setpoints Updated");
  }
}

// =========================================================================
// MQTT Reconnection
// =========================================================================
void mqtt_reconnect() {
  while (!mqttClient.connected()) {
    // Check WiFi first
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connection lost! Reconnecting...");
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0, 0);
      tft.setTextColor(TFT_RED);
      tft.println("WiFi Lost...");
      tft.println("Reconnecting...");
      wifi_connect(60);
    }

    // Connect to MQTT
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[MQTT] Connecting...");
      String clientId = "TTGO-" + String(random(0xffff), HEX);

      if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
        Serial.println("Connected!");
        mqttClient.subscribe(MQTT_TOPIC_SUB);
        Serial.print("[MQTT] Subscribed to: ");
        Serial.println(MQTT_TOPIC_SUB);

        // Display success notification
        tft.fillRect(0, 110, 240, 25, TFT_GREEN);
        tft.setTextColor(TFT_BLACK);
        tft.setCursor(5, 115);
        tft.print("MQTT Connected");
        delay(1000);
      } else {
        Serial.print("Failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" retry in 5s");
        delay(5000);
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize

  Serial.println("\n========================================");
  Serial.println("ESP32 TTGO Bioreactor Display + MQTT");
  Serial.println("========================================");
  
  // Initialize UART for receiving data from ESP32 Arduino Nano
  Serial.print("Initializing UART (Serial2)... ");
  Serial.print("RX Pin: GPIO");
  Serial.print(UART_RX_PIN);
  Serial.print(", TX Pin: GPIO");
  Serial.print(UART_TX_PIN);
  Serial.print(", Baud: ");
  Serial.println(UART_BAUD);
  
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Test UART initialization
  if (Serial2) {
    Serial.println("✓ UART Serial2 initialized successfully");
  } else {
    Serial.println("✗ ERROR: UART Serial2 initialization failed!");
  }
  
  Serial.print("UART RX Pin configured: GPIO");
  Serial.println(UART_RX_PIN);
  Serial.println("Waiting for data from ESP32 DevKit...");
  Serial.println("Expected format: TARGET_RPM,MEASURED_RPM,ERROR_RPM,PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM\\n");
  Serial.println("Note: pH data will use placeholder values until pH sensor is implemented");
  Serial.println("========================================\n");
 
  // Initialize TFT Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.println("Initializing...");

  // Connect to WiFi
  tft.println("WiFi: Connecting...");
  wifi_connect();

  // Setup MQTT
  espClient.setInsecure();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);

  tft.println("MQTT: Connecting...");
  mqtt_reconnect();

  tft.println("\nSystem Ready!");
  delay(2000);

  // Create the display sprite
  face.setColorDepth(8);
  face.createSprite(FACE_W, FACE_H);

  // Initial display
  renderBioreactorScreen();

  targetTime = millis() + UPDATE_INTERVAL_MS;
  lastUARTActivity = millis();
  lastDebugPrint = millis();
  lastMqttPublish = millis();

  Serial.println("Display initialized");
  Serial.println("Waiting for NTP time sync...");

  // Send initial setpoints to DevKit on startup
  Serial.println("\n[INFO] Sending initial setpoints to DevKit...");
  sendSetpoints();

  Serial.println("\n========================================");
  Serial.println("System Running:");
  Serial.println("- UART: Sending setpoints ONLY on MQTT updates");
  Serial.println("- UART: Receiving sensor data from DevKit");
  Serial.println("- MQTT: Publishing sensor data (1s interval)");
  Serial.println("- MQTT: Listening for setpoint commands");
  Serial.println("========================================\n");
}

void loop()
{
  // =========================================================================
  // MQTT Connection Management
  // =========================================================================
  if (!mqttClient.connected()) {
    mqtt_reconnect();
  }
  mqttClient.loop();

  // =========================================================================
  // Publish Sensor Data to MQTT Server (Every 5 seconds)
  // Will republish last valid data even if no new UART data has arrived
  // =========================================================================
  if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
    publishSensorDataToMQTT();  // Publishes last valid data from bioreactorData
    lastMqttPublish = millis();
  }

  // =========================================================================
  // Send Setpoints to ESP32 DevKit via UART (ONLY when updated from MQTT)
  // =========================================================================
  if (setpointsUpdated) {
    Serial.println("\n[INFO] New setpoints received from MQTT - transmitting to DevKit...");
    sendSetpoints();
    setpointsUpdated = false;  // Clear the flag after sending
  }

  // =========================================================================
  // UART Data Reception from ESP32 DevKit (Non-blocking)
  // Process a limited number of bytes per loop to prevent blocking
  // =========================================================================
  if (Serial2.available() > 0) {
    lastUARTActivity = millis();

    // Debug: print every 100th time we receive data (disabled to prevent blocking)
    // static unsigned long rxCount = 0;
    // rxCount++;
    // if (rxCount % 100 == 0) {
    //   Serial.print("[DEBUG] UART RX active, bytes available: ");
    //   Serial.println(Serial2.available());
    // }

    // Process max MAX_BYTES_PER_LOOP bytes per iteration to avoid blocking
    int bytesToProcess = min(Serial2.available(), MAX_BYTES_PER_LOOP);

    for (int i = 0; i < bytesToProcess; i++) {
      if (Serial2.available() <= 0) break;  // Safety check

      char c = Serial2.read();
      bytesReceived++;

      if (c == '\n') {
        // End of packet, parse the data
        if (uartBuffer.length() > 0) {
          // Debug: print raw packet to diagnose parsing issues (DISABLED in production)
          // Uncomment the lines below for debugging UART communication issues
          // Serial.print("[DEBUG] Raw packet (len=");
          // Serial.print(uartBuffer.length());
          // Serial.print("): '");
          // Serial.print(uartBuffer);
          // Serial.println("'");

          parseBioreactorData(uartBuffer);
          packetsReceived++;
        }
        uartBuffer = "";  // Clear buffer
      } else if (c == '\r') {
        // Ignore carriage return
        continue;
      } else {
        uartBuffer += c;  // Add character to buffer
        if (uartBuffer.length() > 200) {
          // Buffer overflow protection - increased to 200 to accommodate longer packets
          Serial.print("[UART] ✗ Buffer overflow! Length: ");
          Serial.print(uartBuffer.length());
          Serial.print(" Content: '");
          Serial.print(uartBuffer.substring(0, 50));  // Print first 50 chars
          Serial.println("...'");
          uartBuffer = "";  // Clear
        }
      }
    }
  }
  
  // Check for stale data (if no update in 1 second, mark as invalid)
  if (millis() - bioreactorData.lastUpdate > 1000) {
    if (bioreactorData.dataValid) {
      Serial.println("[UART] WARNING: Data stale (>1s since last update)");
      bioreactorData.dataValid = false;
    }
  }
  
  // Periodic debug output
  if (millis() - lastDebugPrint > DEBUG_INTERVAL_MS) {
    printUARTDebugInfo();
    lastDebugPrint = millis();
  }

  // Clear notification after duration expires
  if (showNotification && (millis() - notificationStartTime >= NOTIFICATION_DURATION_MS)) {
    showNotification = false;
  }

  // Update display periodically
  if (targetTime < millis()) {
    targetTime = millis() + UPDATE_INTERVAL_MS;

    // Update time (NTP sync happens in syncTime())
    time_secs += (UPDATE_INTERVAL_MS / 1000.0);

    // Midnight roll-over
    if (time_secs >= (60 * 60 * 24)) time_secs = 0;

    // Request time from NTP server and synchronise the local clock
    syncTime();

    // Render the display with time and RPM data
    renderBioreactorScreen();
  }
}

// =========================================================================
// Publish sensor data to MQTT server
// Format: JSON {"rpm": 450.5, "temp": 35.2, "ph": 7.1}
// =========================================================================
void publishSensorDataToMQTT() {
  if (!bioreactorData.dataValid) {
    Serial.println("[MQTT] WARNING: No valid sensor data to publish");
    return;
  }

  // Create JSON document with only rpm, temp, and ph
  JsonDocument doc;
  doc["rpm"] = bioreactorData.measuredRPM;
  doc["temp"] = bioreactorData.temperature;
  doc["ph"] = bioreactorData.pH;

  // Serialize JSON to buffer
  char buffer[256];
  serializeJson(doc, buffer);

  // Publish to MQTT
  if (mqttClient.publish(MQTT_TOPIC_PUB, buffer)) {
    mqttPublishCount++;

    // Always print when publishing
    Serial.print("Sending JSON: ");
    Serial.println(buffer);
  } else {
    Serial.println("[MQTT] ERROR: Publish failed!");
  }
}

// =========================================================================
// Send setpoints to ESP32 DevKit via UART
// Format: SET:RPM=500.0,TEMP=35.0,PH=7.0\n
// =========================================================================
void sendSetpoints() {
  Serial2.print("SET:RPM=");
  Serial2.print(SETPOINT_RPM, 1);
  Serial2.print(",TEMP=");
  Serial2.print(SETPOINT_TEMP, 1);
  Serial2.print(",PH=");
  Serial2.print(SETPOINT_PH, 2);
  Serial2.print("\n");

  setpointsSent++;

  // Always output when setpoints are sent (since it's event-driven now)
  Serial.print("[UART TX #");
  Serial.print(setpointsSent);
  Serial.print("] Setpoints sent to DevKit - RPM=");
  Serial.print(SETPOINT_RPM, 1);
  Serial.print(" TEMP=");
  Serial.print(SETPOINT_TEMP, 1);
  Serial.print(" PH=");
  Serial.println(SETPOINT_PH, 2);
}

// =========================================================================
// Parse bioreactor data from UART
// Format (11 fields): TARGET_RPM,MEASURED_RPM,ERROR_RPM,RPM_PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM,PH_TARGET,PH_MEASURED,PUMP_STATE
// =========================================================================
void parseBioreactorData(String data) {
  if (data.length() == 0) {
    Serial.println("[UART] ERROR: Empty data packet received");
    parseErrors++;
    return;
  }

  int comma1 = data.indexOf(',');
  int comma2 = data.indexOf(',', comma1 + 1);
  int comma3 = data.indexOf(',', comma2 + 1);
  int comma4 = data.indexOf(',', comma3 + 1);
  int comma5 = data.indexOf(',', comma4 + 1);
  int comma6 = data.indexOf(',', comma5 + 1);
  int comma7 = data.indexOf(',', comma6 + 1);
  int comma8 = data.indexOf(',', comma7 + 1);
  int comma9 = data.indexOf(',', comma8 + 1);
  int comma10 = data.indexOf(',', comma9 + 1);

  // Check for 11-field format (with pH data)
  if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0 && comma5 > 0 &&
      comma6 > 0 && comma7 > 0 && comma8 > 0 && comma9 > 0 && comma10 > 0) {
    // Parse all 11 fields
    bioreactorData.targetRPM = data.substring(0, comma1).toFloat();
    bioreactorData.measuredRPM = data.substring(comma1 + 1, comma2).toFloat();
    bioreactorData.error = data.substring(comma2 + 1, comma3).toFloat();
    bioreactorData.motorPWM = data.substring(comma3 + 1, comma4).toInt();

    float tempTarget = data.substring(comma4 + 1, comma5).toFloat();
    bioreactorData.temperature = data.substring(comma5 + 1, comma6).toFloat();
    bioreactorData.tempError = data.substring(comma6 + 1, comma7).toFloat();
    bioreactorData.heaterPWM = data.substring(comma7 + 1, comma8).toInt();

    float phTarget = data.substring(comma8 + 1, comma9).toFloat();
    bioreactorData.pH = data.substring(comma9 + 1, comma10).toFloat();
    int pumpState = data.substring(comma10 + 1).toInt();

    // Calculate pH error
    bioreactorData.pHError = phTarget - bioreactorData.pH;

    bioreactorData.dataValid = true;
    bioreactorData.lastUpdate = millis();

    // Reduced debug output to prevent blocking
    if (packetsReceived % 10 == 0) {  // Print every 10th packet
      Serial.print("[UART] ✓ RPM:");
      Serial.print(bioreactorData.measuredRPM, 1);
      Serial.print(" Temp:");
      Serial.print(bioreactorData.temperature, 1);
      Serial.print("°C pH:");
      Serial.print(bioreactorData.pH, 2);
      Serial.print(" Pump:");
      Serial.println(pumpState);
    }
  } else {
    bioreactorData.dataValid = false;
    Serial.print("[UART] ✗ Parse ERROR: Invalid format. Data: '");
    Serial.print(data);
    Serial.println("'");
    Serial.println("[UART] Expected: TARGET_RPM,MEASURED_RPM,ERROR_RPM,RPM_PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM,PH_TARGET,PH_MEASURED,PUMP_STATE");
    parseErrors++;
  }
}

// =========================================================================
// Print UART debug information
// =========================================================================
void printUARTDebugInfo() {
  unsigned long timeSinceLastActivity = millis() - lastUARTActivity;
  unsigned long timeSinceLastUpdate = millis() - bioreactorData.lastUpdate;
  
  Serial.println("\n--- UART Connection Status ---");
  Serial.print("UART RX Pin: GPIO");
  Serial.println(UART_RX_PIN);
  Serial.print("Baud Rate: ");
  Serial.println(UART_BAUD);
  Serial.print("Bytes received (total): ");
  Serial.println(bytesReceived);
  Serial.print("Packets received (total): ");
  Serial.println(packetsReceived);
  Serial.print("Parse errors (total): ");
  Serial.println(parseErrors);
  Serial.print("Time since last UART activity: ");
  Serial.print(timeSinceLastActivity);
  Serial.println(" ms");
  Serial.print("Time since last valid data: ");
  Serial.print(timeSinceLastUpdate);
  Serial.println(" ms");
  Serial.print("Data valid: ");
  Serial.println(bioreactorData.dataValid ? "YES" : "NO");
  Serial.print("Current buffer length: ");
  Serial.println(uartBuffer.length());
  
  if (timeSinceLastActivity > 2000) {
    Serial.println("⚠ WARNING: No UART activity for >2 seconds!");
    Serial.println("   Check connections:");
    Serial.println("   - ESP32_DevKit GPIO1 (TX) -> ESP32_TTGO GPIO25 (RX)");
    Serial.println("   - GND connected between boards");
  }
  
  if (bioreactorData.dataValid) {
    Serial.print("Last received data - Target: ");
    Serial.print(bioreactorData.targetRPM);
    Serial.print(" RPM, Measured: ");
    Serial.print(bioreactorData.measuredRPM);
    Serial.print(" RPM");
    Serial.println();
  } else {
    Serial.println("⚠ No valid data received yet");
  }
  Serial.println("--------------------------------\n");
}

// =========================================================================
// Render bioreactor display with time and sensor data
// Format: DDMMYYYY HH:MM:SS
//         RPM: xxx.x ± x.x
//         Temp: xx.x ± x.x°C
//         pH: x.x ± x.x
// =========================================================================
void renderBioreactorScreen() {
  face.fillSprite(TFT_BLACK);

  // Show notification banner if active
  if (showNotification) {
    face.fillRect(0, 110, 240, 25, TFT_BLUE);
    face.setTextColor(TFT_WHITE, TFT_BLUE);
    face.setTextDatum(MC_DATUM);
    face.drawString("MQTT: Setpoints Updated", 120, 122);
    face.setTextDatum(TL_DATUM);  // Reset to top-left
  }

  int yPos = 5;

  // Row 1: Date and Time - DDMMYYYY HH:MM:SS
  face.setTextColor(TEXT_FG);
  face.setFreeFont(&FreeSansBold9pt7b);
  String dateTimeStr = dateString(utc) + " " + timeString(time_secs);
  face.drawString(dateTimeStr, 5, yPos);

  yPos += 30;

  // Row 2: RPM ± error
  face.setTextColor(LABEL_FG);
  face.drawString("RPM:", 5, yPos);
  if (bioreactorData.dataValid) {
    face.setTextColor(VALUE_FG);
    String rpmStr = String(bioreactorData.measuredRPM, 1) + " +- " + String(abs(bioreactorData.error), 1);
    face.drawString(rpmStr, 60, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("N/A", 60, yPos);
  }

  yPos += 30;

  // Row 3: Temperature ± error
  face.setTextColor(LABEL_FG);
  face.drawString("Temp:", 5, yPos);
  if (bioreactorData.dataValid) {
    face.setTextColor(VALUE_FG);
    String tempStr = String(bioreactorData.temperature, 1) + " +- " + String(abs(bioreactorData.tempError), 1) + " C";
    face.drawString(tempStr, 60, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("N/A", 60, yPos);
  }

  yPos += 30;

  // Row 4: pH ± error
  face.setTextColor(LABEL_FG);
  face.drawString("pH:", 5, yPos);
  if (bioreactorData.dataValid) {
    face.setTextColor(VALUE_FG);
    String pHStr = String(bioreactorData.pH, 1) + " +- " + String(abs(bioreactorData.pHError), 1);
    face.drawString(pHStr, 60, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("N/A", 60, yPos);
  }

  // Push sprite to display
  face.pushSprite(0, 0, TFT_TRANSPARENT);
}
