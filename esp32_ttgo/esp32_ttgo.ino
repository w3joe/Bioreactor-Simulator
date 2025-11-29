#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

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

// Time for next screen update
uint32_t targetTime = 0;
#define UPDATE_INTERVAL_MS 100  // Update display every 100ms

// UART Communication for bioreactor data
// Connect ESP32_DevKit GPIO1 (TX) to ESP32_TTGO GPIO25 (RX)
// Connect GND of both boards together
#define UART_RX_PIN 25   // RX pin for Serial2
#define UART_TX_PIN 26   // TX pin for Serial2 (not used for receiving, but needed for Serial2.begin)
#define UART_BAUD 115200

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
#define DEBUG_INTERVAL_MS 5000  // Print debug info every 5 seconds
#define MAX_BYTES_PER_LOOP 32   // Process max 32 bytes per loop iteration to prevent blocking

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to initialize
  
  Serial.println("\n========================================");
  Serial.println("ESP32 TTGO Bioreactor Display");
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
 
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Create the display sprite
  face.setColorDepth(8);
  face.createSprite(FACE_W, FACE_H);

  // Initial display
  renderBioreactorScreen();

  targetTime = millis() + UPDATE_INTERVAL_MS;
  lastUARTActivity = millis();
  lastDebugPrint = millis();
  
  Serial.println("Display initialized");
  Serial.println("Waiting for NTP time sync...");
}

void loop()
{
  // =========================================================================
  // UART Data Reception from ESP32 Arduino Nano (Non-blocking)
  // Process a limited number of bytes per loop to prevent blocking
  // =========================================================================
  if (Serial2.available() > 0) {
    lastUARTActivity = millis();

    // Process max MAX_BYTES_PER_LOOP bytes per iteration to avoid blocking
    int bytesToProcess = min(Serial2.available(), MAX_BYTES_PER_LOOP);

    for (int i = 0; i < bytesToProcess; i++) {
      if (Serial2.available() <= 0) break;  // Safety check

      char c = Serial2.read();
      bytesReceived++;

      if (c == '\n') {
        // End of packet, parse the data
        if (uartBuffer.length() > 0) {
          parseBioreactorData(uartBuffer);
          packetsReceived++;
        }
        uartBuffer = "";  // Clear buffer
      } else if (c == '\r') {
        // Ignore carriage return
        continue;
      } else {
        uartBuffer += c;  // Add character to buffer
        if (uartBuffer.length() > 100) {
          // Buffer overflow protection
          Serial.println("[UART] WARNING: Buffer overflow, clearing buffer");
          uartBuffer = "";
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
// Parse bioreactor data from UART
// Current format (8 fields): TARGET_RPM,MEASURED_RPM,ERROR_RPM,PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM
// Note: pH values use placeholders until pH sensor is implemented
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

  // Check for 8-field format (current DevKit implementation)
  if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0 && comma5 > 0 && comma6 > 0 && comma7 > 0) {
    // Parse 8 fields: TARGET_RPM,MEASURED_RPM,ERROR_RPM,PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM
    bioreactorData.targetRPM = data.substring(0, comma1).toFloat();
    bioreactorData.measuredRPM = data.substring(comma1 + 1, comma2).toFloat();
    bioreactorData.error = data.substring(comma2 + 1, comma3).toFloat();
    bioreactorData.motorPWM = data.substring(comma3 + 1, comma4).toInt();

    float tempTarget = data.substring(comma4 + 1, comma5).toFloat();
    bioreactorData.temperature = data.substring(comma5 + 1, comma6).toFloat();
    bioreactorData.tempError = data.substring(comma6 + 1, comma7).toFloat();
    bioreactorData.heaterPWM = data.substring(comma7 + 1).toInt();

    // Use placeholder pH values until sensor is implemented
    bioreactorData.pH = 7.0;  // Neutral pH placeholder
    bioreactorData.pHError = 0.1;  // Small error placeholder

    bioreactorData.dataValid = true;
    bioreactorData.lastUpdate = millis();

    // Reduced debug output to prevent blocking
    if (packetsReceived % 10 == 0) {  // Print every 10th packet
      Serial.print("[UART] ✓ RPM:");
      Serial.print(bioreactorData.measuredRPM, 1);
      Serial.print(" Temp:");
      Serial.print(bioreactorData.temperature, 1);
      Serial.print("C pH:");
      Serial.print(bioreactorData.pH, 2);
      Serial.println(" (placeholder)");
    }
  } else {
    bioreactorData.dataValid = false;
    Serial.print("[UART] ✗ Parse ERROR: Invalid format. Data: '");
    Serial.print(data);
    Serial.println("'");
    Serial.println("[UART] Expected: TARGET_RPM,MEASURED_RPM,ERROR_RPM,PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM");
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
  face.setTextColor(VALUE_FG);
  String tempStr = String(bioreactorData.temperature, 1) + " +- " + String(bioreactorData.tempError, 1) + " C";
  face.drawString(tempStr, 60, yPos);

  yPos += 30;

  // Row 4: pH ± error
  face.setTextColor(LABEL_FG);
  face.drawString("pH:", 5, yPos);
  face.setTextColor(VALUE_FG);
  String pHStr = String(bioreactorData.pH, 1) + " +- " + String(bioreactorData.pHError, 1);
  face.drawString(pHStr, 60, yPos);

  // Push sprite to display
  face.pushSprite(0, 0, TFT_TRANSPARENT);
}
