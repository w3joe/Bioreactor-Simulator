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
// Connect ESP32_Arduino_Nano GPIO1 (TX) to ESP32_TTGO GPIO3 (RX)
// Connect GND of both boards together
// Note: GPIO16/17 may have restrictions, using GPIO1/3 which are confirmed working
#define UART_RX_PIN 3   // RX pin for Serial2
#define UART_TX_PIN 1   // TX pin for Serial2 (not used for receiving, but needed for Serial2 initialization)
#define UART_BAUD 115200

// Bioreactor data received from ESP32 Arduino Nano
struct BioreactorData {
  float targetRPM = 0.0;
  float measuredRPM = 0.0;
  float error = 0.0;
  int motorPWM = 0;
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
  Serial.println("Waiting for data from ESP32 Arduino Nano...");
  Serial.println("Expected format: TARGET,MEASURED,ERROR,PWM\\n");
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
  // UART Data Reception from ESP32 Arduino Nano
  // =========================================================================
  int availableBytes = Serial2.available();
  if (availableBytes > 0) {
    lastUARTActivity = millis();
    bytesReceived += availableBytes;
    
    Serial.print("[UART] Received ");
    Serial.print(availableBytes);
    Serial.print(" byte(s), Buffer: ");
    
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      if (c == '\n') {
        // End of packet, parse the data
        Serial.print(uartBuffer);
        Serial.println(" [END]");
        parseBioreactorData(uartBuffer);
        packetsReceived++;
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
// Format: TARGET,MEASURED,ERROR,PWM
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
  
  if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
    bioreactorData.targetRPM = data.substring(0, comma1).toFloat();
    bioreactorData.measuredRPM = data.substring(comma1 + 1, comma2).toFloat();
    bioreactorData.error = data.substring(comma2 + 1, comma3).toFloat();
    bioreactorData.motorPWM = data.substring(comma3 + 1).toInt();
    bioreactorData.dataValid = true;
    bioreactorData.lastUpdate = millis();
    
    // Debug output to Serial
    Serial.print("[UART] ✓ Parsed - Target: ");
    Serial.print(bioreactorData.targetRPM);
    Serial.print(" RPM, Measured: ");
    Serial.print(bioreactorData.measuredRPM);
    Serial.print(" RPM, Error: ");
    Serial.print(bioreactorData.error);
    Serial.print(" RPM, PWM: ");
    Serial.println(bioreactorData.motorPWM);
  } else {
    bioreactorData.dataValid = false;
    Serial.print("[UART] ✗ Parse ERROR: Invalid format. Data: '");
    Serial.print(data);
    Serial.print("' (Length: ");
    Serial.print(data.length());
    Serial.println(")");
    Serial.print("[UART] Expected format: TARGET,MEASURED,ERROR,PWM\\n");
    Serial.print("[UART] Found commas at positions: ");
    Serial.print(comma1);
    Serial.print(", ");
    Serial.print(comma2);
    Serial.print(", ");
    Serial.println(comma3);
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
    Serial.println("   - ESP32_Arduino_Nano GPIO1 (TX) -> ESP32_TTGO GPIO3 (RX)");
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
// Render bioreactor display with time and RPM data
// =========================================================================
void renderBioreactorScreen() {
  face.fillSprite(TFT_BLACK);

  // Display current time (from NTP)
  face.setTextColor(LABEL_FG);
  face.setFreeFont(&FreeSansBold9pt7b);
  face.drawString("Time:", 10, 5);
  
  face.setTextColor(TEXT_FG);
  face.setFreeFont(&FreeSansBold12pt7b);
  face.drawString(timeString(time_secs), 10, 25);
  
  face.setTextColor(LABEL_FG);
  face.setFreeFont(&FreeSansBold9pt7b);
  face.drawString(dateString(utc), 10, 45);

  // Display bioreactor RPM data
  int yPos = 70;
  int labelX = 10;
  int valueX = 120;
  
  face.setTextColor(LABEL_FG);
  face.setFreeFont(&FreeSansBold9pt7b);
  
  // Target RPM
  face.drawString("Target RPM:", labelX, yPos);
  if (bioreactorData.dataValid) {
    face.setTextColor(VALUE_FG);
    face.drawString(String(bioreactorData.targetRPM, 1), valueX, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("N/A", valueX, yPos);
  }
  
  yPos += 20;
  
  // Measured RPM
  face.setTextColor(LABEL_FG);
  face.drawString("Measured RPM:", labelX, yPos);
  if (bioreactorData.dataValid) {
    face.setTextColor(VALUE_FG);
    face.drawString(String(bioreactorData.measuredRPM, 1), valueX, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("No Data", valueX, yPos);
  }
  
  yPos += 20;
  
  // Error
  face.setTextColor(LABEL_FG);
  face.drawString("Error:", labelX, yPos);
  if (bioreactorData.dataValid) {
    uint16_t errorColor = (abs(bioreactorData.error) < 5.0) ? VALUE_FG : ERROR_FG;
    face.setTextColor(errorColor);
    face.drawString(String(bioreactorData.error, 1) + " RPM", valueX, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("N/A", valueX, yPos);
  }
  
  yPos += 20;
  
  // Motor PWM
  face.setTextColor(LABEL_FG);
  face.drawString("Motor PWM:", labelX, yPos);
  if (bioreactorData.dataValid) {
    face.setTextColor(VALUE_FG);
    face.drawString(String(bioreactorData.motorPWM), valueX, yPos);
  } else {
    face.setTextColor(ERROR_FG);
    face.drawString("N/A", valueX, yPos);
  }

  // Push sprite to display
  face.pushSprite(0, 0, TFT_TRANSPARENT);
}
