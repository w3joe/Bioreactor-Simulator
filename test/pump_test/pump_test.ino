/*
 * Pump Test Sketch for ESP32 Arduino Nano
 * ========================================
 * Simple test program to control two pumps connected to D11 and D12
 *
 * Hardware Setup:
 * - Pump 1 connected to D11 (GPIO11)
 * - Pump 2 connected to D12 (GPIO12)
 * - Pump drivers should be connected with:
 *   - Signal wire to GPIO pin
 *   - VCC to appropriate pump power supply
 *   - GND to common ground with ESP32
 *
 * This sketch allows you to:
 * - Test both pumps individually
 * - Control pump speed via PWM (0-255)
 * - Control via Serial commands
 *
 * Serial Commands:
 * - '1' : Turn ON Pump 1 at 100%
 * - '2' : Turn ON Pump 2 at 100%
 * - 'q' : Turn OFF Pump 1
 * - 'w' : Turn OFF Pump 2
 * - 'a' : Turn OFF both pumps
 * - '+' : Increase PWM by 25
 * - '-' : Decrease PWM by 25
 * - 'h' : Show help menu
 */

#include <Arduino.h>

// Pin definitions for ESP32 Arduino Nano
#define PUMP1_PIN 11  // D11 (GPIO11)
#define PUMP2_PIN 12  // D12 (GPIO12)

// PWM parameters
#define PWM_FREQ 1000      // 1kHz PWM frequency
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)
#define PWM_CHANNEL_1 0    // PWM channel for pump 1
#define PWM_CHANNEL_2 1    // PWM channel for pump 2

// Pump control variables
int pump1PWM = 0;   // Pump 1 PWM value (0-255)
int pump2PWM = 0;   // Pump 2 PWM value (0-255)
int currentPWM = 128; // Default PWM level (50%)

// Status tracking
unsigned long lastStatusPrint = 0;
const long STATUS_INTERVAL = 1000; // Print status every 1 second

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("     ESP32 Pump Test Program");
  Serial.println("========================================");

  // Initialize pump pins
  pinMode(PUMP1_PIN, OUTPUT);
  pinMode(PUMP2_PIN, OUTPUT);

  // Setup PWM channels
  ledcAttach(PUMP1_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PUMP2_PIN, PWM_FREQ, PWM_RESOLUTION);

  // Ensure pumps start OFF
  ledcWrite(PUMP1_PIN, 0);
  ledcWrite(PUMP2_PIN, 0);

  Serial.print("Pump 1 Pin: D11 (GPIO");
  Serial.print(PUMP1_PIN);
  Serial.println(")");
  Serial.print("Pump 2 Pin: D12 (GPIO");
  Serial.print(PUMP2_PIN);
  Serial.println(")");
  Serial.print("PWM Frequency: ");
  Serial.print(PWM_FREQ);
  Serial.println(" Hz");
  Serial.print("Default PWM Level: ");
  Serial.print(currentPWM);
  Serial.print(" / 255 (");
  Serial.print((currentPWM * 100) / 255);
  Serial.println("%)");
  Serial.println();

  printHelp();

  Serial.println("Initialization complete. Both pumps OFF.");
  Serial.println("========================================\n");

  lastStatusPrint = millis();
}

void loop() {
  // Handle serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // Print status periodically
  if (millis() - lastStatusPrint >= STATUS_INTERVAL) {
    printStatus();
    lastStatusPrint = millis();
  }
}

void handleCommand(char cmd) {
  switch (cmd) {
    case '1':
      // Turn ON Pump 1
      pump1PWM = currentPWM;
      ledcWrite(PUMP1_PIN, pump1PWM);
      Serial.print("✓ Pump 1 ON at PWM ");
      Serial.print(pump1PWM);
      Serial.print(" (");
      Serial.print((pump1PWM * 100) / 255);
      Serial.println("%)");
      break;

    case '2':
      // Turn ON Pump 2
      pump2PWM = currentPWM;
      ledcWrite(PUMP2_PIN, pump2PWM);
      Serial.print("✓ Pump 2 ON at PWM ");
      Serial.print(pump2PWM);
      Serial.print(" (");
      Serial.print((pump2PWM * 100) / 255);
      Serial.println("%)");
      break;

    case 'q':
      // Turn OFF Pump 1
      pump1PWM = 0;
      ledcWrite(PUMP1_PIN, pump1PWM);
      Serial.println("✓ Pump 1 OFF");
      break;

    case 'w':
      // Turn OFF Pump 2
      pump2PWM = 0;
      ledcWrite(PUMP2_PIN, pump2PWM);
      Serial.println("✓ Pump 2 OFF");
      break;

    case 'a':
      // Turn OFF both pumps
      pump1PWM = 0;
      pump2PWM = 0;
      ledcWrite(PUMP1_PIN, pump1PWM);
      ledcWrite(PUMP2_PIN, pump2PWM);
      Serial.println("✓ Both pumps OFF");
      break;

    case '+':
      // Increase PWM
      currentPWM += 25;
      if (currentPWM > 255) currentPWM = 255;
      Serial.print("✓ PWM increased to ");
      Serial.print(currentPWM);
      Serial.print(" (");
      Serial.print((currentPWM * 100) / 255);
      Serial.println("%)");
      Serial.println("  Press '1' or '2' to apply to pumps");
      break;

    case '-':
      // Decrease PWM
      currentPWM -= 25;
      if (currentPWM < 0) currentPWM = 0;
      Serial.print("✓ PWM decreased to ");
      Serial.print(currentPWM);
      Serial.print(" (");
      Serial.print((currentPWM * 100) / 255);
      Serial.println("%)");
      Serial.println("  Press '1' or '2' to apply to pumps");
      break;

    case 'h':
      // Show help
      printHelp();
      break;

    case 't':
      // Test sequence
      runTestSequence();
      break;

    case '\r':
    case '\n':
      // Ignore newline characters
      break;

    default:
      Serial.print("✗ Unknown command: ");
      Serial.println(cmd);
      Serial.println("  Press 'h' for help");
      break;
  }
}

void printHelp() {
  Serial.println("\n--- COMMAND MENU ---");
  Serial.println("1  : Turn ON Pump 1");
  Serial.println("2  : Turn ON Pump 2");
  Serial.println("q  : Turn OFF Pump 1");
  Serial.println("w  : Turn OFF Pump 2");
  Serial.println("a  : Turn OFF both pumps");
  Serial.println("+  : Increase PWM level");
  Serial.println("-  : Decrease PWM level");
  Serial.println("t  : Run automatic test sequence");
  Serial.println("h  : Show this help menu");
  Serial.println("--------------------\n");
}

void printStatus() {
  Serial.print("[STATUS] Pump1: ");
  if (pump1PWM > 0) {
    Serial.print("ON (");
    Serial.print(pump1PWM);
    Serial.print("/255, ");
    Serial.print((pump1PWM * 100) / 255);
    Serial.print("%)");
  } else {
    Serial.print("OFF");
  }

  Serial.print(" | Pump2: ");
  if (pump2PWM > 0) {
    Serial.print("ON (");
    Serial.print(pump2PWM);
    Serial.print("/255, ");
    Serial.print((pump2PWM * 100) / 255);
    Serial.print("%)");
  } else {
    Serial.print("OFF");
  }

  Serial.print(" | Default PWM: ");
  Serial.print(currentPWM);
  Serial.print("/255 (");
  Serial.print((currentPWM * 100) / 255);
  Serial.println("%)");
}

void runTestSequence() {
  Serial.println("\n========================================");
  Serial.println("  RUNNING AUTOMATIC TEST SEQUENCE");
  Serial.println("========================================");

  // Test Pump 1 at various speeds
  Serial.println("\n[TEST 1] Pump 1 - Speed Ramp");
  for (int pwm = 0; pwm <= 255; pwm += 51) {
    pump1PWM = pwm;
    ledcWrite(PUMP1_PIN, pump1PWM);
    Serial.print("  Pump 1 PWM: ");
    Serial.print(pwm);
    Serial.print(" (");
    Serial.print((pwm * 100) / 255);
    Serial.println("%)");
    delay(2000);
  }
  pump1PWM = 0;
  ledcWrite(PUMP1_PIN, 0);
  Serial.println("  Pump 1 OFF");
  delay(1000);

  // Test Pump 2 at various speeds
  Serial.println("\n[TEST 2] Pump 2 - Speed Ramp");
  for (int pwm = 0; pwm <= 255; pwm += 51) {
    pump2PWM = pwm;
    ledcWrite(PUMP2_PIN, pump2PWM);
    Serial.print("  Pump 2 PWM: ");
    Serial.print(pwm);
    Serial.print(" (");
    Serial.print((pwm * 100) / 255);
    Serial.println("%)");
    delay(2000);
  }
  pump2PWM = 0;
  ledcWrite(PUMP2_PIN, 0);
  Serial.println("  Pump 2 OFF");
  delay(1000);

  // Test both pumps simultaneously
  Serial.println("\n[TEST 3] Both Pumps - 50% Power");
  pump1PWM = 128;
  pump2PWM = 128;
  ledcWrite(PUMP1_PIN, pump1PWM);
  ledcWrite(PUMP2_PIN, pump2PWM);
  Serial.println("  Both pumps ON at 50%");
  delay(3000);

  // Turn off both pumps
  pump1PWM = 0;
  pump2PWM = 0;
  ledcWrite(PUMP1_PIN, 0);
  ledcWrite(PUMP2_PIN, 0);
  Serial.println("  Both pumps OFF");

  Serial.println("\n========================================");
  Serial.println("  TEST SEQUENCE COMPLETE");
  Serial.println("========================================\n");
}
