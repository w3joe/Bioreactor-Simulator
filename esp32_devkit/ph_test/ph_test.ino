/*
 * pH Sensor Voltage Calibration Helper
 * =====================================
 * This sketch reads analog voltage from a pH sensor and displays
 * the values to help you calibrate your pH measurement system.
 *
 * Hardware Setup:
 * - Connect pH sensor output to GPIO5 (or modify PH_SENSOR_PIN below)
 * - Connect pH sensor VCC to 3.3V
 * - Connect pH sensor GND to GND
 *
 * Calibration Procedure:
 * 1. Upload this sketch to your ESP32
 * 2. Open Serial Monitor (115200 baud)
 * 3. Place sensor in pH 4.0 buffer solution
 * 4. Wait for readings to stabilize (30 seconds)
 * 5. Record the voltage value
 * 6. Rinse sensor with distilled water
 * 7. Place sensor in pH 7.0 buffer solution
 * 8. Wait for readings to stabilize
 * 9. Record the voltage value
 * 10. Rinse sensor with distilled water
 * 11. Place sensor in pH 10.0 buffer solution
 * 12. Wait for readings to stabilize
 * 13. Record the voltage value
 *
 * Use these voltage values in ph_control.h calibration functions
 */

#include <Arduino.h>

// Pin definition
#define PH_SENSOR_PIN 5  // GPIO5 for pH sensor

// ADC parameters for ESP32
const int ADC_RESOLUTION = 4095;    // 12-bit ADC (0-4095)
const float ADC_VREF = 3.3;         // ESP32 ADC reference voltage

// Sampling parameters
const int SAMPLE_COUNT = 20;        // Number of samples to average
const int SAMPLE_INTERVAL_MS = 50;  // Time between samples (ms)
const int DISPLAY_INTERVAL_MS = 500; // Update display every 500ms

unsigned long lastDisplay = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  pH Sensor Voltage Calibration Tool");
  Serial.println("========================================");
  Serial.println();
  Serial.print("Sensor Pin: GPIO");
  Serial.println(PH_SENSOR_PIN);
  Serial.print("ADC Resolution: ");
  Serial.print(ADC_RESOLUTION);
  Serial.println(" (12-bit)");
  Serial.print("Reference Voltage: ");
  Serial.print(ADC_VREF);
  Serial.println("V");
  Serial.println();
  Serial.println("Instructions:");
  Serial.println("1. Place sensor in pH 4.0 buffer");
  Serial.println("2. Wait for stable reading (~30 sec)");
  Serial.println("3. Record voltage");
  Serial.println("4. Repeat for pH 7.0 and pH 10.0");
  Serial.println("========================================");
  Serial.println();

  // Initialize pH sensor pin
  pinMode(PH_SENSOR_PIN, INPUT);

  // Give time to place sensor in first buffer
  Serial.println("Starting measurements in 5 seconds...");
  delay(5000);

  Serial.println("\nFormat: [Raw ADC] Voltage (V) | Avg Voltage (V) | Std Dev");
  Serial.println("----------------------------------------------------------");
}

void loop() {
  if (millis() - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = millis();

    // Read multiple samples
    float voltageSum = 0.0;
    float voltageSamples[SAMPLE_COUNT];
    int rawADCSum = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
      int rawADC = analogRead(PH_SENSOR_PIN);
      float voltage = (rawADC / (float)ADC_RESOLUTION) * ADC_VREF;

      voltageSamples[i] = voltage;
      voltageSum += voltage;
      rawADCSum += rawADC;

      delay(SAMPLE_INTERVAL_MS);
    }

    // Calculate statistics
    float avgVoltage = voltageSum / SAMPLE_COUNT;
    float avgRawADC = rawADCSum / (float)SAMPLE_COUNT;

    // Calculate standard deviation
    float variance = 0.0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
      float diff = voltageSamples[i] - avgVoltage;
      variance += diff * diff;
    }
    float stdDev = sqrt(variance / SAMPLE_COUNT);

    // Find min and max
    float minVoltage = voltageSamples[0];
    float maxVoltage = voltageSamples[0];
    for (int i = 1; i < SAMPLE_COUNT; i++) {
      if (voltageSamples[i] < minVoltage) minVoltage = voltageSamples[i];
      if (voltageSamples[i] > maxVoltage) maxVoltage = voltageSamples[i];
    }

    // Display results
    Serial.print("[");
    Serial.print(avgRawADC, 0);
    Serial.print("] ");
    Serial.print(avgVoltage, 4);
    Serial.print("V | Range: ");
    Serial.print(minVoltage, 4);
    Serial.print("V - ");
    Serial.print(maxVoltage, 4);
    Serial.print("V | StdDev: ");
    Serial.print(stdDev, 5);
    Serial.println("V");

    // Provide feedback on measurement quality
    if (stdDev < 0.001) {
      Serial.println("  ✓ EXCELLENT - Very stable reading");
    } else if (stdDev < 0.005) {
      Serial.println("  ✓ GOOD - Acceptable stability");
    } else if (stdDev < 0.01) {
      Serial.println("  ⚠ FAIR - Somewhat noisy, wait longer");
    } else {
      Serial.println("  ✗ POOR - Very noisy, check connections");
    }
    Serial.println();
  }
}

/*
 * Example Calibration Output Format:
 *
 * For pH 4.0 buffer:
 * [2750] 2.0300V | Range: 2.0285V - 2.0315V | StdDev: 0.00087V
 *
 * For pH 7.0 buffer:
 * [2234] 1.6500V | Range: 1.6485V - 1.6515V | StdDev: 0.00092V
 *
 * For pH 10.0 buffer:
 * [1720] 1.2700V | Range: 1.2685V - 1.2715V | StdDev: 0.00089V
 *
 * Then in ph_control.h, use:
 * calibratepH(4.0, 2.0300, 10.0, 1.2700);
 *
 * Or for three-point calibration:
 * calibratepH3Point(2.0300, 1.6500, 1.2700);
 */
