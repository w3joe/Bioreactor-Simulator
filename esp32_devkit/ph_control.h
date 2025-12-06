#ifndef PH_CONTROL_H
#define PH_CONTROL_H

#include <Arduino.h>
#include <math.h>

// =========================================================================
// pH Sensor Measurement and Control Module
// =========================================================================
// This module handles:
// - Voltage-based pH sensor reading with averaging
// - Startup priming sequence for pumps (60s acid + 50s base)
// - Deadband control to prevent oscillation
// - Pulse-based pump control (300ms pulses)
// =========================================================================

// pH sensor sampling configuration
#define SAMPLE_COUNT 20

// pH control parameters
static float targetVoltage = 0.95;      // Target voltage (updated based on target pH)
static float voltageTolerance = 0.003;  // ±0.003V deadband
static float targetPH = 5.0;
static float currentPH = 0.0;
static float currentVoltage = 0.0;

// Pump timing settings
#define PUMP_DURATION 300        // Pump pulse duration (ms)
static unsigned long REST_INTERVAL = 3000;  // Time between pump actions (ms)
static unsigned long LAST_ACTUATION = 0;

// Startup priming sequence
static int startupPhase = 0;    // 0 = Acid pump priming, 1 = Base pump priming, 2 = normal operation
static unsigned long startupEndTime = 0;
static const unsigned long acidPrimeDuration = 76000;  // 60 seconds for acid
static const unsigned long basePrimeDuration = 14000;  // 50 seconds for base

static int pumpState = 0;       // 0 = off, 1 = Base pump, 2 = Acid pump

// Pin assignments (set during initialization)
static int phSensorPin = 0;
static int basePumpPin = 0;
static int acidPumpPin = 0;

// =========================================================================
// Helper Functions
// =========================================================================

/**
 * Read voltage from pH sensor with averaging
 * @return Voltage reading from pH sensor (0-3.3V)
 */
float readPHVoltage() {
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++){
    sum += analogRead(phSensorPin);
  }
  float average = sum / float(SAMPLE_COUNT);
  float voltage = (average / 4095.0) * 3.3;  // ESP32 ADC: 12-bit (0-4095), 3.3V
  return voltage;
}

/**
 * Convert voltage to pH using calibration formula
 * @param voltage Voltage reading from sensor
 * @return pH value
 */
float voltageToPH(float voltage) {
  float pH = (voltage + 1.4) / 0.47;
  return pH;
}

/**
 * Calculate current pH from sensor
 * @return Current pH value
 */
float calculateCurrentPH() {
  float voltage = readPHVoltage();
  float pH = voltageToPH(voltage);
  return pH;
}

/**
 * Activate pump for specified duration (blocking delay)
 * @param pin GPIO pin number for pump
 */
void activatePump(int pin){
  digitalWrite(pin, HIGH);
  delay(PUMP_DURATION);
  digitalWrite(pin, LOW);
}

// =========================================================================
// Initialize pH Control System
// =========================================================================
/**
 * Initialize the pH control system
 * @param phPin GPIO pin for pH sensor (analog input)
 * @param basePin GPIO pin for base pump (digital output)
 * @param acidPin GPIO pin for acid pump (digital output)
 */
void initPHControl(int phPin, int basePin, int acidPin) {
  phSensorPin = phPin;
  basePumpPin = basePin;
  acidPumpPin = acidPin;

  analogSetPinAttenuation(phSensorPin, ADC_11db);

  pinMode(basePumpPin, OUTPUT);
  pinMode(acidPumpPin, OUTPUT);

  digitalWrite(basePumpPin, LOW);
  digitalWrite(acidPumpPin, LOW);

  Serial.println("pH Control System Initializing...");

  // Begin priming acid pump first
  startupPhase = 0;
  startupEndTime = millis() + acidPrimeDuration;

  Serial.println("Priming acid pump for 60 seconds...");
}

// =========================================================================
// Update pH Control Loop
// Call this repeatedly in main loop()
// =========================================================================
/**
 * Update pH control - must be called repeatedly in main loop
 * Handles priming sequence and normal pH control
 */
void updatePHControl() {
  unsigned long now = millis();

  // ---- STARTUP PRIMING SEQUENCE ----
  // Phase 0: Pump acid continuously for 60 seconds
  if (startupPhase == 0) {
    if (now < startupEndTime) {
      // Keep acid pump ON continuously
      digitalWrite(acidPumpPin, HIGH);
      pumpState = 2;
    } else {
      // Acid priming complete, turn off pump
      digitalWrite(acidPumpPin, LOW);
      Serial.println("Acid priming complete (60s)");
      Serial.println("Priming base pump for 50 seconds...");

      // Move to base priming phase
      startupPhase = 1;
      startupEndTime = now + basePrimeDuration;
    }
    return;
  }

  // Phase 1: Pump base continuously for 50 seconds
  if (startupPhase == 1) {
    if (now < startupEndTime) {
      // Keep base pump ON continuously
      digitalWrite(basePumpPin, HIGH);
      pumpState = 1;
    } else {
      // Base priming complete, turn off pump
      digitalWrite(basePumpPin, LOW);
      Serial.println("Base priming complete (50s)");
      Serial.println("Priming complete. Switching to normal pH control.");

      // Move to normal operation
      startupPhase = 2;
      LAST_ACTUATION = now;
    }
    return;
  }

  // ---- Normal operation AFTER priming ----

  // Read current voltage and pH
  currentVoltage = readPHVoltage();
  currentPH = voltageToPH(currentVoltage);

  float difference = targetVoltage - currentVoltage;

  // Check if enough time has passed since last actuation
  if ((now - LAST_ACTUATION) < REST_INTERVAL) {
    pumpState = 0;
    return;
  }

  // Check if within tolerance (deadband)
  if (fabs(difference) < voltageTolerance) {
    pumpState = 0;
    return;
  }

  // Determine which pump to activate
  if (difference < 0){
    // Voltage too high, need to lower it - activate acid pump
    Serial.print("Voltage: ");
    Serial.print(currentVoltage, 4);
    Serial.print(" V, Error: ");
    Serial.print(difference, 4);
    Serial.println(" V");
    Serial.println("Acid pump engaged");

    activatePump(acidPumpPin);
    pumpState = 2;
    LAST_ACTUATION = now;
  } else {
    // Voltage too low, need to raise it - activate base pump
    Serial.print("Voltage: ");
    Serial.print(currentVoltage, 4);
    Serial.print(" V, Error: ");
    Serial.print(difference, 4);
    Serial.println(" V");
    Serial.println("Base pump engaged");

    activatePump(basePumpPin);
    pumpState = 1;
    LAST_ACTUATION = now;
  }
}

// =========================================================================
// Getter and Setter Functions
// =========================================================================

/**
 * Get current pH reading
 * @return Current pH value
 */
float getCurrentpH() {
  return currentPH;
}

/**
 * Get target pH value
 * @return Target pH value
 */
float getTargetpH() {
  return targetPH;
}

/**
 * Set target pH value and calculate corresponding target voltage
 * @param target New target pH value
 */
void setTargetpH(float target) {
  targetPH = target;

  // Convert pH to voltage using inverse of: pH = (voltage + 1.4) / 0.47
  // Solving for voltage: voltage = (pH * 0.47) - 1.4
  targetVoltage = (targetPH * 0.4667) - 1.3667;

  Serial.print("Target pH updated to: ");
  Serial.print(targetPH, 2);
  Serial.print(" pH (");
  Serial.print(targetVoltage, 3);
  Serial.println("V)");
}

/**
 * Get current pump state
 * @return 0=off, 1=base pump, 2=acid pump
 */
int getPumpState() {
  return pumpState;
}

/**
 * Get startup phase
 * @return 0=acid priming, 1=base priming, 2=normal operation
 */
int getStartupPhase() {
  return startupPhase;
}

/**
 * Get current voltage reading
 * @return Current voltage from pH sensor
 */
float getCurrentVoltage() {
  return currentVoltage;
}

#endif // PH_CONTROL_H
