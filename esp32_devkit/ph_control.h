#ifndef PH_CONTROL_H
#define PH_CONTROL_H

// =========================================================================
// pH Sensor Measurement and Control Module
// =========================================================================
// This module handles:
// - Analog pH sensor reading with moving average filtering
// - Startup priming sequence for pumps
// - Deadband control to prevent oscillation
// - PWM ramping for smooth motor control
// =========================================================================

// pH sensor sampling configuration
#define PH_SENSOR_PIN 39
#define SAMPLING_INTERVAL 20
#define PRINT_INTERVAL 800
#define ARRAY_LENGTH 40

// Pump pin assignments
const int MOTOR_A_PIN = 25;  // Pump A (raises pH - base)
const int MOTOR_B_PIN = 26;  // Pump B (lowers pH - acid)

// pH control parameters
float targetPH = 5.0;
float deadband = 0.2;        // ±0.2 pH deadband

// Pump timing settings
unsigned long pumpRunTime = 3000;      // pump dosing time (ms)
unsigned long pumpCooldown = 5000;     // minimum time between doses (ms)

// Timing variables
unsigned long lastPumpAction = 0;
unsigned long pumpStopTime = 0;

// Startup priming sequence
int startupPhase = 0;    // 0 = Pump A priming, 1 = Pump B priming, 2 = normal operation
unsigned long startupEndTime = 0;
const unsigned long primeDuration = 42000;  // 42 seconds

int pumpState = 0;       // 0 = off, 1 = Pump A, 2 = Pump B

// PWM configuration
const int maxDuty = 255;
int currentDutyA = 0;
int currentDutyB = 0;
int rampStep = 1;

// pH sensor data - moving average filter
int pHArray[ARRAY_LENGTH];
int pHArrayIndex = 0;
double lastAvgADC = 0;
float currentPH = 0.0;

// Timing for sampling and printing
unsigned long lastSamplingTime = 0;
unsigned long lastPrintTime = 0;

// =========================================================================
// Helper Functions
// =========================================================================

// Simple average helper
double averageArray(int* arr, int len) {
  long sum = 0;
  for (int i = 0; i < len; ++i) sum += arr[i];
  return (double)sum / len;
}

// PWM smooth ramp helper
void rampMotor(int pin, int &currentDuty, int targetDuty) {
  if (currentDuty < targetDuty) currentDuty += rampStep;
  else if (currentDuty > targetDuty) currentDuty -= rampStep;

  if (currentDuty < 0) currentDuty = 0;
  if (currentDuty > maxDuty) currentDuty = maxDuty;

  analogWrite(pin, currentDuty);
}

// =========================================================================
// Initialize pH Control System
// =========================================================================
void initPHControl() {
  analogSetPinAttenuation(PH_SENSOR_PIN, ADC_11db);

  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);

  analogWrite(MOTOR_A_PIN, 0);
  analogWrite(MOTOR_B_PIN, 0);

  Serial.println("pH Control System Initializing...");

  // Begin priming Pump A first
  startupPhase = 0;
  startupEndTime = millis() + primeDuration;
  lastSamplingTime = millis();
  lastPrintTime = millis();

  Serial.println("Priming Pump A for 42 seconds...");
}

// =========================================================================
// Update pH Control Loop
// Call this repeatedly in main loop()
// =========================================================================
void updatePHControl() {
  unsigned long now = millis();

  // ---- STARTUP PRIMING SEQUENCE ----
  if (startupPhase < 2) {
    if (startupPhase == 0) {
      rampMotor(MOTOR_A_PIN, currentDutyA, maxDuty);
      rampMotor(MOTOR_B_PIN, currentDutyB, 0);

      if (now >= startupEndTime) {
        startupPhase = 1;
        startupEndTime = now + primeDuration;
        Serial.println("Priming Pump B for 42 seconds...");
      }
    }
    else if (startupPhase == 1) {
      rampMotor(MOTOR_A_PIN, currentDutyA, 0);
      rampMotor(MOTOR_B_PIN, currentDutyB, maxDuty);

      if (now >= startupEndTime) {
        startupPhase = 2;
        Serial.println("Priming complete. Switching to normal pH control.");
      }
    }
    return;
  }

  // ---- Normal operation AFTER priming ----

  // ---- pH Sampling ----
  if (now - lastSamplingTime > SAMPLING_INTERVAL) {
    pHArray[pHArrayIndex++] = analogRead(PH_SENSOR_PIN);
    if (pHArrayIndex >= ARRAY_LENGTH) pHArrayIndex = 0;

    lastAvgADC = averageArray(pHArray, ARRAY_LENGTH);

    // Calibration line: pH = 0.0064 * ADC + 3.0644
    currentPH = 0.0064 * lastAvgADC + 3.0644;

    lastSamplingTime = now;
  }

  // ---- Pump stop logic ----
  if (pumpState != 0 && now >= pumpStopTime) {
    pumpState = 0;
    Serial.println("Pump stopping...");
  }

  // ---- Pump start logic with deadband ----
  if (pumpState == 0 && (now - lastPumpAction) > pumpCooldown) {
    if (currentPH < (targetPH - deadband)) {
      pumpState = 1;
      lastPumpAction = now;
      pumpStopTime = now + pumpRunTime;
      Serial.println("pH LOW → Pump A dosing");
    }
    else if (currentPH > (targetPH + deadband)) {
      pumpState = 2;
      lastPumpAction = now;
      pumpStopTime = now + pumpRunTime;
      Serial.println("pH HIGH → Pump B dosing");
    }
  }

  // ---- Apply PWM ramp control ----
  if (pumpState == 1) {
    rampMotor(MOTOR_A_PIN, currentDutyA, maxDuty);
    rampMotor(MOTOR_B_PIN, currentDutyB, 0);
  }
  else if (pumpState == 2) {
    rampMotor(MOTOR_A_PIN, currentDutyA, 0);
    rampMotor(MOTOR_B_PIN, currentDutyB, maxDuty);
  }
  else {
    rampMotor(MOTOR_A_PIN, currentDutyA, 0);
    rampMotor(MOTOR_B_PIN, currentDutyB, 0);
  }
//CHANGE1
  // // ---- Print readings ----
  // if (now - lastPrintTime > PRINT_INTERVAL) {
  //   Serial.print("Current pH: ");
  //   Serial.println(currentPH, 2);

  //   Serial.print("avgADC: ");
  //   Serial.println(lastAvgADC);

  //   Serial.print("Pump A duty = ");
  //   Serial.print(currentDutyA);
  //   Serial.print(" , Pump B duty = ");
  //   Serial.println(currentDutyB);

  //   lastPrintTime = now;
  // }
}

// =========================================================================
// Getter functions
// =========================================================================
float getCurrentpH() {
  return currentPH;
}

float getTargetpH() {
  return targetPH;
}

void setTargetpH(float target) {
  targetPH = target;
  Serial.print("Target pH updated to: ");
  Serial.println(targetPH, 2);
}

int getPumpState() {
  return pumpState;
}

int getStartupPhase() {
  return startupPhase;
}

#endif // PH_CONTROL_H

