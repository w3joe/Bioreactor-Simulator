#ifndef PH_CONTROL_H
#define PH_CONTROL_H

// =========================================================================
// pH Sensor Measurement Module
// =========================================================================
// This module handles analog pH sensor reading and calibration
// Typical pH sensors output 0-3.3V (ESP32 ADC range)
// pH range: 0-14 (acidic to basic)
// Neutral: pH 7.0
// =========================================================================

// pH sensor calibration parameters
// These should be calibrated with known pH buffer solutions
// Default values assume linear sensor with midpoint at 1.65V (pH 7.0)
struct pHCalibration {
  float voltage_pH4 = 0.507;    // Voltage reading at pH 4.0 buffer
  float voltage_pH7 = 1.69;    // Voltage reading at pH 7.0 buffer (neutral)
  float voltage_pH10 = 3.30;   // Voltage reading at pH 10.0 buffer
  float slope = -0.1727;       // mV per pH unit (calculated from calibration)
  float intercept = 8.21;      // pH at 0V (calculated from calibration)
};

pHCalibration phCal;

// pH measurement variables
const int PH_SAMPLE_COUNT = 10;     // Number of samples to average
const int PH_SAMPLE_INTERVAL = 10;  // ms between samples
float pHValue = 7.0;                // Current pH reading
float pHVoltage = 1.65;             // Current voltage reading
float pHError = 0.0;                // Estimated measurement error (std dev)

// Non-blocking sampling state
static float voltageSamples[10];    // Array to store samples
static int currentSampleIndex = 0;  // Current sample being taken
static unsigned long lastSampleTime = 0;  // Last time a sample was taken
static bool samplingComplete = false;     // Flag indicating sampling is done

// ADC parameters for ESP32
const int ADC_RESOLUTION = 4095;    // 12-bit ADC (0-4095)
const float ADC_VREF = 3.3;         // ESP32 ADC reference voltage

// =========================================================================
// Initialize pH sensor
// =========================================================================
void initpHSensor(int pin) {
  pinMode(pin, INPUT);

  // ESP32 ADC attenuation setting for 0-3.3V range
  // This is handled automatically by analogRead() on ESP32

  Serial.println("pH sensor initialized");
  Serial.print("Using pin: GPIO");
  Serial.println(pin);
  Serial.println("Default calibration loaded (pH 4.0, 7.0, 10.0)");
}

// =========================================================================
// Start non-blocking pH voltage sampling
// Call this to start a new sampling sequence
// =========================================================================
void startpHSampling() {
  currentSampleIndex = 0;
  samplingComplete = false;
  lastSampleTime = 0;
}

// =========================================================================
// Update pH voltage sampling (non-blocking)
// Call this in loop() - it takes one sample per call when interval elapsed
// Returns true when sampling is complete
// =========================================================================
bool updatepHSampling(int pin, unsigned long currentTime) {
  if (samplingComplete) {
    return true;  // Already done
  }

  // Check if it's time to take next sample
  if (currentTime - lastSampleTime >= PH_SAMPLE_INTERVAL || currentSampleIndex == 0) {
    // Take a sample
    int rawADC = analogRead(pin);
    float voltage = (rawADC / (float)ADC_RESOLUTION) * ADC_VREF;
    voltageSamples[currentSampleIndex] = voltage;

    lastSampleTime = currentTime;
    currentSampleIndex++;

    // Check if we've collected all samples
    if (currentSampleIndex >= PH_SAMPLE_COUNT) {
      // Calculate mean
      float voltageSum = 0.0;
      for (int i = 0; i < PH_SAMPLE_COUNT; i++) {
        voltageSum += voltageSamples[i];
      }
      float meanVoltage = voltageSum / PH_SAMPLE_COUNT;

      // Calculate standard deviation for error estimation
      float variance = 0.0;
      for (int i = 0; i < PH_SAMPLE_COUNT; i++) {
        float diff = voltageSamples[i] - meanVoltage;
        variance += diff * diff;
      }
      float stdDev = sqrt(variance / PH_SAMPLE_COUNT);

      // Convert voltage std dev to pH std dev (approximate)
      float slopeAbs = phCal.slope;
      if (slopeAbs < 0) slopeAbs = -slopeAbs;
      pHError = stdDev / slopeAbs;

      // Update global pH voltage
      pHVoltage = meanVoltage;

      samplingComplete = true;
      return true;
    }
  }

  return false;  // Still sampling
}

// =========================================================================
// Get current pH voltage (from last completed sampling)
// =========================================================================
float getpHVoltageReading() {
  return pHVoltage;
}

// =========================================================================
// Convert voltage to pH using calibration
// Linear regression: pH = slope * voltage + intercept
// =========================================================================
float voltageToPH(float voltage) {
  // Using two-point calibration (pH 4.0 and pH 7.0)
  // slope = (pH2 - pH1) / (V2 - V1)
  // intercept = pH1 - slope * V1

  float pH = phCal.slope * voltage + phCal.intercept;

  // Clamp to valid pH range
  if (pH < 0.0) pH = 0.0;
  if (pH > 14.0) pH = 14.0;

  return pH;
}

// =========================================================================
// Calculate pH from current voltage reading
// Call this after sampling is complete to update pH value
// =========================================================================
void calculatepH() {
  pHValue = voltageToPH(pHVoltage);
}

// =========================================================================
// Get current pH value
// =========================================================================
float getCurrentpH() {
  return pHValue;
}

// =========================================================================
// Get pH measurement error (standard deviation)
// =========================================================================
float getpHError() {
  return pHError;
}

// =========================================================================
// Get pH sensor voltage
// =========================================================================
float getpHVoltage() {
  return pHVoltage;
}

// =========================================================================
// Calibrate pH sensor with known buffer solutions
// Call this function when sensor is in calibration buffers
// =========================================================================
void calibratepH(float pH_low, float voltage_low, float pH_high, float voltage_high) {
  // Calculate slope and intercept from two-point calibration
  phCal.slope = (pH_high - pH_low) / (voltage_high - voltage_low);
  phCal.intercept = pH_low - phCal.slope * voltage_low;

  Serial.println("pH sensor calibrated:");
  Serial.print("  pH ");
  Serial.print(pH_low, 1);
  Serial.print(" @ ");
  Serial.print(voltage_low, 3);
  Serial.println("V");
  Serial.print("  pH ");
  Serial.print(pH_high, 1);
  Serial.print(" @ ");
  Serial.print(voltage_high, 3);
  Serial.println("V");
  Serial.print("  Slope: ");
  Serial.print(phCal.slope, 4);
  Serial.println(" pH/V");
  Serial.print("  Intercept: ");
  Serial.println(phCal.intercept, 4);
}

// =========================================================================
// Three-point calibration (more accurate)
// Use pH 4.0, 7.0, and 10.0 buffer solutions
// =========================================================================
void calibratepH3Point(float v4, float v7, float v10) {
  phCal.voltage_pH4 = v4;
  phCal.voltage_pH7 = v7;
  phCal.voltage_pH10 = v10;

  // Use pH 4.0 and pH 10.0 for calibration (wider range)
  calibratepH(4.0, v4, 10.0, v10);
}

// =========================================================================
// Set default calibration values
// =========================================================================
void setDefaultpHCalibration() {
  // Standard values for common pH sensors
  // These assume pH 7.0 at 1.65V (mid-range of 0-3.3V)
  calibratepH(4.0, 2.03, 10.0, 1.27);
}

// =========================================================================
// pH PID Controller
// =========================================================================
// Controls pH by activating base pump (increase pH) or acid pump (decrease pH)
// Target pH: 7.0 (neutral)
// =========================================================================

struct pHController {
  float targetpH = 7.0;           // Target pH setpoint
  float Kp = 100.0;               // Proportional gain (pump duty cycle per pH unit)
  float Ki = 5.0;                 // Integral gain
  float Kd = 0.0;                 // Derivative gain
  float integral = 0.0;           // Integral accumulator
  float lastError = 0.0;          // Previous error for derivative
  float deadband = 0.05;          // Deadband around target (±0.05 pH units)
  int basePumpPWM = 0;            // Base pump PWM output (0-255)
  int acidPumpPWM = 0;            // Acid pump PWM output (0-255)
  float controlOutput = 0.0;      // Raw controller output
  unsigned long lastControlTime = 0;

  // Pump timing control
  unsigned long pumpStartTime = 0;       // When pump started
  unsigned long pumpStopTime = 0;        // When pump last stopped
  bool pumpActive = false;               // Is pump currently running
  bool pumpRampingUp = false;            // Is pump in ramp-up phase
  unsigned long rampStartTime = 0;       // When ramp started
  const unsigned long MIN_PUMP_ON_TIME = 1000;    // Minimum 1 second pump activation
  const unsigned long MIN_PUMP_OFF_TIME = 5000;   // Minimum 5 seconds between activations (allow pH to stabilize)
  const unsigned long RAMP_DURATION = 500;        // 500ms ramp-up time
};

pHController phPID;

// =========================================================================
// Initialize pH PID controller
// =========================================================================
void initpHController(float targetpH = 7.0) {
  phPID.targetpH = targetpH;
  phPID.integral = 0.0;
  phPID.lastError = 0.0;
  phPID.basePumpPWM = 0;
  phPID.acidPumpPWM = 0;
  phPID.lastControlTime = millis();

  Serial.println("pH PID Controller initialized");
  Serial.print("Target pH: ");
  Serial.println(phPID.targetpH, 2);
  Serial.print("Kp: ");
  Serial.print(phPID.Kp);
  Serial.print(", Ki: ");
  Serial.print(phPID.Ki);
  Serial.print(", Kd: ");
  Serial.println(phPID.Kd);
  Serial.print("Deadband: ±");
  Serial.println(phPID.deadband, 3);
}

// =========================================================================
// Set target pH
// =========================================================================
void setTargetpH(float target) {
  phPID.targetpH = target;
  Serial.print("Target pH updated to: ");
  Serial.println(phPID.targetpH, 2);
}

// =========================================================================
// Get target pH
// =========================================================================
float getTargetpH() {
  return phPID.targetpH;
}

// =========================================================================
// Calculate pH control output with pump timing and ramp-up
// Returns: positive = add base, negative = add acid
// Pumps ramp from 0->255 over 500ms, then run at 255 for minimum duration
// =========================================================================
void calculatepHControl(float currentpH, float deltaTime) {
  unsigned long currentTime = millis();

  // Calculate error (target - current)
  float error = phPID.targetpH - currentpH;

  // Apply deadband to avoid unnecessary pump activation
  bool inDeadband = (abs(error) < phPID.deadband);

  if (inDeadband) {
    error = 0.0;
    // Reset integral when in deadband to prevent windup
    phPID.integral = 0.0;
  }

  // Proportional term
  float P = phPID.Kp * error;

  // Integral term (only accumulate outside deadband)
  if (!inDeadband) {
    phPID.integral += error * deltaTime;
  }

  // Clamp integral to prevent windup
  float integralMax = 255.0 / phPID.Ki;  // Limit based on max PWM
  if (phPID.integral > integralMax) phPID.integral = integralMax;
  if (phPID.integral < -integralMax) phPID.integral = -integralMax;

  float I = phPID.Ki * phPID.integral;

  // Derivative term
  float derivative = (error - phPID.lastError) / deltaTime;
  float D = phPID.Kd * derivative;

  // Calculate total control output
  phPID.controlOutput = P + I + D;

  // Determine if pump should be active based on error
  bool shouldActivateBase = (phPID.controlOutput > 0);  // pH too low
  bool shouldActivateAcid = (phPID.controlOutput < 0);  // pH too high

  // Check if enough time has passed since last pump activation
  bool canActivatePump = (currentTime - phPID.pumpStopTime >= phPID.MIN_PUMP_OFF_TIME);

  // Pump state machine
  if (phPID.pumpActive) {
    // Pump is currently running
    unsigned long pumpRunTime = currentTime - phPID.pumpStartTime;

    // Check if minimum on-time has elapsed
    if (pumpRunTime >= phPID.MIN_PUMP_ON_TIME) {
      // Minimum time met - check if we should turn off
      if (inDeadband || error * phPID.lastError < 0) {
        // Turn off pump: either in deadband OR error changed sign (crossed setpoint)
        phPID.pumpActive = false;
        phPID.pumpRampingUp = false;
        phPID.pumpStopTime = currentTime;
        phPID.basePumpPWM = 0;
        phPID.acidPumpPWM = 0;
      } else {
        // Keep pump at full speed (255)
        if (shouldActivateBase) {
          phPID.basePumpPWM = 255;
          phPID.acidPumpPWM = 0;
        } else {
          phPID.acidPumpPWM = 255;
          phPID.basePumpPWM = 0;
        }
      }
    } else {
      // Still in minimum on-time period
      // Handle ramp-up during first 500ms
      if (phPID.pumpRampingUp) {
        unsigned long rampTime = currentTime - phPID.rampStartTime;
        if (rampTime < phPID.RAMP_DURATION) {
          // Ramp PWM from 0 to 255 over RAMP_DURATION
          int rampPWM = (rampTime * 255) / phPID.RAMP_DURATION;
          if (shouldActivateBase) {
            phPID.basePumpPWM = rampPWM;
            phPID.acidPumpPWM = 0;
          } else {
            phPID.acidPumpPWM = rampPWM;
            phPID.basePumpPWM = 0;
          }
        } else {
          // Ramp complete - switch to full speed
          phPID.pumpRampingUp = false;
          if (shouldActivateBase) {
            phPID.basePumpPWM = 255;
            phPID.acidPumpPWM = 0;
          } else {
            phPID.acidPumpPWM = 255;
            phPID.basePumpPWM = 0;
          }
        }
      } else {
        // Already at full speed
        if (shouldActivateBase) {
          phPID.basePumpPWM = 255;
          phPID.acidPumpPWM = 0;
        } else {
          phPID.acidPumpPWM = 255;
          phPID.basePumpPWM = 0;
        }
      }
    }
  } else {
    // Pump is not running - check if we should start it
    if (!inDeadband && canActivatePump && (shouldActivateBase || shouldActivateAcid)) {
      // Start pump with ramp-up
      phPID.pumpActive = true;
      phPID.pumpRampingUp = true;
      phPID.pumpStartTime = currentTime;
      phPID.rampStartTime = currentTime;
      // PWM will be set by ramp logic in next iteration
      phPID.basePumpPWM = 0;
      phPID.acidPumpPWM = 0;
    } else {
      // Keep pumps off
      phPID.basePumpPWM = 0;
      phPID.acidPumpPWM = 0;
    }
  }

  // Store error for next derivative calculation
  phPID.lastError = error;
}

// =========================================================================
// Get base pump PWM
// =========================================================================
int getBasePumpPWM() {
  return phPID.basePumpPWM;
}

// =========================================================================
// Get acid pump PWM
// =========================================================================
int getAcidPumpPWM() {
  return phPID.acidPumpPWM;
}

// =========================================================================
// Get pH error
// =========================================================================
float getpHControlError() {
  return phPID.targetpH - pHValue;
}

// =========================================================================
// Set PID gains
// =========================================================================
void setpHPIDGains(float Kp, float Ki, float Kd) {
  phPID.Kp = Kp;
  phPID.Ki = Ki;
  phPID.Kd = Kd;

  Serial.println("pH PID gains updated:");
  Serial.print("Kp: ");
  Serial.print(phPID.Kp);
  Serial.print(", Ki: ");
  Serial.print(phPID.Ki);
  Serial.print(", Kd: ");
  Serial.println(phPID.Kd);
}

#endif // PH_CONTROL_H

