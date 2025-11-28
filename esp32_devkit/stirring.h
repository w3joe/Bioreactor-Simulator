#ifndef STIRRING_H
#define STIRRING_H

// Bang-Bang Controller state variables
static float targetSpeed = 500.0;      // Target speed in RPM (default 500 RPM)
static float hysteresis = 10.0;        // Hysteresis band (±10 RPM around setpoint)
static bool controllerInitialized = false; // Initialization flag
static float integral = 0.0;           // Dummy variable for compatibility with getIntegralTerm()

// Constants
const float MIN_SETPOINT = 0.0;        // Minimum setpoint (RPM)
const float MAX_SETPOINT = 1000.0;     // Maximum setpoint (RPM)
const int MIN_VOLTAGE = 0;             // Minimum motor voltage (PWM)
const int MAX_VOLTAGE = 1023;          // Maximum motor voltage (PWM) - 10-bit
const float SAMPLING_TIME = 0.01;       // Sampling time in seconds (100 ms)

/**
 * Initialize the Bang-Bang controller
 * Should be called once in setup()
 */
void initController() {
  controllerInitialized = true;
  integral = 0.0;
}

/**
 * Set the target speed in RPM
 * @param targetRPM Target speed in RPM (0-1000)
 */
void setTargetSpeed(float targetRPM) {
  // Validate and constrain setpoint
  if (targetRPM < MIN_SETPOINT) {
    targetSpeed = MIN_SETPOINT;
  } else if (targetRPM > MAX_SETPOINT) {
    targetSpeed = MAX_SETPOINT;
  } else {
    targetSpeed = targetRPM;
  }
}

/**
 * Get the current target speed
 * @return Current target speed in RPM
 */
float getTargetSpeed() {
  return targetSpeed;
}

/**
 * Set hysteresis band for bang-bang controller
 * @param hyst New hysteresis value (RPM, typically 5-20)
 */
void setHysteresis(float hyst) {
  if (hyst >= 1.0 && hyst <= 50.0) {
    hysteresis = hyst;
  }
}

/**
 * Reset controller state (compatibility function)
 */
void resetIntegral() {
  integral = 0.0;
}

/**
 * Get the hysteresis value (compatibility function)
 * @return Current hysteresis value
 */
float getIntegralTerm() {
  return hysteresis;
}

/**
 * Calculate motor voltage using Bang-Bang controller with hysteresis
 *
 * Bang-bang control (also called on-off control):
 * - If speed < (target - hysteresis): motor ON at full power
 * - If speed > (target + hysteresis): motor OFF
 * - If speed within hysteresis band: maintain previous state
 *
 * This prevents oscillation by creating a dead zone around the setpoint.
 *
 * @param setpoint Target speed in RPM
 * @param measspeed Current measured speed in RPM
 * @param deltaT Time since last update (unused for bang-bang)
 * @return Motor voltage command (0 or MAX_VOLTAGE) for PWM output
 */
int calculateMotorVoltage(float setpoint, float measspeed, float deltaT) {
  // Initialize controller on first call if not already done
  if (!controllerInitialized) {
    initController();
  }

  // Static variable to store previous output state
  static int previousOutput = 0;

  // Calculate thresholds
  float lowerThreshold = setpoint - hysteresis;
  float upperThreshold = setpoint + hysteresis;

  // Bang-bang logic with hysteresis
  if (measspeed < lowerThreshold) {
    // Speed too low - turn motor ON at full power
    previousOutput = MAX_VOLTAGE;
  }
  else if (measspeed > upperThreshold) {
    // Speed too high - turn motor OFF
    previousOutput = 0;
  }
  // else: within hysteresis band - maintain previous state (no change)

  return previousOutput;
}

/**
 * Check controller health status
 * Detects potential sensor failure or motor stall
 * @param measspeed Current measured speed in RPM
 * @param Vmotor Current motor voltage command
 * @return true if controller appears healthy, false if potential issue detected
 */
bool checkControllerHealth(float measspeed, int Vmotor) {
  // If motor voltage is high but speed is zero, possible sensor failure or stall
  if (measspeed == 0.0 && Vmotor > 100) {
    return false; // Potential issue detected
  }
  return true; // Controller appears healthy
}

#endif // STIRRING_H

