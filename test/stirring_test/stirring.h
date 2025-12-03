#ifndef STIRRING_H
#define STIRRING_H

// Optimized Bang-Bang Controller with PWM Modulation
// This hybrid controller provides:
// - Fast bang-bang response when far from setpoint
// - Smooth proportional control near setpoint
// - Reduced oscillation and improved stability

// Controller state variables
static float targetSpeed = 500.0;      // Target speed in RPM (default 500 RPM)
static float hysteresis = 3.0;         // Hysteresis band (±3 RPM) - REDUCED from ±10
static bool controllerInitialized = false; // Initialization flag
static float integral = 0.0;           // Dummy variable for compatibility with getIntegralTerm()

// PWM Modulation zone parameters
static float proportionalZone = 15.0;  // Proportional control zone (±15 RPM from setpoint)
static float proportionalGain = 15.0;  // Gain for proportional zone (PWM per RPM error)

// Constants
const float MIN_SETPOINT = 0.0;        // Minimum setpoint (RPM)
const float MAX_SETPOINT = 1000.0;     // Maximum setpoint (RPM)
const int MIN_VOLTAGE = 0;             // Minimum motor voltage (PWM)
const int MAX_VOLTAGE = 1023;          // Maximum motor voltage (PWM) - 10-bit
const int BASE_VOLTAGE = 512;          // Base voltage for proportional control (50% duty)
const float SAMPLING_TIME = 0.01;      // Sampling time in seconds (10 ms)

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
 * @param hyst New hysteresis value (RPM, typically 2-5 for reduced oscillation)
 */
void setHysteresis(float hyst) {
  if (hyst >= 1.0 && hyst <= 50.0) {
    hysteresis = hyst;
  }
}

/**
 * Set proportional zone size
 * @param zone Proportional control zone in RPM (typically 10-30 RPM)
 */
void setProportionalZone(float zone) {
  if (zone >= 5.0 && zone <= 100.0) {
    proportionalZone = zone;
  }
}

/**
 * Set proportional gain
 * @param gain PWM change per RPM of error (typically 10-30)
 */
void setProportionalGain(float gain) {
  if (gain >= 1.0 && gain <= 100.0) {
    proportionalGain = gain;
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
 * Calculate motor voltage using Hybrid Bang-Bang + PWM Modulation controller
 *
 * Three control zones:
 * 1. FAR BELOW setpoint (error > proportionalZone):
 *    Full power bang-bang (MAX_VOLTAGE)
 *
 * 2. NEAR setpoint (error within ±proportionalZone):
 *    Proportional PWM control with hysteresis
 *    PWM = BASE_VOLTAGE + (error * proportionalGain)
 *
 * 3. FAR ABOVE setpoint (error < -proportionalZone):
 *    Motor OFF (0)
 *
 * This hybrid approach provides:
 * - Fast response when far from setpoint
 * - Smooth, precise control near setpoint
 * - Minimal oscillation around target
 *
 * @param setpoint Target speed in RPM
 * @param measspeed Current measured speed in RPM
 * @param deltaT Time since last update (unused for this controller)
 * @return Motor voltage command (0-1023) for PWM output
 */
int calculateMotorVoltage(float setpoint, float measspeed, float deltaT) {
  // Initialize controller on first call if not already done
  if (!controllerInitialized) {
    initController();
  }

  // Static variable to store previous output state
  static int previousOutput = 0;

  // Calculate error (positive = too slow, negative = too fast)
  float error = setpoint - measspeed;

  // Calculate zone boundaries
  float proportionalLower = -proportionalZone;  // Below this: motor OFF
  float proportionalUpper = proportionalZone;   // Above this: motor FULL
  float hysteresisLower = -hysteresis;
  float hysteresisUpper = hysteresis;

  // ========== ZONE 1: FAR BELOW SETPOINT ==========
  // Speed is much too low - use full power bang-bang
  if (error > proportionalUpper) {
    previousOutput = MAX_VOLTAGE;
  }

  // ========== ZONE 2: PROPORTIONAL CONTROL ZONE ==========
  // Near setpoint - use smooth proportional control with hysteresis
  else if (error >= proportionalLower && error <= proportionalUpper) {
    // Only update PWM if outside hysteresis band (prevents micro-oscillations)
    if (error > hysteresisUpper || error < hysteresisLower) {
      // Proportional control: PWM = base + (error * gain)
      int pwmOutput = BASE_VOLTAGE + (int)(error * proportionalGain);

      // Constrain to valid PWM range
      if (pwmOutput < MIN_VOLTAGE) pwmOutput = MIN_VOLTAGE;
      if (pwmOutput > MAX_VOLTAGE) pwmOutput = MAX_VOLTAGE;

      previousOutput = pwmOutput;
    }
    // else: within hysteresis band - maintain previous output
  }

  // ========== ZONE 3: FAR ABOVE SETPOINT ==========
  // Speed is much too high - turn motor OFF
  else if (error < proportionalLower) {
    previousOutput = MIN_VOLTAGE;
  }

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

