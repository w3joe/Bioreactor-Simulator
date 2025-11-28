#ifndef STIRRING_H
#define STIRRING_H

// Controller state variables (static to maintain state between calls)
static float integral = 0.0;           // Integral accumulator
static float previousError = 0.0;       // Previous error for derivative calculation
static float targetSpeed = 500.0;      // Target speed in RPM (default 500 RPM)
static float Kp = 0.08;                // Proportional gain (tuned for 8-bit PWM)
static float Ki = 0.005;               // Integral gain (tuned for 8-bit PWM)
static float Kd = 0.015;               // Derivative gain (tuned for 8-bit PWM)
static float feedForwardGain = 0.08;   // Feed-forward gain (PWM per RPM)
static float antiWindupGain = 0.5;     // Anti-windup gain (default 0.5)
static bool controllerInitialized = false; // Initialization flag

// Constants
const float MIN_SETPOINT = 0.0;        // Minimum setpoint (RPM)
const float MAX_SETPOINT = 1000.0;     // Maximum setpoint (RPM)
const int MIN_VOLTAGE = 50;              // Minimum motor voltage (PWM)
const int MAX_VOLTAGE = 255;            // Maximum motor voltage (PWM) - 8-bit
const float SAMPLING_TIME = 0.01;       // Sampling time in seconds (10 ms)

/**
 * Initialize the PID controller
 * Resets integral accumulator and previous error
 * Should be called once in setup()
 */
void initController() {
  integral = 0.0;
  previousError = 0.0;
  controllerInitialized = true;
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
 * Set controller gains
 * @param Kp_new New proportional gain (0.01 - 2.0)
 * @param Ki_new New integral gain (0.0 - 1.0)
 * @param Kd_new New derivative gain (0.0 - 0.5)
 */
void setGains(float Kp_new, float Ki_new, float Kd_new = -1.0) {
  if (Kp_new >= 0.01 && Kp_new <= 2.0) {
    Kp = Kp_new;
  }
  if (Ki_new >= 0.0 && Ki_new <= 1.0) {
    Ki = Ki_new;
  }
  if (Kd_new >= 0.0 && Kd_new <= 0.5) {
    Kd = Kd_new;
  }
}

/**
 * Reset the integral term and previous error
 * Useful when changing setpoint drastically or recovering from errors
 */
void resetIntegral() {
  integral = 0.0;
  previousError = 0.0;
}

/**
 * Get the current integral term value
 * @return Current integral accumulator value
 */
float getIntegralTerm() {
  return integral;
}

/**
 * Get the current proportional term value
 * @param error Current error (setpoint - measured_speed)
 * @return Proportional term value
 */
float getProportionalTerm(float error) {
  return Kp * error;
}

/**
 * Get the current derivative term value
 * @param error Current error (setpoint - measured_speed)
 * @param deltaT Time since last update in seconds
 * @return Derivative term value
 */
float getDerivativeTerm(float error, float deltaT) {
  if (deltaT > 0.0 && deltaT < 0.1) {
    float errorDerivative = (error - previousError) / deltaT;
    return Kd * errorDerivative;
  }
  return 0.0; // Return zero if deltaT is invalid
}

/**
 * Calculate motor voltage using PID controller with anti-windup
 *
 * This is the main controller function. It implements:
 * - Discrete-time PID controller: u(k) = Kp * e(k) + Ki * Ts * sum(e(0)...e(k)) + Kd * (e(k) - e(k-1)) / Ts
 * - Back-calculation anti-windup to prevent integral saturation
 * - Output saturation to [0, 255]
 *
 * @param setpoint Target speed in RPM
 * @param measspeed Current measured speed in RPM
 * @param deltaT Time since last update in seconds (should be ~0.01s for 10ms loop)
 * @return Motor voltage command (0-255) for PWM output
 */
int calculateMotorVoltage(float setpoint, float measspeed, float deltaT) {
  // Initialize controller on first call if not already done
  if (!controllerInitialized) {
    initController();
  }

  // Calculate error
  float error = setpoint - measspeed;

  // Reset integral if error changes sign significantly (crossed setpoint with margin)
  // This prevents integral from winding up in wrong direction after overshoot
  if ((previousError > 10.0 && error < -10.0) || (previousError < -10.0 && error > 10.0)) {
    integral = 0.0;
  }

  // Stop accumulating integral if error is very small (within ±5 RPM deadband)
  bool inDeadband = (error > -5.0 && error < 5.0);
  if (inDeadband) {
    integral *= 0.95;  // Slowly decay integral when near setpoint
  }

  // Calculate proportional term
  float p_term = Kp * error;

  // Update integral term (using actual deltaT, but typically 0.01s)
  // Use actual deltaT for better accuracy if timing varies slightly
  float actualTs = (deltaT > 0.0 && deltaT < 0.1) ? deltaT : SAMPLING_TIME;

  // Only accumulate integral if error is reasonable (within ±200 RPM)
  // This prevents massive windup during startup or large disturbances
  if (!inDeadband && error > -200.0 && error < 200.0) {
    integral += error * actualTs;
  }

  // Clamp integral to prevent excessive windup
  // Limit integral term directly (not the contribution)
  // With Ki=0.005, integral max of 2000 gives contribution of 10 PWM units
  float maxIntegral = 2000.0;
  if (integral > maxIntegral) integral = maxIntegral;
  if (integral < -maxIntegral) integral = -maxIntegral;

  // Calculate derivative term
  // Derivative of error: (error - previousError) / deltaT
  float d_term = 0.0;
  if (actualTs > 0.0) {
    float errorDerivative = (error - previousError) / actualTs;
    d_term = Kd * errorDerivative;
  }

  // Feed-forward term: baseline PWM for target speed
  // This provides a starting point so PID only needs to make small corrections
  float ff_term = setpoint * feedForwardGain;

  // Calculate raw controller output (Feed-forward + PID correction)
  float raw_output = ff_term + p_term + Ki * integral + d_term;

  // Saturate output to [0, 255]
  int saturated_output = (int)raw_output;
  if (saturated_output < MIN_VOLTAGE) saturated_output = MIN_VOLTAGE;
  if (saturated_output > MAX_VOLTAGE) saturated_output = MAX_VOLTAGE;
  
  // Anti-windup: back-calculation method
  // If output is saturated, reduce integral term to prevent windup
  if ((int)raw_output != saturated_output) {
    float saturation_error = raw_output - (float)saturated_output;
    // Back-calculate to prevent windup
    if (Ki > 0.0) {
      integral -= (saturation_error / Ki) * antiWindupGain;
    }
  }
  
  // Update previous error for next iteration
  previousError = error;
  
  return saturated_output;
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

