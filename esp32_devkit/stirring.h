#ifndef STIRRING_H
#define STIRRING_H

// Controller state variables (static to maintain state between calls)
static float integral = 0.0;           // Integral accumulator
static float previousError = 0.0;       // Previous error for derivative calculation
static float targetSpeed = 500.0;      // Target speed in RPM (default 500 RPM)
static float Kp = 2.0;                 // Proportional gain (default 2.0)
static float Ki = 0.5;                 // Integral gain (default 0.5)
static float Kd = 0.1;                 // Derivative gain (default 0.1)
static float antiWindupGain = 0.5;     // Anti-windup gain (default 0.5)
static bool controllerInitialized = false; // Initialization flag

// Constants
const float MIN_SETPOINT = 0.0;        // Minimum setpoint (RPM)
const float MAX_SETPOINT = 1000.0;     // Maximum setpoint (RPM)
const int MIN_VOLTAGE = 0;              // Minimum motor voltage (PWM)
const int MAX_VOLTAGE = 1023;           // Maximum motor voltage (PWM)
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
 * @param Kp_new New proportional gain (0.1 - 10.0)
 * @param Ki_new New integral gain (0.0 - 5.0)
 * @param Kd_new New derivative gain (0.0 - 2.0)
 */
void setGains(float Kp_new, float Ki_new, float Kd_new = -1.0) {
  if (Kp_new >= 0.1 && Kp_new <= 10.0) {
    Kp = Kp_new;
  }
  if (Ki_new >= 0.0 && Ki_new <= 5.0) {
    Ki = Ki_new;
  }
  if (Kd_new >= 0.0 && Kd_new <= 2.0) {
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
 * - Output saturation to [0, 1023]
 * 
 * @param setpoint Target speed in RPM
 * @param measspeed Current measured speed in RPM
 * @param deltaT Time since last update in seconds (should be ~0.01s for 10ms loop)
 * @return Motor voltage command (0-1023) for PWM output
 */
int calculateMotorVoltage(float setpoint, float measspeed, float deltaT) {
  // Initialize controller on first call if not already done
  if (!controllerInitialized) {
    initController();
  }
  
  // Calculate error
  float error = setpoint - measspeed;
  
  // Calculate proportional term
  float p_term = Kp * error;
  
  // Update integral term (using actual deltaT, but typically 0.01s)
  // Use actual deltaT for better accuracy if timing varies slightly
  float actualTs = (deltaT > 0.0 && deltaT < 0.1) ? deltaT : SAMPLING_TIME;
  integral += error * actualTs;
  
  // Calculate derivative term
  // Derivative of error: (error - previousError) / deltaT
  float d_term = 0.0;
  if (actualTs > 0.0) {
    float errorDerivative = (error - previousError) / actualTs;
    d_term = Kd * errorDerivative;
  }
  
  // Calculate raw controller output (PID)
  float raw_output = p_term + Ki * integral + d_term;
  
  // Saturate output to [0, 1023]
  int saturated_output = constrain((int)raw_output, MIN_VOLTAGE, MAX_VOLTAGE);
  
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

