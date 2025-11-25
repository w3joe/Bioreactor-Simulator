#ifndef HEATING_H
#define HEATING_H

// Temperature sensor constants
const float VREF = 3.3;
const int SERIES_RESISTOR = 10000;
const int ADC_MAX = 4095;  // 12-bit ADC

// Steinhart-Hart coefficients for thermistor
const double STEINHART_A = 1.009249522e-03;
const double STEINHART_B = 2.378405444e-04;
const double STEINHART_C = 2.019202697e-07;

// PID Controller state variables (static to maintain state between calls)
static float temp_integral = 0.0;           // Integral accumulator
static float temp_previousError = 0.0;      // Previous error for derivative calculation
static float targetTemp = 37.0;             // Target temperature in °C (default 37°C)
static float temp_Kp = 100.0;               // Proportional gain (default 100.0)
static float temp_Ki = 5.0;                 // Integral gain (default 5.0)
static float temp_Kd = 10.0;                // Derivative gain (default 10.0)
static float temp_antiWindupGain = 0.5;     // Anti-windup gain (default 0.5)
static bool tempControllerInitialized = false; // Initialization flag
static float currentTemp = 25.0;            // Current measured temperature
static float tempError = 0.5;               // Temperature measurement uncertainty

// Constants
const float MIN_TEMP_SETPOINT = 20.0;       // Minimum setpoint (°C)
const float MAX_TEMP_SETPOINT = 45.0;       // Maximum setpoint (°C)
const int MIN_HEATER_PWM = 0;               // Minimum heater PWM
const int MAX_HEATER_PWM = 1023;            // Maximum heater PWM (10-bit)
const int POWER_LIMIT_PWM = 639;            // 62.5% power limit
const float TEMP_SAMPLING_TIME = 0.1;       // Sampling time in seconds (100 ms)

/**
 * Calculate thermistor resistance from ADC reading
 * @param rawValue ADC reading (0-4095)
 * @return Resistance in ohms
 */
double calculateResistance(int rawValue) {
  // Prevent division by zero if rawValue is maxed out
  if (rawValue >= ADC_MAX) rawValue = ADC_MAX - 1;

  double voltage = rawValue * (VREF / (double)ADC_MAX);
  if (voltage == 0) return SERIES_RESISTOR;
  return SERIES_RESISTOR * (VREF / voltage - 1.0);
}

/**
 * Calculate temperature from thermistor resistance using Steinhart-Hart equation
 * @param resistance Thermistor resistance in ohms
 * @return Temperature in °C
 */
double calculateTemperature(double resistance) {
  double logR = log(resistance);
  double tempK = 1.0 / (STEINHART_A + STEINHART_B * logR + STEINHART_C * logR * logR * logR);
  return tempK - 273.15;
}

/**
 * Read temperature from sensor
 * @param analogPin ADC pin number
 * @return Temperature in °C
 */
float readTemperature(int analogPin) {
  int sensorValue = analogRead(analogPin);
  double resistance = calculateResistance(sensorValue);
  currentTemp = calculateTemperature(resistance);
  return currentTemp;
}

/**
 * Initialize the temperature PID controller
 * Resets integral accumulator and previous error
 * Should be called once in setup()
 */
void initTempController() {
  temp_integral = 0.0;
  temp_previousError = 0.0;
  tempControllerInitialized = true;
}

/**
 * Set the target temperature in °C
 * @param targetTempC Target temperature in °C (20-45)
 */
void setTargetTemp(float targetTempC) {
  // Validate and constrain setpoint
  if (targetTempC < MIN_TEMP_SETPOINT) {
    targetTemp = MIN_TEMP_SETPOINT;
  } else if (targetTempC > MAX_TEMP_SETPOINT) {
    targetTemp = MAX_TEMP_SETPOINT;
  } else {
    targetTemp = targetTempC;
  }
}

/**
 * Get the current target temperature
 * @return Current target temperature in °C
 */
float getTargetTemp() {
  return targetTemp;
}

/**
 * Get the current measured temperature
 * @return Current measured temperature in °C
 */
float getCurrentTemp() {
  return currentTemp;
}

/**
 * Get the temperature measurement error/uncertainty
 * @return Temperature error in °C
 */
float getTempError() {
  return tempError;
}

/**
 * Set temperature controller gains
 * @param Kp_new New proportional gain (1.0 - 200.0)
 * @param Ki_new New integral gain (0.0 - 20.0)
 * @param Kd_new New derivative gain (0.0 - 50.0)
 */
void setTempGains(float Kp_new, float Ki_new, float Kd_new = -1.0) {
  if (Kp_new >= 1.0 && Kp_new <= 200.0) {
    temp_Kp = Kp_new;
  }
  if (Ki_new >= 0.0 && Ki_new <= 20.0) {
    temp_Ki = Ki_new;
  }
  if (Kd_new >= 0.0 && Kd_new <= 50.0) {
    temp_Kd = Kd_new;
  }
}

/**
 * Reset the integral term and previous error
 * Useful when changing setpoint drastically or recovering from errors
 */
void resetTempIntegral() {
  temp_integral = 0.0;
  temp_previousError = 0.0;
}

/**
 * Get the current integral term value
 * @return Current integral accumulator value
 */
float getTempIntegralTerm() {
  return temp_integral;
}

/**
 * Calculate heater PWM using PID controller with anti-windup
 *
 * This is the main controller function. It implements:
 * - Discrete-time PID controller: u(k) = Kp * e(k) + Ki * Ts * sum(e(0)...e(k)) + Kd * (e(k) - e(k-1)) / Ts
 * - Back-calculation anti-windup to prevent integral saturation
 * - Output saturation to [0, POWER_LIMIT_PWM]
 *
 * @param setpoint Target temperature in °C
 * @param measTemp Current measured temperature in °C
 * @param deltaT Time since last update in seconds (should be ~0.1s for 100ms loop)
 * @return Heater PWM command (0-639, representing 0-62.5% power)
 */
int calculateHeaterPWM(float setpoint, float measTemp, float deltaT) {
  // Initialize controller on first call if not already done
  if (!tempControllerInitialized) {
    initTempController();
  }

  // Calculate error (setpoint - measured)
  float error = setpoint - measTemp;

  // Calculate proportional term
  float p_term = temp_Kp * error;

  // Update integral term (using actual deltaT, but typically 0.1s)
  // Use actual deltaT for better accuracy if timing varies slightly
  float actualTs = (deltaT > 0.0 && deltaT < 1.0) ? deltaT : TEMP_SAMPLING_TIME;
  temp_integral += error * actualTs;

  // Calculate derivative term
  // Derivative of error: (error - previousError) / deltaT
  float d_term = 0.0;
  if (actualTs > 0.0) {
    float errorDerivative = (error - temp_previousError) / actualTs;
    d_term = temp_Kd * errorDerivative;
  }

  // Calculate raw controller output (PID)
  float raw_output = p_term + temp_Ki * temp_integral + d_term;

  // Saturate output to [0, POWER_LIMIT_PWM] for safety
  int saturated_output = constrain((int)raw_output, MIN_HEATER_PWM, POWER_LIMIT_PWM);

  // Anti-windup: back-calculation method
  // If output is saturated, reduce integral term to prevent windup
  if ((int)raw_output != saturated_output) {
    float saturation_error = raw_output - (float)saturated_output;
    // Back-calculate to prevent windup
    if (temp_Ki > 0.0) {
      temp_integral -= (saturation_error / temp_Ki) * temp_antiWindupGain;
    }
  }

  // Update previous error for next iteration
  temp_previousError = error;

  return saturated_output;
}

/**
 * Check temperature controller health status
 * Detects potential sensor failure or heater malfunction
 * @param measTemp Current measured temperature in °C
 * @param heaterPWM Current heater PWM command
 * @return true if controller appears healthy, false if potential issue detected
 */
bool checkTempControllerHealth(float measTemp, int heaterPWM) {
  // If temperature is out of reasonable range, possible sensor failure
  if (measTemp < 0.0 || measTemp > 100.0) {
    return false; // Temperature reading is unrealistic
  }

  // If heater is at max but temperature is far below setpoint, possible heater failure
  if (heaterPWM >= POWER_LIMIT_PWM - 50 && (targetTemp - measTemp) > 10.0) {
    return false; // Heater may be malfunctioning
  }

  return true; // Controller appears healthy
}

#endif // HEATING_H
