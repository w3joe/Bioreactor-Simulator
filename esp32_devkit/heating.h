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

// Bang-Bang Controller state variables
static float targetTemp = 0.0;              // Target temperature in °C (initialized to 0, set by user)
static float tempDeltaT = 0.5;              // Hysteresis band (±0.5°C around setpoint)
static bool tempControllerInitialized = false; // Initialization flag
static float currentTemp = 25.0;            // Current measured temperature
static int currentHeaterPwm = 0;            // Current heater PWM state

// PWM Settings
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 10;              // 0-1023

// Constants
const float MIN_TEMP_SETPOINT = 20.0;       // Minimum setpoint (°C)
const float MAX_TEMP_SETPOINT = 45.0;       // Maximum setpoint (°C)
const int PWM_POWER_VALUE = 400;            // 39% Power (400/1023)
const int PWM_OFF_VALUE = 0;                // 0% Power (OFF)
const float TEMP_SAMPLING_TIME = 0.1;       // Sampling time in seconds (100 ms)

/**
 * Calculate thermistor resistance from ADC reading
 * @param rawValue ADC reading (0-4095)
 * @return Resistance in ohms
 */
double calculateResistance(int rawValue) {
  double voltage = rawValue * (VREF / 4095.0);
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

  // Debug output (uncomment to debug temperature readings)
  // Serial.print("ADC: "); Serial.print(sensorValue);
  // Serial.print(" | Resistance: "); Serial.print(resistance);
  // Serial.print(" | Temp: "); Serial.println(currentTemp);

  return currentTemp;
}

/**
 * Initialize the temperature bang-bang controller
 * Should be called once in setup()
 */
void initTempController() {
  tempControllerInitialized = true;
  currentHeaterPwm = 0;
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
 * Get the current hysteresis band
 * @return Hysteresis band in °C
 */
float getTempHysteresis() {
  return tempDeltaT;
}

/**
 * Set hysteresis band for bang-bang controller
 * @param deltaT New hysteresis value (°C, typically 0.3-1.0)
 */
void setTempHysteresis(float deltaT) {
  if (deltaT >= 0.1 && deltaT <= 2.0) {
    tempDeltaT = deltaT;
  }
}

/**
 * Get the current heater PWM state
 * @return Current PWM value (0 or PWM_POWER_VALUE)
 */
int getCurrentHeaterPWM() {
  return currentHeaterPwm;
}

/**
 * Calculate heater PWM using Bang-Bang controller with hysteresis
 *
 * Bang-bang control (also called on-off control):
 * - If temp < (target - deltaT): heater ON at limited power (39%)
 * - If temp > (target + deltaT): heater OFF
 * - If temp within deltaT band: maintain previous state
 *
 * This prevents oscillation by creating a dead zone around the setpoint.
 *
 * @param setpoint Target temperature in °C
 * @param measTemp Current measured temperature in °C
 * @param deltaT Time since last update (unused for bang-bang)
 * @return Heater PWM command (0 or PWM_POWER_VALUE)
 */
int calculateHeaterPWM(float setpoint, float measTemp, float deltaT) {
  // Initialize controller on first call if not already done
  if (!tempControllerInitialized) {
    initTempController();
  }

  // Calculate thresholds
  float lowerThreshold = setpoint - tempDeltaT;
  float upperThreshold = setpoint + tempDeltaT;

  // Bang-bang logic with hysteresis
  // If temp is too low, set PWM to Power Limit (Turn ON)
  if (measTemp < lowerThreshold) {
    currentHeaterPwm = PWM_POWER_VALUE;
  }
  // If temp is too high, set PWM to 0 (Turn OFF)
  else if (measTemp > upperThreshold) {
    currentHeaterPwm = PWM_OFF_VALUE;
  }
  // else: within hysteresis band - maintain previous state (no change)

  return currentHeaterPwm;
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

  // If heater is ON but temperature is far below setpoint, possible heater failure
  if (heaterPWM >= PWM_POWER_VALUE && (targetTemp - measTemp) > 10.0) {
    return false; // Heater may be malfunctioning
  }

  return true; // Controller appears healthy
}

#endif // HEATING_H
