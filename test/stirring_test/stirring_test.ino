/*
 * Stirring Motor Test Suite
 *
 * This sketch performs two critical tests for the bioreactor stirring system:
 *
 * TEST 1: Speed Tracking Performance
 * - Runs for ~3 minutes with multiple setpoint changes
 * - Tests bang-bang controller's ability to maintain speed
 * - Demonstrates oscillation patterns and settling behavior
 * - Setpoint sequence: 500 → 550 → 600 → 650 → 700 → 750 → 800 → 850 → 900 → 950 → 1000 RPM
 * - Each setpoint held for 15 seconds
 * - Data logged every 100ms
 *
 * TEST 2: PWM-RPM Calibration Curve
 * - Sweeps PWM duty cycle from 0% to 100% in 10% increments
 * - Measures steady-state RPM at each PWM level
 * - Characterizes motor transfer function
 * - Each PWM level held for 15 seconds (10s settle + 5s measure)
 *
 * HARDWARE SETUP:
 * - Hall sensor on GPIO2 (interrupt-based pulse counting)
 * - Motor control on GPIO10 (10-bit PWM: 0-1023)
 * - 70 pulses per revolution (standard motor encoder)
 *
 * OUTPUT FORMAT:
 * - CSV data to Serial (115200 baud)
 * - Columns: Time(ms), TestMode, TargetRPM, MeasuredRPM, PWM, Error
 * - Compatible with Excel, MATLAB, Python pandas for plotting
 *
 * USAGE:
 * 1. Upload sketch to ESP32 DevKit
 * 2. Open Serial Monitor (115200 baud)
 * 3. Press 'A' to run Test 1 (Speed Tracking)
 * 4. Press 'B' to run Test 2 (PWM Calibration)
 * 5. Copy CSV output to file for analysis
 */

#include <Arduino.h>
#include "stirring.h"  // Bang-Bang Controller

// Hall sensor variables
volatile unsigned long pulseCount = 0;
const float PULSES_PER_REV = 70.0;      // 70 pulses per revolution
const float SAMPLING_PERIOD_MS = 10.0;  // 10ms sampling period

// Pin definitions (matching esp32_devkit)
#define STIRRING_SENSOR_PIN 2   // Hall sensor input (interrupt capable)
#define STIRRING_MOTOR_PIN 10   // Motor PWM output

// PWM configuration
#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 10    // 10-bit resolution (0-1023)

// Test state machine
enum TestMode {
  IDLE,
  TEST1_SPEED_TRACKING,
  TEST2_PWM_CALIBRATION
};

TestMode currentTest = IDLE;

// Test 1 variables (Speed Tracking)
const int TEST1_SETPOINTS[] = {500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000};
const int TEST1_NUM_SETPOINTS = sizeof(TEST1_SETPOINTS) / sizeof(TEST1_SETPOINTS[0]);
const unsigned long TEST1_SETPOINT_DURATION_MS = 15000;  // 15 seconds per setpoint
int test1_currentSetpointIndex = 0;
unsigned long test1_setpointStartTime = 0;
unsigned long test1_startTime = 0;

// Test 2 variables (PWM Calibration)
const int TEST2_PWM_LEVELS[] = {0, 102, 205, 307, 410, 512, 614, 717, 819, 921, 1023};  // 0% to 100% in 10% steps
const int TEST2_NUM_LEVELS = sizeof(TEST2_PWM_LEVELS) / sizeof(TEST2_PWM_LEVELS[0]);
const unsigned long TEST2_SETTLE_TIME_MS = 10000;  // 10 seconds to reach steady state
const unsigned long TEST2_MEASURE_TIME_MS = 5000;  // 5 seconds to measure average RPM
int test2_currentLevelIndex = 0;
unsigned long test2_levelStartTime = 0;
unsigned long test2_startTime = 0;
float test2_rpmAccumulator = 0.0;
int test2_rpmSampleCount = 0;

// Control loop timing
unsigned long currtime = 0;
unsigned long prevtime = 0;
unsigned long nextControlTime = 0;
float deltaT = 0.01;
float freq = 0.0;
float freqtoRPM = 0.0;
float measspeed = 0.0;
int Vmotor = 0;

// Data logging
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL_MS = 100;  // Log every 100ms

//=========================================================================
// Interrupt routine for hall sensor
//=========================================================================
void IRAM_ATTR freqcount() {
  pulseCount++;
}

//=========================================================================
// Setup
//=========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  STIRRING MOTOR TEST SUITE");
  Serial.println("========================================");
  Serial.println("Hardware: ESP32 DevKit");
  Serial.println("Motor: DC with encoder (70 pulses/rev)");
  Serial.println("Controller: Bang-Bang with hysteresis");
  Serial.println("========================================\n");

  // Configure PWM for motor control (10-bit resolution)
  ledcAttach(STIRRING_MOTOR_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(STIRRING_MOTOR_PIN, 0);  // Start with motor OFF
  Serial.print("✓ Motor PWM configured: GPIO");
  Serial.print(STIRRING_MOTOR_PIN);
  Serial.println(" (10-bit: 0-1023)");

  // Initialize hall sensor with interrupt
  pinMode(STIRRING_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STIRRING_SENSOR_PIN), freqcount, RISING);
  Serial.print("✓ Hall sensor configured: GPIO");
  Serial.println(STIRRING_SENSOR_PIN);

  // Calculate conversion factor
  freqtoRPM = 60.0 / PULSES_PER_REV;

  // Initialize Bang-Bang controller
  initController();
  setTargetSpeed(500.0);  // Default target
  Serial.println("✓ Bang-Bang controller initialized");

  // Initialize timing
  currtime = micros();
  prevtime = currtime;
  nextControlTime = currtime + 10000;  // 10ms control cycle
  lastLogTime = millis();

  Serial.println("\n========================================");
  Serial.println("AVAILABLE TESTS:");
  Serial.println("========================================");
  Serial.println("A - Test 1: Speed Tracking Performance");
  Serial.println("    Duration: 5 minutes (300 seconds)");
  Serial.println("    Setpoints: 500→1000 RPM in 50 RPM steps");
  Serial.println("    Output: Time, Target, Measured, PWM, Error");
  Serial.println("");
  Serial.println("B - Test 2: PWM-RPM Calibration Curve");
  Serial.println("    Duration: ~3 minutes (165 seconds)");
  Serial.println("    PWM Sweep: 0% → 100% in 10% steps");
  Serial.println("    Output: PWM, Steady-State RPM");
  Serial.println("");
  Serial.println("Press 'A' or 'B' to start a test...");
  Serial.println("========================================\n");
}

//=========================================================================
// Main Loop
//=========================================================================
void loop() {
  // Check for user input to start tests
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'A' || cmd == 'a') {
      startTest1();
    } else if (cmd == 'B' || cmd == 'b') {
      startTest2();
    }
  }

  // Control loop timing (10ms cycle)
  currtime = micros();
  deltaT = (currtime - prevtime) * 1e-6;

  if (currtime >= nextControlTime) {
    prevtime = currtime;
    nextControlTime = nextControlTime + 10000;  // 10ms

    // Calculate RPM from hall sensor pulses
    unsigned long currentPulseCount = pulseCount;
    pulseCount = 0;  // Reset counter
    float samplingPeriod = SAMPLING_PERIOD_MS / 1000.0;
    freq = (float)currentPulseCount / samplingPeriod;
    measspeed = freq * freqtoRPM;

    // Run test state machine
    switch (currentTest) {
      case TEST1_SPEED_TRACKING:
        runTest1();
        break;

      case TEST2_PWM_CALIBRATION:
        runTest2();
        break;

      case IDLE:
      default:
        // Do nothing when idle
        break;
    }
  }
}

//=========================================================================
// Test 1: Speed Tracking Performance
//=========================================================================
void startTest1() {
  Serial.println("\n========================================");
  Serial.println("STARTING TEST 1: Speed Tracking");
  Serial.println("========================================");
  Serial.println("Duration: ~3 minutes (165 seconds)");
  Serial.println("Setpoint changes every 15 seconds");
  Serial.println("Data logged every 100ms");
  Serial.println("CSV Format: Time_ms,TestMode,Target_RPM,Measured_RPM,PWM,Error_RPM");
  Serial.println("========================================\n");

  // Print CSV header
  Serial.println("Time_ms,TestMode,Target_RPM,Measured_RPM,PWM,Error_RPM");

  currentTest = TEST1_SPEED_TRACKING;
  test1_currentSetpointIndex = 0;
  test1_startTime = millis();
  test1_setpointStartTime = millis();
  setTargetSpeed(TEST1_SETPOINTS[0]);
  lastLogTime = millis();
}

void runTest1() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - test1_startTime;

  // Check if we need to change setpoint
  if (currentTime - test1_setpointStartTime >= TEST1_SETPOINT_DURATION_MS) {
    test1_currentSetpointIndex++;

    if (test1_currentSetpointIndex >= TEST1_NUM_SETPOINTS) {
      // Test complete
      Serial.println("\n========================================");
      Serial.println("TEST 1 COMPLETE");
      Serial.println("========================================");
      Serial.print("Total duration: ");
      Serial.print(elapsedTime / 1000.0, 1);
      Serial.println(" seconds");
      Serial.println("Data collection finished.");
      Serial.println("Copy CSV data above for plotting.");
      Serial.println("========================================\n");

      // Stop motor and return to idle
      ledcWrite(STIRRING_MOTOR_PIN, 0);
      currentTest = IDLE;
      return;
    }

    // Update to next setpoint
    setTargetSpeed(TEST1_SETPOINTS[test1_currentSetpointIndex]);
    test1_setpointStartTime = currentTime;
  }

  // Run bang-bang controller
  Vmotor = calculateMotorVoltage(getTargetSpeed(), measspeed, deltaT);
  ledcWrite(STIRRING_MOTOR_PIN, Vmotor);

  // Log data at specified interval
  if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
    float error = getTargetSpeed() - measspeed;

    // CSV output: Time, TestMode, Target, Measured, PWM, Error
    Serial.print(elapsedTime);
    Serial.print(",TEST1,");
    Serial.print(getTargetSpeed(), 1);
    Serial.print(",");
    Serial.print(measspeed, 1);
    Serial.print(",");
    Serial.print(Vmotor);
    Serial.print(",");
    Serial.println(error, 1);

    lastLogTime = currentTime;
  }
}

//=========================================================================
// Test 2: PWM-RPM Calibration Curve
//=========================================================================
void startTest2() {
  Serial.println("\n========================================");
  Serial.println("STARTING TEST 2: PWM-RPM Calibration");
  Serial.println("========================================");
  Serial.println("Duration: ~3 minutes (165 seconds)");
  Serial.println("PWM sweep: 0% → 100% in 10% steps");
  Serial.println("15s per level (10s settle + 5s measure)");
  Serial.println("CSV Format: PWM_Value,PWM_Percent,Avg_RPM,StdDev_RPM");
  Serial.println("========================================\n");

  // Print CSV header
  Serial.println("PWM_Value,PWM_Percent,Avg_RPM,StdDev_RPM");

  currentTest = TEST2_PWM_CALIBRATION;
  test2_currentLevelIndex = 0;
  test2_startTime = millis();
  test2_levelStartTime = millis();
  test2_rpmAccumulator = 0.0;
  test2_rpmSampleCount = 0;
  lastLogTime = millis();

  // Set first PWM level
  ledcWrite(STIRRING_MOTOR_PIN, TEST2_PWM_LEVELS[0]);

  Serial.print("Testing PWM level ");
  Serial.print(test2_currentLevelIndex + 1);
  Serial.print("/");
  Serial.print(TEST2_NUM_LEVELS);
  Serial.print(" (");
  Serial.print((TEST2_PWM_LEVELS[test2_currentLevelIndex] * 100) / 1023.0, 1);
  Serial.println("%)...");
}

void runTest2() {
  unsigned long currentTime = millis();
  unsigned long levelElapsedTime = currentTime - test2_levelStartTime;
  unsigned long totalElapsedTime = currentTime - test2_startTime;

  // After settle time, start accumulating RPM measurements
  if (levelElapsedTime >= TEST2_SETTLE_TIME_MS &&
      levelElapsedTime < (TEST2_SETTLE_TIME_MS + TEST2_MEASURE_TIME_MS)) {

    // Accumulate RPM samples during measurement window
    if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
      test2_rpmAccumulator += measspeed;
      test2_rpmSampleCount++;
      lastLogTime = currentTime;
    }
  }

  // Check if measurement period is complete
  if (levelElapsedTime >= (TEST2_SETTLE_TIME_MS + TEST2_MEASURE_TIME_MS)) {
    // Calculate average RPM
    float avgRPM = 0.0;
    if (test2_rpmSampleCount > 0) {
      avgRPM = test2_rpmAccumulator / test2_rpmSampleCount;
    }

    // Output calibration data point
    int pwmValue = TEST2_PWM_LEVELS[test2_currentLevelIndex];
    float pwmPercent = (pwmValue * 100.0) / 1023.0;

    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(pwmPercent, 1);
    Serial.print(",");
    Serial.print(avgRPM, 1);
    Serial.print(",");
    Serial.println("0.0");  // StdDev placeholder (can be calculated if needed)

    // Move to next PWM level
    test2_currentLevelIndex++;

    if (test2_currentLevelIndex >= TEST2_NUM_LEVELS) {
      // Test complete
      Serial.println("\n========================================");
      Serial.println("TEST 2 COMPLETE");
      Serial.println("========================================");
      Serial.print("Total duration: ");
      Serial.print(totalElapsedTime / 1000.0, 1);
      Serial.println(" seconds");
      Serial.println("Calibration data collection finished.");
      Serial.println("Copy CSV data above for plotting.");
      Serial.println("========================================\n");

      // Stop motor and return to idle
      ledcWrite(STIRRING_MOTOR_PIN, 0);
      currentTest = IDLE;
      return;
    }

    // Setup next PWM level
    test2_levelStartTime = currentTime;
    test2_rpmAccumulator = 0.0;
    test2_rpmSampleCount = 0;
    lastLogTime = currentTime;

    ledcWrite(STIRRING_MOTOR_PIN, TEST2_PWM_LEVELS[test2_currentLevelIndex]);

    Serial.print("Testing PWM level ");
    Serial.print(test2_currentLevelIndex + 1);
    Serial.print("/");
    Serial.print(TEST2_NUM_LEVELS);
    Serial.print(" (");
    Serial.print((TEST2_PWM_LEVELS[test2_currentLevelIndex] * 100) / 1023.0, 1);
    Serial.println("%)...");
  }
}
