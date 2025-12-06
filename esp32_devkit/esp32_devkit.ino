#include <Arduino.h>
#include "stirring.h"  // Bang-Bang Controller for stirring motor
#include "heating.h"   // Bang-Bang Controller for heating element
#include "ph_control.h" // pH sensor and pump control

// Stirring motor control variables
unsigned long currtime = 0;
unsigned long prevtime = 0;
unsigned long nextControlTime = 0;
float measspeed = 0.0;
int Vmotor = 0;
float deltaT = 0.01;
float freq = 0.0;
float freqtoRPM = 0.0;

// Rolling average for RPM (1 second window = 100 samples at 10ms rate)
#define RPM_BUFFER_SIZE 100
float rpmBuffer[RPM_BUFFER_SIZE];
int rpmBufferIndex = 0;
float rpmAverage = 0.0;

// Temperature control variables
float measTemp = 0.0;
int heaterPWM = 0;

// Rolling average for Temperature (1 second window = 100 samples at 10ms rate)
#define TEMP_BUFFER_SIZE 100
float tempBuffer[TEMP_BUFFER_SIZE];
int tempBufferIndex = 0;
float tempAverage = 0.0;

// Hall sensor variables (adjust pin and conversion factor as needed)
volatile unsigned long pulseCount = 0;
const float PULSES_PER_REV = 70.0;      // 70 pulses per revolution
const float SAMPLING_PERIOD_MS = 10.0;  // 10ms sampling period

//Avoid GPIO6-11
//Inputs: use GPIO34,35,36,39
#define STIRRING_SENSOR_PIN 2
#define PH_SENSOR_PIN 3
#define TEMP_SENSOR_PIN A0

#define STIRRING_MOTOR_PIN 10 
#define BASE_PUMP_PIN 8
#define ACID_PUMP_PIN 9
#define HEATING_ELEMENT_PIN 4

// PWM configuration for motor control
#define PWM_CHANNEL 0        // PWM channel (0-15)
#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 10    // 10-bit resolution (0-1023)

// Default initial setpoints (will be updated by TTGO via UART)
#define DEFAULT_RPM 500.0
#define DEFAULT_PH 7.0
#define DEFAULT_TEMP 35.0

// UART Communication pins (Bidirectional)
// Connect ESP32_DevKit GPIO11 (TX) to ESP32_TTGO GPIO25 (RX)
// Connect ESP32_DevKit GPIO12 (RX) to ESP32_TTGO GPIO26 (TX)
// Connect GND of both boards together
#define UART_TX_PIN 11  // TX pin for Serial2 (sends sensor data to TTGO)
#define UART_RX_PIN 12  // RX pin for Serial2 (receives setpoints from TTGO)
#define UART_BAUD 115200

// UART receive buffer for setpoint commands
String uartRxBuffer = "";
#define MAX_UART_BUFFER_SIZE 100

//interrupt routine
void freqcount() {
  pulseCount++;
}

void setup()
{
  Serial.begin(UART_BAUD);
  delay(1000);  // Wait for Serial to stabilize

  Serial.println("\n========================================");
  Serial.println("ESP32 DevKit Bioreactor Controller");
  Serial.println("========================================");

  // Initialize UART for communication with ESP32 TTGO
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("✓ UART initialized");

  // Configure PWM for stirring motor (10-bit resolution)
  ledcAttach(STIRRING_MOTOR_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(STIRRING_MOTOR_PIN, 0);  // Start with motor OFF (0=off, 1023=full speed)
  Serial.print("✓ Stirring motor pin: GPIO");
  Serial.print(STIRRING_MOTOR_PIN);
  Serial.println(" (10-bit PWM: 0-1023)");

  // Configure PWM for heating element (10-bit resolution)
  ledcAttach(HEATING_ELEMENT_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(HEATING_ELEMENT_PIN, 0);  // Start with heater OFF
  Serial.print("✓ Heating element pin: GPIO");
  // Serial.print(HEATING_ELEMENT_PIN);
  Serial.println(" (10-bit PWM: 0-400, 39% power limit)");

  // Initialize temperature sensor
  pinMode(TEMP_SENSOR_PIN, INPUT);
  Serial.print("✓ Temperature sensor: GPIO");
  Serial.println(TEMP_SENSOR_PIN);

  // Initialize hall sensor
  pinMode(STIRRING_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STIRRING_SENSOR_PIN), freqcount, RISING);
  Serial.print("✓ Hall sensor: GPIO");
  Serial.println(STIRRING_SENSOR_PIN);

  // Calculate conversion factor: RPM = (freq * 60) / PULSES_PER_REV
  freqtoRPM = 60.0 / PULSES_PER_REV;

  // Initialize RPM rolling average buffer
  for (int i = 0; i < RPM_BUFFER_SIZE; i++) {
    rpmBuffer[i] = 0.0;
  }

  // Initialize Temperature rolling average buffer
  for (int i = 0; i < TEMP_BUFFER_SIZE; i++) {
    tempBuffer[i] = 0.0;
  }

  // Initialize Bang-Bang controllers with default setpoints
  // These will be updated by TTGO via UART once connection is established
  initController();
  setTargetSpeed(DEFAULT_RPM);  // Default: 500 RPM

  initTempController();
  setTargetTemp(DEFAULT_TEMP);  // Default: 35°C

  initPHControl(PH_SENSOR_PIN, BASE_PUMP_PIN, ACID_PUMP_PIN);
  setTargetpH(DEFAULT_PH);  // Default: 7.0 pH

  // Initialize timing variables
  currtime = micros();
  prevtime = currtime;
  nextControlTime = currtime + 10000;  // Start first control cycle in 10ms

  Serial.print("✓ Stirring controller initialized - Target: ");
  Serial.print(getTargetSpeed());
  Serial.println(" RPM");
  Serial.print("✓ Temperature controller initialized - Target: ");
  Serial.print(getTargetTemp());
  Serial.println(" °C");
  Serial.print("✓ pH controller initialized - Target: ");
  Serial.print(getTargetpH());
  Serial.println(" pH");
  Serial.println("========================================\n");

  Serial.println("Starting control loops NOW");
  Serial.println("RPM_Target,RPM_Measured,RPM_Error,RPM_PWM,Hysteresis,MotorPWM,Temp_Target,Temp_Measured,Temp_Error,HeaterPWM,PH_Target,PH_Measured,PumpState");
  Serial.println("\nWaiting for setpoint commands from TTGO...");
  Serial.println("Expected format: SET:RPM=500.0,TEMP=35.0,PH=7.0");
}

void loop()
{
  // =========================================================================
  // Check for Setpoint Commands from TTGO (Non-blocking)
  // =========================================================================
  checkForSetpointCommands();

  // =========================================================================
  // pH Control Update (Non-blocking)
  // =========================================================================
  updatePHControl();

  // =========================================================================
  // Control Loop (10ms update rate)
  // =========================================================================
  currtime = micros();
  deltaT = (currtime - prevtime) * 1e-6;  // Convert microseconds to seconds

  if (currtime >= nextControlTime) {
    prevtime = currtime;
    nextControlTime = nextControlTime + 10000;  // 10 ms = 10000 microseconds

    // =========================================================================
    // Stirring Motor Control
    // =========================================================================
    // Calculate frequency from pulse count over the sampling period
    // Frequency = pulseCount / (sampling period in seconds)
    unsigned long currentPulseCount = pulseCount;
    pulseCount = 0;  // Reset counter for next period
    float samplingPeriod = SAMPLING_PERIOD_MS / 1000.0;  // Convert to seconds
    freq = (float)currentPulseCount / samplingPeriod;

    // Convert frequency to RPM
    measspeed = freq * freqtoRPM;

    // Add current RPM to rolling average buffer
    rpmBuffer[rpmBufferIndex] = measspeed;
    rpmBufferIndex = (rpmBufferIndex + 1) % RPM_BUFFER_SIZE;

    // Calculate rolling average over 500ms
    float rpmSum = 0.0;
    for (int i = 0; i < RPM_BUFFER_SIZE; i++) {
      rpmSum += rpmBuffer[i];
    }
    rpmAverage = rpmSum / RPM_BUFFER_SIZE;

    // Bang-Bang Controller calculates motor voltage (0 or 1023)
    Vmotor = calculateMotorVoltage(getTargetSpeed(), measspeed, deltaT);

    // Bang-bang outputs either 0 (OFF) or 1023 (FULL SPEED)
    int motorPWM = Vmotor;

    // Output PWM to motor using LEDC
    ledcWrite(STIRRING_MOTOR_PIN, motorPWM);

    // Check stirring controller health
    if (!checkControllerHealth(measspeed, Vmotor)) {
      // Serial.println("WARNING: Stirring controller health check failed");
    }

    // Calculate stirring error
    float rpmError = getTargetSpeed() - measspeed;

    // =========================================================================
    // Temperature Control
    // =========================================================================
    // Read temperature from thermistor
    measTemp = readTemperature(TEMP_SENSOR_PIN);

    // Add current temperature to rolling average buffer
    tempBuffer[tempBufferIndex] = measTemp;
    tempBufferIndex = (tempBufferIndex + 1) % TEMP_BUFFER_SIZE;

    // Calculate rolling average over 1 second
    float tempSum = 0.0;
    for (int i = 0; i < TEMP_BUFFER_SIZE; i++) {
      tempSum += tempBuffer[i];
    }
    tempAverage = tempSum / TEMP_BUFFER_SIZE;

    // Bang-Bang Controller calculates heater PWM (0 or PWM_POWER_VALUE)
    heaterPWM = calculateHeaterPWM(getTargetTemp(), measTemp, deltaT);

    // Output PWM to heater using LEDC
    ledcWrite(HEATING_ELEMENT_PIN, heaterPWM);

    // Check temperature controller health
    if (!checkTempControllerHealth(measTemp, heaterPWM)) {
      // Serial.println("WARNING: Temperature controller health check failed");
    }

    // Calculate temperature error
    float tempError = getTargetTemp() - measTemp;

    // =========================================================================
    // UART Communication to Display Board (Send every 100ms = every 10 loops)
    // =========================================================================
    static unsigned long loopCounter = 0;
    loopCounter++;
    if (loopCounter % 10 == 0) {  // Send every 10th loop (10ms × 10 = 100ms)
      // Send data via UART to ESP32 TTGO display board
      // Format: RPM_TARGET,RPM_MEASURED,RPM_ERROR,RPM_PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM,PH_TARGET,PH_MEASURED,PUMP_STATE\n
      // Use 1 second rolling average (100 samples) for RPM and Temperature display and server transmission
      float rpmErrorAvg = getTargetSpeed() - rpmAverage;
      float tempErrorAvg = getTargetTemp() - tempAverage;

      Serial2.print(getTargetSpeed(), 1);
      Serial2.print(",");
      Serial2.print(rpmAverage, 1);  // Send 1s averaged RPM
      Serial2.print(",");
      Serial2.print(rpmErrorAvg, 1);  // Error based on averaged RPM
      Serial2.print(",");
      Serial2.print(Vmotor);
      Serial2.print(",");
      Serial2.print(getTargetTemp(), 1);
      Serial2.print(",");
      Serial2.print(tempAverage, 1);  // Send 1s averaged Temperature
      Serial2.print(",");
      Serial2.print(tempErrorAvg, 1);  // Error based on averaged Temperature
      Serial2.print(",");
      Serial2.print(heaterPWM);
      Serial2.print(",");
      Serial2.print(getTargetpH(), 2);
      Serial2.print(",");
      Serial2.print(getCurrentpH(), 2);
      Serial2.print(",");
      Serial2.print(getPumpState());
      Serial2.print("\n");

      // Debug: Confirm UART transmission every 100 sends (every 10 seconds)
      static unsigned long txCount = 0;
      txCount++;
      if (txCount % 100 == 0) {
        Serial.print("[DEBUG] UART TX sent #");
        Serial.println(txCount);
      }
    }

    // =========================================================================
    // Serial Plotter Output
    // =========================================================================
    // Print data for Serial Plotter (comma-separated values)
    // Format: RPM_Target,RPM_Measured,RPM_Error,RPM_PWM,Hysteresis,MotorPWM,Temp_Target,Temp_Measured,Temp_Error,HeaterPWM,PH_Target,PH_Measured,PumpState
    Serial.print(getTargetSpeed());
    Serial.print(",");
    Serial.print(measspeed);
    Serial.print(",");
    Serial.print(rpmError);
    Serial.print(",");
    Serial.print(Vmotor);
    Serial.print(",");
    Serial.print(getIntegralTerm());  // Returns hysteresis value for stirring
    Serial.print(",");
    Serial.print(motorPWM);
    Serial.print(",");
    Serial.print(getTargetTemp());
    Serial.print(",");
    Serial.print(measTemp);
    Serial.print(",");
    Serial.print(tempError);
    Serial.print(",");
    Serial.print(heaterPWM);
    Serial.print("\n");
    Serial.print(",");
    Serial.print(getTargetpH());
    Serial.print(",");
    Serial.print(readPHVoltage());
    Serial.print(",");
    Serial.println(getPumpState());
  }
}

// =========================================================================
// Non-blocking function to check for and parse setpoint commands from TTGO
// Format: SET:RPM=500.0,TEMP=35.0\n
// =========================================================================
void checkForSetpointCommands() {
  // Process available UART data without blocking
  while (Serial2.available() > 0) {
    char c = Serial2.read();

    if (c == '\n') {
      // End of command - parse it
      if (uartRxBuffer.length() > 0) {
        parseSetpointCommand(uartRxBuffer);
      }
      uartRxBuffer = "";  // Clear buffer
    } else if (c == '\r') {
      // Ignore carriage return
      continue;
    } else {
      uartRxBuffer += c;

      // Buffer overflow protection
      if (uartRxBuffer.length() > MAX_UART_BUFFER_SIZE) {
        Serial.print("[UART RX] ERROR: Buffer overflow! Clearing buffer. Content: ");
        Serial.println(uartRxBuffer.substring(0, 50));
        uartRxBuffer = "";
      }
    }
  }
}

// =========================================================================
// Parse setpoint command and update controller targets
// Expected format: SET:RPM=500.0,TEMP=35.0,PH=7.0
// =========================================================================
void parseSetpointCommand(String command) {
  // Check if this is a setpoint command
  if (!command.startsWith("SET:")) {
    Serial.print("[UART RX] WARNING: Unknown command format: ");
    Serial.println(command);
    return;
  }

  // Remove "SET:" prefix
  String data = command.substring(4);

  // Parse RPM, TEMP, and PH values
  int rpmIndex = data.indexOf("RPM=");
  int tempIndex = data.indexOf("TEMP=");
  int phIndex = data.indexOf("PH=");

  // Parse and validate RPM setpoint
  if (rpmIndex >= 0) {
    int rpmEnd = data.indexOf(',', rpmIndex);
    if (rpmEnd < 0) rpmEnd = data.length();

    String rpmStr = data.substring(rpmIndex + 4, rpmEnd);
    float newRPM = rpmStr.toFloat();

    if (newRPM >= 0.0 && newRPM <= 1000.0) {
      setTargetSpeed(newRPM);
      Serial.print("[UART RX] ✓ Updated RPM setpoint: ");
      Serial.print(newRPM, 1);
      Serial.println(" RPM");
    } else {
      Serial.print("[UART RX] ERROR: Invalid RPM value: ");
      Serial.println(newRPM);
    }
  }

  // Parse and validate TEMP setpoint
  if (tempIndex >= 0) {
    int tempEnd = data.indexOf(',', tempIndex);
    if (tempEnd < 0) tempEnd = data.length();

    String tempStr = data.substring(tempIndex + 5, tempEnd);
    float newTemp = tempStr.toFloat();

    if (newTemp >= 20.0 && newTemp <= 45.0) {
      setTargetTemp(newTemp);
      Serial.print("[UART RX] ✓ Updated TEMP setpoint: ");
      Serial.print(newTemp, 1);
      Serial.println(" °C");
    } else {
      Serial.print("[UART RX] ERROR: Invalid TEMP value: ");
      Serial.println(newTemp);
    }
  }

  // Parse and validate pH setpoint
  if (phIndex >= 0) {
    int phEnd = data.indexOf(',', phIndex);
    if (phEnd < 0) phEnd = data.length();

    String phStr = data.substring(phIndex + 3, phEnd);
    float newPH = phStr.toFloat();

    if (newPH >= 4.0 && newPH <= 10.0) {
      setTargetpH(newPH);
      Serial.print("[UART RX] ✓ Updated PH setpoint: ");
      Serial.print(newPH, 2);
      Serial.println(" pH");
    } else {
      Serial.print("[UART RX] ERROR: Invalid PH value: ");
      Serial.println(newPH);
    }
  }

  // Verify at least one valid parameter was found
  if (rpmIndex < 0 && tempIndex < 0 && phIndex < 0) {
    Serial.print("[UART RX] ERROR: Invalid setpoint format: ");
    Serial.println(command);
    Serial.println("Expected: SET:RPM=500.0,TEMP=35.0,PH=7.0");
  }
}