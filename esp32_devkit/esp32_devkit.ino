#include <Arduino.h>
#include "stirring.h"  // Bang-Bang Controller for stirring motor
#include "heating.h"   // Bang-Bang Controller for heating element

// Stirring motor control variables
unsigned long currtime = 0;
unsigned long prevtime = 0;
unsigned long nextControlTime = 0;
float measspeed = 0.0;
int Vmotor = 0;
float deltaT = 0.01;
float freq = 0.0;
float freqtoRPM = 0.0;

// Temperature control variables
float measTemp = 0.0;
int heaterPWM = 0;

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

#define TARGET_RPM 400.0
#define TARGET_PH 5.0
#define TARGET_TEMP 35.0

// UART Communication pins
// Connect ESP32_Nano GPIO17 (TX2) to ESP32_TTGO GPIO25 (RX)
// Connect GND of both boards together
#define UART_TX_PIN 11  // TX pin for Serial2 (use GPIO instead of GPIO1 to avoid USB conflict)
#define UART_RX_PIN 12  // RX pin for Serial2 (use GPIO16 instead of GPIO0)
#define UART_BAUD 115200

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

  // Initialize Bang-Bang controllers
  initController();
  setTargetSpeed(500.0);  // Set initial target speed to 500 RPM

  initTempController();
  setTargetTemp(TARGET_TEMP);  // Set initial target temperature (35°C)

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
  Serial.println("========================================\n");

  Serial.println("Starting control loops NOW");
  Serial.println("RPM_Target,RPM_Measured,RPM_Error,RPM_PWM,Hysteresis,MotorPWM,Temp_Target,Temp_Measured,Temp_Error,HeaterPWM");
}

void loop()
{
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
      // Format: RPM_TARGET,RPM_MEASURED,RPM_ERROR,RPM_PWM,TEMP_TARGET,TEMP_MEASURED,TEMP_ERROR,HEATER_PWM\n
      Serial2.print(getTargetSpeed(), 1);
      Serial2.print(",");
      Serial2.print(measspeed, 1);
      Serial2.print(",");
      Serial2.print(rpmError, 1);
      Serial2.print(",");
      Serial2.print(Vmotor);
      Serial2.print(",");
      Serial2.print(getTargetTemp(), 1);
      Serial2.print(",");
      Serial2.print(measTemp, 1);
      Serial2.print(",");
      Serial2.print(tempError, 1);
      Serial2.print(",");
      Serial2.print(heaterPWM);
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
    // Format: RPM_Target,RPM_Measured,RPM_Error,RPM_PWM,Hysteresis,MotorPWM,Temp_Target,Temp_Measured,Temp_Error,HeaterPWM
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
    Serial.println(heaterPWM);
  }
}