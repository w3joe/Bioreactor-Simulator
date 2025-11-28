#include <Arduino.h>
#include "stirring.h"  // PI Controller for stirring motor

// Stirring motor control variables
unsigned long currtime = 0;        
unsigned long prevtime = 0;       
unsigned long nextControlTime = 0; 
float measspeed = 0.0;             
int Vmotor = 0;                   
float deltaT = 0.01;               
float freq = 0.0;                  
float freqtoRPM = 0.0;        

// Hall sensor variables (adjust pin and conversion factor as needed)
volatile unsigned long pulseCount = 0;  
const float PULSES_PER_REV = 70.0;      // 70 pulses per revolution
const float SAMPLING_PERIOD_MS = 10.0;  // 10ms sampling period

//Avoid GPIO6-11
//Inputs: use GPIO34,35,36,39
#define STIRRING_SENSOR_PIN 2
#define TEMP_SENSOR_PIN 34
#define PH_SENSOR_PIN 35

#define STIRRING_MOTOR_PIN 10  // Changed from GPIO10 (restricted) to GPIO25
#define HEATING_ELEMENT_PIN 26
#define BASE_PUMP_PIN 8
#define ACID_PUMP_PIN 9

// PWM configuration for motor control
#define PWM_CHANNEL 0        // PWM channel (0-15)
#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)

// UART Communication pins
// Connect ESP32_Arduino_Nano GPIO4 (TX) to ESP32_TTGO GPIO16 (RX)
// Connect GND of both boards together
#define UART_TX_PIN 1  // TX pin for Serial2
#define UART_RX_PIN 0  // RX pin for Serial2 (not used for sending, but needed for Serial2 initialization)
#define UART_BAUD 115200

//interrupt routine
void freqcount() {
  pulseCount++;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to stabilize

  Serial.println("\n========================================");
  Serial.println("ESP32 DevKit Bioreactor Controller");
  Serial.println("========================================");

  // Initialize UART for communication with ESP32 TTGO
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("✓ UART initialized");

  // Initialize motor pin
  pinMode(STIRRING_MOTOR_PIN, OUTPUT);
  analogWrite(STIRRING_MOTOR_PIN, 0);  // Start with motor OFF (0=off, 255=full speed)
  Serial.print("✓ Motor pin configured: GPIO");
  Serial.print(STIRRING_MOTOR_PIN);
  Serial.println(" (8-bit PWM: 0-255, NORMAL)");

  // Initialize hall sensor
  pinMode(STIRRING_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STIRRING_SENSOR_PIN), freqcount, RISING);
  Serial.print("✓ Hall sensor: GPIO");
  Serial.println(STIRRING_SENSOR_PIN);

  // Calculate conversion factor: RPM = (freq * 60) / PULSES_PER_REV
  freqtoRPM = 60.0 / PULSES_PER_REV;

  // Initialize PID controller
  initController();
  setTargetSpeed(100.0);  // Set initial target speed to 100 RPM

  // Initialize timing variables
  currtime = micros();
  prevtime = currtime;
  nextControlTime = currtime;

  Serial.print("✓ PID controller initialized - Target: ");
  Serial.print(getTargetSpeed());
  Serial.println(" RPM");
  Serial.println("========================================\n");

  Serial.println("Starting motor control NOW");
  Serial.println("Target,Measured,Error,PWM,Integral");
}

void loop()
{
  // =========================================================================
  // Stirring Motor Control Loop (10ms update rate)
  // =========================================================================
  currtime = micros();
  deltaT = (currtime - prevtime) * 1e-6;  // Convert microseconds to seconds
  
  if (currtime >= nextControlTime) {
    prevtime = currtime;
    nextControlTime = nextControlTime + 10000;  // 10 ms = 10000 microseconds
    
    // Calculate frequency from pulse count over the sampling period
    // Frequency = pulseCount / (sampling period in seconds)
    unsigned long currentPulseCount = pulseCount;
    pulseCount = 0;  // Reset counter for next period
    float samplingPeriod = SAMPLING_PERIOD_MS / 1000.0;  // Convert to seconds
    freq = (float)currentPulseCount / samplingPeriod;
    
    // Convert frequency to RPM
    measspeed = freq * freqtoRPM;
    
    // PI Controller calculates motor voltage (0-255)
    Vmotor = calculateMotorVoltage(getTargetSpeed(), measspeed, deltaT);

    // Constrain motor voltage to ensure max is 255
    Vmotor = constrain(Vmotor, 0, 255);

    // Dead zone compensation: motor needs minimum PWM of 50 to overcome friction
    // Map PID output (0-255) to actual motor range (0 or 50-255)
    int motorPWM;
    if (Vmotor < 10) {
      // If PID wants very low speed, turn motor completely off
      motorPWM = 0;
    } else {
      // Map PID range [10-255] to motor range [50-255]
      // This ensures motor always gets enough PWM to spin when PID is active
      motorPWM = map(Vmotor, 10, 255, 50, 255);
    }

    // Calculate duty cycle percentage (0-100%) based on actual motor PWM
    float dutyCycle = (motorPWM / 255.0) * 100.0;

    // NORMAL PWM: 0 = OFF, 255 = FULL SPEED
    // Output compensated PWM to motor
    analogWrite(STIRRING_MOTOR_PIN, motorPWM);

    // Optional: Check controller health
    if (!checkControllerHealth(measspeed, Vmotor)) {
      // Potential sensor failure or motor stall detected
      Serial.println("WARNING: Controller health check failed - possible sensor failure or motor stall");
    }

    // Calculate error for transmission
    float error = getTargetSpeed() - measspeed;

    // Send data via UART to ESP32 TTGO display board
    // Format: TARGET,MEASURED,ERROR,PWM\n
    Serial2.print(getTargetSpeed(), 1);
    Serial2.print(",");
    Serial2.print(measspeed, 1);
    Serial2.print(",");
    Serial2.print(error, 1);
    Serial2.print(",");
    Serial2.print(Vmotor);
    Serial2.print("\n");

    // Print data for Serial Plotter (comma-separated values)
    // Format: Target,Measured,Error,PWM,Integral
    Serial.print(getTargetSpeed());
    Serial.print(",");
    Serial.print(measspeed);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.print(Vmotor);
    Serial.print(",");
    Serial.println(getIntegralTerm());
  }
}