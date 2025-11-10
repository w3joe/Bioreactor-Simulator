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
#define STIRRING_SENSOR_PIN 36
#define TEMP_SENSOR_PIN 34
#define PH_SENSOR_PIN 35

#define STIRRING_MOTOR_PIN 33
#define HEATING_ELEMENT_PIN 26
#define BASE_PUMP_PIN 27
#define ACID_PUMP_PIN 14

// UART Communication pins
// Connect ESP32_Arduino_Nano GPIO4 (TX) to ESP32_TTGO GPIO16 (RX)
// Connect GND of both boards together
#define UART_TX_PIN 13  // TX pin for Serial2
#define UART_RX_PIN 12  // RX pin for Serial2 (not used for sending, but needed for Serial2 initialization)
#define UART_BAUD 115200

//interrupt routine
void freqcount() {
  pulseCount++;
}

void setup()
{
  Serial.begin(115200);
  
  // Initialize UART for communication with ESP32 TTGO
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Initialize stirring motor control
  pinMode(STIRRING_MOTOR_PIN, OUTPUT);
  pinMode(STIRRING_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STIRRING_SENSOR_PIN), freqcount, RISING);
  
  // Calculate conversion factor: RPM = (freq * 60) / PULSES_PER_REV
  freqtoRPM = 60.0 / PULSES_PER_REV;
  
  // Initialize PID controller
  initController();
  setTargetSpeed(500.0);  // Set initial target speed to 500 RPM
  
  // Initialize timing variables
  currtime = micros();
  prevtime = currtime;
  nextControlTime = currtime;
  
  Serial.println("Bioreactor stirring motor controller initialized");
  Serial.print("Target speed: ");
  Serial.print(getTargetSpeed());
  Serial.println(" RPM");
  Serial.println("UART communication initialized");
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
    
    // PI Controller calculates motor voltage
    Vmotor = calculateMotorVoltage(getTargetSpeed(), measspeed, deltaT);
    
    // Constrain and apply motor voltage
    Vmotor = constrain(Vmotor, 0, 1023);
    if (Vmotor == 255) { Vmotor = 256; }  // Avoid specific PWM value if needed
    analogWrite(STIRRING_MOTOR_PIN, Vmotor);
    
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
    
    // Print RPM and control information to Serial (for debugging)
    Serial.print("Target: ");
    Serial.print(getTargetSpeed());
    Serial.print(" RPM | Measured: ");
    Serial.print(measspeed);
    Serial.print(" RPM | Error: ");
    Serial.print(error);
    Serial.print(" RPM | Motor PWM: ");
    Serial.println(Vmotor);
  }
}
