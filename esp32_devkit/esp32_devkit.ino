#include <Arduino.h>
#include "stirring.h"  // PI Controller for stirring motor

// Stirring motor control variables
unsigned long currtime = 0;        
unsigned long prevtime = 0;       
unsigned long nextControlTime = 0; 
float measspeed = 0.0;            
int Vmotor = 0;                   
float deltaT = 0.01;               

// Hall sensor variables - CORRECTED
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseT[8];  // Store last 8 pulse times for averaging
const float PULSES_PER_REV = 70.0;
const float SAMPLING_PERIOD_MS = 10.0;
unsigned long lastRPMCalc = 0;
const long RPM_CALC_INTERVAL = 100;  // Calculate RPM every 100ms

// Pin definitions
#define STIRRING_SENSOR_PIN 2
#define TEMP_SENSOR_PIN 34
#define PH_SENSOR_PIN 35
#define STIRRING_MOTOR_PIN 10
#define HEATING_ELEMENT_PIN 26
#define BASE_PUMP_PIN 27
#define ACID_PUMP_PIN 14
#define UART_TX_PIN 1
#define UART_RX_PIN 0
#define UART_BAUD 115200

// Interrupt routine - CORRECTED
void IRAM_ATTR freqcount() {
  unsigned long currentTime = micros();
  
  // Debounce - ignore pulses closer than 500us
  if (currentTime - lastPulseTime > 500) {
    pulseCount++;
    lastPulseTime = currentTime;
    
    // Store pulse times for period-based calculation
    for (int i = 7; i > 0; i--) {
      pulseT[i] = pulseT[i-1];
    }
    pulseT[0] = currentTime;
  }
}

// Calculate RPM - METHOD 1: Pulse counting over time interval
void calculateRPM_Method1() {
  // Disable interrupts while reading
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;  // Reset counter
  interrupts();
  
  // Calculate RPM from pulse count
  // RPM = (pulses / PULSES_PER_REV) * (60000ms / interval_ms)
  measspeed = (pulses / PULSES_PER_REV) * (60000.0 / RPM_CALC_INTERVAL);
  
  // If no pulses, RPM is 0
  if (pulses == 0) {
    measspeed = 0;
  }
}

// Calculate RPM - METHOD 2: Period measurement (more accurate at low speeds)
void calculateRPM_Method2() {
  noInterrupts();
  unsigned long currentTime = micros();
  unsigned long period = pulseT[0] - pulseT[7];  // Time for 7 pulses
  interrupts();
  
  // If we have valid period data
  if (period > 0 && (currentTime - pulseT[0]) < 200000) {  // Within 200ms
    // Frequency = 7 pulses / (period in seconds)
    float freq = 7.0e6 / period;  // 1e6 to convert microseconds to seconds
    
    // RPM = (freq / PULSES_PER_REV) * 60
    measspeed = (freq / PULSES_PER_REV) * 60.0;
  } else {
    // No recent pulses, motor stopped
    measspeed = 0;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize UART for communication with ESP32 TTGO
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Initialize stirring motor control
  pinMode(STIRRING_MOTOR_PIN, OUTPUT);
  pinMode(STIRRING_SENSOR_PIN, INPUT_PULLUP);
  
  // Setup PWM for ESP32
  ledcAttach(STIRRING_MOTOR_PIN, 20000, 10);  // 20kHz, 10-bit
  
  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(STIRRING_SENSOR_PIN), freqcount, RISING);
  
  // Initialize PID controller
  initController();
  setTargetSpeed(500.0);
  
  // Initialize timing variables
  currtime = micros();
  prevtime = currtime;
  nextControlTime = currtime;
  lastRPMCalc = millis();
  
  Serial.println("Bioreactor stirring motor controller initialized");
  Serial.print("Target speed: ");
  Serial.print(getTargetSpeed());
  Serial.println(" RPM");
  Serial.println("UART communication initialized");
}

void loop() {
  currtime = micros();
  deltaT = (currtime - prevtime) * 1e-6;
  
  // Calculate RPM every 100ms (METHOD 1 - good for general use)
  if (millis() - lastRPMCalc >= RPM_CALC_INTERVAL) {
    calculateRPM_Method1();
    // OR use: calculateRPM_Method2();  // Better at low speeds
    lastRPMCalc = millis();
  }
  
  // Control loop at 10ms rate
  if (currtime >= nextControlTime) {
    prevtime = currtime;
    nextControlTime = nextControlTime + 10000;
    
    // PI Controller calculates motor voltage
    Vmotor = calculateMotorVoltage(getTargetSpeed(), measspeed, deltaT);
    
    // Constrain motor voltage
    Vmotor = constrain(Vmotor, 0, 1023);
    if (Vmotor == 255) { Vmotor = 256; }
    
    // INVERTED PWM: 0 = highest speed, 1023 = lowest speed
    int invertedPWM = 1023 - Vmotor;
    
    // Apply inverted PWM using ESP32 method
    ledcWrite(STIRRING_MOTOR_PIN, invertedPWM);
    
    // Check controller health
    if (!checkControllerHealth(measspeed, Vmotor)) {
      Serial.println("WARNING: Controller health check failed");
    }
    
    // Calculate error
    float error = getTargetSpeed() - measspeed;
    
    // Send data via UART
    Serial2.print(getTargetSpeed(), 1);
    Serial2.print(",");
    Serial2.print(measspeed, 1);
    Serial2.print(",");
    Serial2.print(error, 1);
    Serial2.print(",");
    Serial2.print(Vmotor);  // Send logical PWM value (not inverted)
    Serial2.print("\n");
    
    // Debug output
    Serial.print("Target: ");
    Serial.print(getTargetSpeed());
    Serial.print(" RPM | Measured: ");
    Serial.print(measspeed, 1);
    Serial.print(" RPM | Error: ");
    Serial.print(error, 1);
    Serial.print(" RPM | PWM: ");
    Serial.print(Vmotor);
    Serial.print(" (Inverted: ");
    Serial.print(invertedPWM);
    Serial.println(")");
  }
}

/* ============================================
   COMPARISON OF METHODS:
   ============================================
   
   METHOD 1 (Pulse Counting):
   - Counts pulses over fixed time interval (100ms)
   - Good for: Medium to high speeds
   - Pros: Simple, predictable update rate
   - Cons: Poor resolution at low speeds
   
   METHOD 2 (Period Measurement):
   - Measures time between pulses
   - Good for: Low speeds, high accuracy
   - Pros: Accurate at all speeds, fast response
   - Cons: Needs recent pulses, can be noisy
   
   For stirring motor at 750 RPM, METHOD 1 is recommended.
   
   ============================================
   IMPORTANT: INVERTED PWM CONTROL
   ============================================
   This motor driver has INVERTED PWM logic:
   - PWM 0    = HIGHEST speed (motor runs fastest)
   - PWM 1023 = LOWEST speed (motor runs slowest)
   
   The PI controller outputs normal logic (higher value = faster)
   So we INVERT it before sending to motor:
   invertedPWM = 1023 - Vmotor
   
   This is common with P-channel MOSFETs or certain motor drivers.
   ============================================ */