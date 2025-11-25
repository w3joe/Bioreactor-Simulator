# Software Requirements Specification (SRS)
## PID Controller for Bioreactor Stirring Motor Control

**Document Version:** 2.1  
**Target File:** `stirring.h`, `esp32_arduino_nano.ino`, `esp32_ttgo.ino`  
**Language:** C++ (Arduino)  
**Date:** November 9, 2025  
**Updated:** 
- v2.0: Added Derivative (D) term to implement full PID controller
- v2.1: Added UART communication between ESP32 boards for data transmission

---

## 1. OVERVIEW

### 1.1 Purpose
Implement a Proportional-Integral-Derivative (PID) controller in `stirring.h` that regulates DC motor speed for a bioreactor stirrer using hall sensor feedback. The header file will be included in a main `.cpp` file.

### 1.2 Scope
The `stirring.h` file shall provide:
- PID controller algorithm
- Speed setpoint management
- Motor voltage calculation with anti-windup
- Configuration parameters
- Helper functions for controller tuning and monitoring

---

## 2. SYSTEM CONTEXT

### 2.1 Hardware Specifications
- **Motor:** DC motor with 10:1 gearing, max 1000 RPM at output shaft
- **Hall Sensor:** 70 pulses per revolution (1167 pulses/s at 1000 RPM)
- **PWM Control:** 10-bit (0-1023), mapped to 0-5V motor voltage
- **Update Rate:** 10 ms (100 Hz control loop)
- **Microcontroller:** ESP32 (Arduino Nano form factor for control, TTGO for display)
- **Communication:** UART (Serial2) between ESP32 boards at 115200 baud

### 2.2 Existing Code Integration
The main `.cpp` file already implements:
- Hall sensor interrupt routine (`freqcount()`)
- Speed measurement (`measspeed` variable in RPM)
- 10 ms timing loop
- PWM output to motor via `Vmotor` variable (0-1023)
- UART communication to ESP32 TTGO display board

### 2.3 Key Variables from Main Code
```cpp
// Available from main .cpp:
float measspeed;        // Current measured speed in RPM
int Vmotor;            // Motor voltage command (0-1023)
float deltaT;          // Time since last update (seconds)
```

---

## 3. FUNCTIONAL REQUIREMENTS

### 3.1 Controller Structure

**FR-1: PID Controller Algorithm**
- Implement discrete-time PID controller with the following equation:
  ```
  u(k) = Kp * e(k) + Ki * Ts * sum(e(0)...e(k)) + Kd * (e(k) - e(k-1)) / Ts
  ```
  Where:
  - `u(k)` = controller output (motor voltage)
  - `e(k)` = error = setpoint - measured_speed
  - `e(k-1)` = previous error
  - `Kp` = proportional gain
  - `Ki` = integral gain  
  - `Kd` = derivative gain
  - `Ts` = sampling time (0.01 seconds)

**FR-2: Function Interface**
- Primary function shall be: `int calculateMotorVoltage(float setpoint, float measspeed, float deltaT)`
- Function shall return motor voltage value (0-1023)
- Function shall be callable from the main loop every 10 ms

**FR-3: Setpoint Management**
- Shall accept setpoint in RPM (typical range: 300-1000 RPM)
- Shall validate setpoint is within safe limits (0-1000 RPM)
- Shall provide function to update setpoint: `void setTargetSpeed(float rpm)`

### 3.2 Anti-Windup Protection

**FR-4: Integral Windup Prevention**
- Shall implement back-calculation anti-windup method
- When motor voltage saturates (reaches 0 or 1023), integral term shall stop accumulating
- Shall provide configurable anti-windup gain parameter

**FR-5: Saturation Limits**
- Motor voltage output shall be constrained to [0, 1023]
- Shall prevent integral term from growing when output is saturated

### 3.3 Controller Tuning Parameters

**FR-6: Configurable Gains**
- Proportional gain (Kp): default = 2.0, range [0.1 - 10.0]
- Integral gain (Ki): default = 0.5, range [0.0 - 5.0]
- Derivative gain (Kd): default = 0.1, range [0.0 - 2.0]
- Gains shall be modifiable via setter functions or public variables

**FR-7: Initial Recommended Tuning**
Provide conservative starting values:
- `Kp = 2.0` (2% motor voltage change per RPM error)
- `Ki = 0.5` (moderate integral action)
- `Kd = 0.1` (light derivative action for damping)
- Anti-windup gain = 0.5

### 3.4 Initialization and Reset

**FR-8: Controller Initialization**
- Shall provide `void initController()` function
- Shall reset integral accumulator to zero
- Shall initialize previous error to zero (required for derivative calculation)
- Should be called in Arduino `setup()` or at program start

**FR-9: Integral and Derivative Reset**
- Shall provide `void resetIntegral()` function
- Shall allow manual reset of integral term and previous error (e.g., when changing setpoint drastically)
- Resetting previous error prevents derivative kick on setpoint changes

### 3.5 Safety Features

**FR-10: Speed Measurement Validation**
- If `measspeed == 0` and motor voltage > 100, assume sensor failure or stall
- Shall provide status flag or return code indicating controller health

**FR-11: Startup Behavior**
- When starting from zero speed, controller shall ramp up smoothly
- Consider providing optional setpoint ramping function

### 3.6 UART Communication

**FR-12: Data Transmission**
- Shall transmit stirring motor data via UART to ESP32 TTGO display board
- Data packet shall include:
  - Target RPM (float)
  - Measured RPM (float)
  - Error (float, calculated as target - measured)
  - Motor PWM value (int, 0-1023)
- Transmission rate: Every 10 ms (100 Hz) to match control loop
- Baud rate: 115200
- Protocol: Simple text-based format with delimiter (comma-separated or structured)

**FR-13: Data Format**
- Data shall be formatted as: `TARGET,MEASURED,ERROR,PWM\n`
- Example: `500.0,498.5,1.5,512\n`
- Each value separated by comma, terminated with newline
- Receiver shall parse and validate received data

**FR-14: Error Handling**
- Shall handle UART transmission errors gracefully
- Shall not block control loop if UART transmission fails
- Receiver shall handle incomplete or corrupted data packets

---

## 4. NON-FUNCTIONAL REQUIREMENTS

### 4.1 Performance

**NFR-1: Execution Time**
- Controller calculation shall complete in < 500 microseconds per call
- No use of `delay()` function
- Minimize floating-point operations where possible

**NFR-2: Steady-State Performance**
- Steady-state error shall be < 1% of setpoint (for speeds > 300 RPM)
- Settling time shall be < 5 seconds for 10% setpoint change

### 4.2 Code Quality

**NFR-3: Header File Structure**
```cpp
#ifndef STIRRING_H
#define STIRRING_H

// Class or namespace containing all controller functionality
// OR: Collection of functions with static variables

#endif
```

**NFR-4: Documentation**
- All public functions shall have brief comments explaining purpose and parameters
- Key variables shall have inline comments
- Include usage example in header comment block

**NFR-5: Memory Usage**
- Minimize global variable usage
- Consider using a class or struct to encapsulate controller state
- Total RAM usage for controller state shall be < 100 bytes

### 4.3 Maintainability

**NFR-6: Code Style**
- Use descriptive variable names
- Follow Arduino/C++ naming conventions
- Use const for parameters that don't change

**NFR-7: Modularity**
- Controller logic shall be independent of sensor/actuator interface
- Gains and parameters shall be easily modifiable at compile time or runtime

---

## 5. INTERFACE SPECIFICATION

### 5.1 Required Functions

```cpp
// Initialize controller (call once in setup)
void initController();

// Set target speed in RPM
void setTargetSpeed(float targetRPM);

// Calculate motor voltage based on current speed
// Call this every loop iteration (10 ms)
int calculateMotorVoltage(float setpoint, float measspeed, float deltaT);

// Optional: Reset integral term
void resetIntegral();

// Optional: Manually set gains
void setGains(float Kp_new, float Ki_new, float Kd_new = -1.0);

// Optional: Get current controller state for debugging
float getIntegralTerm();
float getProportionalTerm(float error);
float getDerivativeTerm(float error, float deltaT);
```

### 5.2 Usage Example in Main Code

```cpp
#include "stirring.h"

void setup() {
  // ... existing setup code ...
  initController();
  setTargetSpeed(500.0); // Set target to 500 RPM
}

void loop() {
  currtime = micros();
  deltaT = (currtime-prevtime)*1e-6;
  
  if (currtime-T1>0) {
    prevtime = currtime;
    T1 = T1+10000; // 10 ms update
    
    measspeed = freq*freqtoRPM;
    
    // PID Controller calculates motor voltage
    Vmotor = calculateMotorVoltage(getTargetSpeed(), measspeed, deltaT);
    
    // Existing code handles PWM output
    Vmotor = constrain(Vmotor,0,1023);
    if (Vmotor==255) {Vmotor=256;}
    analogWrite(motorpin,Vmotor);
    
    // ... rest of existing code ...
  }
}
```

---

## 6. DESIGN CONSTRAINTS

### 6.1 Platform Constraints
- Must compile for Arduino Uno/Nano (ATmega328P)
- Header-only implementation (no separate .cpp file)
- Compatible with Arduino IDE

### 6.2 Timing Constraints
- Designed for 10 ms sampling period (100 Hz)
- Must work with existing interrupt-driven speed measurement

### 6.3 Numerical Constraints
- Use `float` for calculations (single precision sufficient)
- Be aware of integer overflow in timing calculations (handled by main code)

---

## 7. TESTING REQUIREMENTS

### 7.1 Functional Testing
- Test setpoint tracking from 300-1000 RPM
- Test step response (e.g., 500→700 RPM)
- Test disturbance rejection (manual load applied to stirrer)

### 7.2 Edge Cases
- Starting from zero speed
- Very low setpoints (< 100 RPM where sensor is less accurate)
- Setpoint changes during operation
- Integral windup scenarios (blocked impeller)

### 7.3 Tuning Validation
- Document recommended Kp, Ki values after testing
- Verify no sustained oscillations
- Check for overshoot < 10% on step response

---

## 8. IMPLEMENTATION NOTES

### 8.1 Recommended Approach

**Option A: Simple function-based approach**
```cpp
// Use static variables inside calculateMotorVoltage() 
// to maintain controller state between calls
```

**Option B: Class-based approach**
```cpp
class PIDController {
private:
  float integral;
  float previousError;
  float Kp, Ki, Kd;
  float targetSpeed;
public:
  // Methods...
};
```

### 8.2 Key Implementation Details

1. **Integral Term Calculation:**
   ```cpp
   integral += error * deltaT;
   // Then apply anti-windup after saturation check
   ```

2. **Anti-Windup:**
   ```cpp
   float raw_output = Kp*error + Ki*integral;
   int saturated_output = constrain(raw_output, 0, 1023);
   
   if (raw_output != saturated_output) {
     // Back-calculate to prevent windup
     integral -= (raw_output - saturated_output) * (deltaT / Ki) * antiWindupGain;
   }
   ```

3. **Derivative Term Calculation:**
   ```cpp
   float errorDerivative = (error - previousError) / deltaT;
   float d_term = Kd * errorDerivative;
   // Update previousError after calculation
   previousError = error;
   ```

4. **First Call Handling:**
   - On first call, previousError should be initialized to zero
   - This prevents derivative kick on startup
   - Derivative term uses error rate of change, not setpoint changes (avoids derivative kick)

### 8.3 Debugging Support

Consider adding optional debug output:
```cpp
void printControllerState() {
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" P: "); Serial.print(p_term);
  Serial.print(" I: "); Serial.print(i_term);
  Serial.print(" D: "); Serial.println(d_term);
}
```

---

## 9. ACCEPTANCE CRITERIA

The implementation shall be considered complete when:

1. ✅ Code compiles without errors on Arduino IDE
2. ✅ Controller maintains setpoint within ±2% for constant loads
3. ✅ No sustained oscillations at any setpoint (300-1000 RPM)
4. ✅ Settling time < 5 seconds for 10% step changes
5. ✅ Integral windup protection prevents excessive overshoot
6. ✅ Code is documented with usage comments
7. ✅ All required functions are implemented

---

## 10. FUTURE ENHANCEMENTS (Optional)

- Auto-tuning algorithm (e.g., Ziegler-Nichols)
- Setpoint ramping/trajectory generation
- Adaptive gains based on operating speed
- Feed-forward compensation for known disturbances
- Derivative filtering to reduce noise sensitivity
- Setpoint weighting for derivative term (derivative on measurement)

---

## APPENDIX A: Reference Equations

### Discrete PID Controller
```
e(k) = setpoint - measured_speed
P(k) = Kp * e(k)
I(k) = I(k-1) + Ki * Ts * e(k)
D(k) = Kd * (e(k) - e(k-1)) / Ts
u(k) = P(k) + I(k) + D(k)
output = constrain(u(k), 0, 1023)
```

### Suggested Starting Parameters
- Sampling time (Ts): 0.01 s (10 ms)
- Kp: 2.0 (provides 2 PWM units per 1 RPM error)
- Ki: 0.5 (integral time constant ~4 seconds)
- Kd: 0.1 (derivative action for damping oscillations)
- Anti-windup gain: 0.5

---

## APPENDIX B: Quick Reference

**Key Points for LLM Implementation:**
1. Create header file with include guards
2. Implement PID algorithm with anti-windup
3. Use static variables or class to maintain state (including previousError for derivative)
4. Accept measspeed, setpoint, deltaT as inputs
5. Return motor voltage (0-1023)
6. Provide clear initialization function (reset integral and previousError)
7. Add helpful comments and usage example
8. Keep code efficient (< 500 µs execution time)
9. Calculate derivative term using (error - previousError) / deltaT

**Files to create:**
- `stirring.h` only (header-only implementation)

**Integration:**
- Include in main .cpp with `#include "stirring.h"`
- Call `initController()` in `setup()`
- Call `calculateMotorVoltage()` in 10ms loop
- Assign result to `Vmotor` variable

---

**END OF SPECIFICATION**