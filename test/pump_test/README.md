# Pump Test Program

Simple test sketch for controlling two pumps connected to ESP32 Arduino Nano pins D11 and D12.

## Hardware Setup

### Connections
- **Pump 1**: Connected to D11 (GPIO11)
- **Pump 2**: Connected to D12 (GPIO12)

### Wiring
1. Connect pump driver signal pin to GPIO11 (Pump 1) or GPIO12 (Pump 2)
2. Connect pump driver VCC to appropriate power supply (typically 5V or 12V depending on pump)
3. Connect pump driver GND to common ground with ESP32
4. Connect ESP32 GND to power supply GND

**Important**: Make sure your pump driver can handle the voltage/current requirements of your pumps.

## Features

- **Individual pump control** (on/off for each pump)
- **PWM speed control** (0-255, adjustable in increments of 25)
- **Serial command interface** (interactive control via Serial Monitor)
- **Automatic test sequence** (ramps through different speeds)
- **Real-time status display** (shows current pump states every second)

## Serial Commands

| Command | Action |
|---------|--------|
| `1` | Turn ON Pump 1 at current PWM level |
| `2` | Turn ON Pump 2 at current PWM level |
| `q` | Turn OFF Pump 1 |
| `w` | Turn OFF Pump 2 |
| `a` | Turn OFF both pumps (emergency stop) |
| `+` | Increase PWM level by 25 |
| `-` | Decrease PWM level by 25 |
| `t` | Run automatic test sequence |
| `h` | Show help menu |

## Usage

### Basic Operation

1. **Upload** the sketch to your ESP32 Arduino Nano
2. **Open** Serial Monitor (115200 baud)
3. **Set PWM level**: Use `+` and `-` to adjust (default is 128 = 50%)
4. **Turn on pump**: Press `1` for Pump 1 or `2` for Pump 2
5. **Turn off pump**: Press `q` for Pump 1, `w` for Pump 2, or `a` for both

### Example Workflow

```
Press '+' three times → PWM = 203 (80%)
Press '1'             → Pump 1 turns on at 80%
Press '2'             → Pump 2 turns on at 80%
Press 'a'             → Both pumps turn off
```

### Automatic Test Sequence

Press `t` to run an automatic test that:
1. Ramps Pump 1 through speeds: 0%, 20%, 40%, 60%, 80%, 100% (2 seconds each)
2. Ramps Pump 2 through speeds: 0%, 20%, 40%, 60%, 80%, 100% (2 seconds each)
3. Runs both pumps at 50% for 3 seconds
4. Turns off both pumps

## PWM Settings

- **Frequency**: 1 kHz
- **Resolution**: 8-bit (0-255)
- **PWM Channel 0**: Pump 1
- **PWM Channel 1**: Pump 2

## Status Output

The program prints status every second:
```
[STATUS] Pump1: ON (128/255, 50%) | Pump2: OFF | Default PWM: 128/255 (50%)
```

## Safety Notes

⚠️ **IMPORTANT SAFETY CONSIDERATIONS:**

1. **Never** run pumps dry - ensure liquid is available
2. **Always** connect common ground between ESP32 and pump power supply
3. **Check** pump driver current rating matches your pump requirements
4. **Use** appropriate voltage for your pump (check pump specifications)
5. **Test** with low PWM values first to verify correct operation
6. **Press** `a` for emergency stop if needed

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Pumps don't turn on | Check wiring, ensure common ground, verify pump driver voltage |
| Pumps run backwards | Swap pump polarity (+ and -) |
| Erratic behavior | Check for loose connections, verify power supply capacity |
| Pumps always run at full speed | Verify pump driver supports PWM control |
| Serial commands don't work | Check baud rate is 115200, ensure Serial Monitor is connected |

## Technical Details

- **PWM Frequency**: 1000 Hz (suitable for most pump drivers)
- **Duty Cycle Range**: 0% (off) to 100% (full speed)
- **Control Method**: ESP32 LEDC PWM peripheral
- **Update Rate**: Commands processed immediately, status printed every 1 second

## Example Output

```
========================================
     ESP32 Pump Test Program
========================================
Pump 1 Pin: D11 (GPIO11)
Pump 2 Pin: D12 (GPIO12)
PWM Frequency: 1000 Hz
Default PWM Level: 128 / 255 (50%)

--- COMMAND MENU ---
1  : Turn ON Pump 1
2  : Turn ON Pump 2
q  : Turn OFF Pump 1
w  : Turn OFF Pump 2
a  : Turn OFF both pumps
+  : Increase PWM level
-  : Decrease PWM level
t  : Run automatic test sequence
h  : Show this help menu
--------------------

Initialization complete. Both pumps OFF.
========================================

[STATUS] Pump1: OFF | Pump2: OFF | Default PWM: 128/255 (50%)
```

## Integration with Main Bioreactor Code

Once pumps are verified working with this test sketch, you can integrate them into the main bioreactor control system where they will be controlled by the pH PID controller.
