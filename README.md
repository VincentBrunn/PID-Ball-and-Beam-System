# PID-Ball-and-Beam-System

A reproducible ball-and-beam control system built around an Arduino Uno. Position feedback is measured with an HC-SR04 ultrasonic sensor, and a NEMA-17 stepper motor driven by a TMC2209 (CNC Shield) tilts the beam. A discrete PID loop closes the loop on ball position with optional homing via an AS5600 magnetic encoder.

## Features
- PID position control with manual and automatic setpoint modes
- Stepper-driven actuation (TMC2209 on CNC Shield; STEP/DIR)
- Ultrasonic position sensing with filtering and out-of-range handling
- Optional homing using AS5600 encoder (or limit switch)
- Serial plotting/logging for tuning and diagnostics
- Configurable pin map, geometry, and PID gains

## System Overview
**Sensor → Controller → Actuator**
1. Measure ball distance along the beam with HC-SR04 (filtered).
2. Compute error = setpoint − position; update PID.
3. Convert controller output to beam angle command.
4. Drive stepper (STEP/DIR) to reach the target angle.
5. Repeat at a fixed control period.

## Bill of Materials
- Arduino Uno (or compatible 5V board)
- CNC Shield for Arduino
- TMC2209 stepper driver + heatsink
- NEMA-17 stepper motor (e.g., 1.2–2.0 A rated)
- HC-SR04 ultrasonic sensor
- Potentiometer (10 kΩ) for manual setpoint
- Optional: AS5600 magnetic encoder (I2C) for homing/angle reference
- Optional: Limit switch for hard homing
- 12 V power supply (≥2 A recommended)
- Beam, bearings/hinge, ball (steel; e.g., 16–20 mm), wiring, hardware

## Pinout (example)
Adjust in `config.h` as needed.
| Function            | Pin        |
|---------------------|------------|
| STEP                | D2         |
| DIR                 | D3         |
| ENABLE (optional)   | D8         |
| HC-SR04 TRIG        | D6         |
| HC-SR04 ECHO        | D7         |
| Potentiometer       | A0         |
| AS5600 SDA/SCL      | A4 / A5    |
| Limit Switch        | D9         |
| Serial Monitor      | 115200 bps |

## Libraries
Install via Arduino Library Manager or from source:
- **TMCStepper** (for TMC2209 configuration over UART/STEP-DIR)
- **AccelStepper** (optional; for stepper motion profiling)
- **NewPing** (HC-SR04 handling) or equivalent
- **Wire** (AS5600 over I2C) and a simple AS5600 helper (optional)

## Geometry and Units
Define the beam length and sensor mounting so the raw sensor reading maps to a linear position `x` along the beam (millimeters). A two-point calibration is recommended:
- Place the ball at two known positions and record sensor readings.
- Solve a linear fit `x = a * reading + b` and store `a, b` in `config.h`.

## Build and Run
1. Clone the repository and open the sketch in Arduino IDE.
2. Install the libraries above.
3. Update `config.h`:
   - Pin map
   - Beam geometry and calibration coefficients
   - PID gains `Kp`, `Ki`, `Kd`
   - Control period (e.g., 5–10 ms)
4. Upload to the Arduino Uno.
5. Open Serial Monitor (115200 bps) to view telemetry and adjust parameters.

## PID Loop (discrete)
At fixed period `Ts`:
