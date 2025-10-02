**Title:** PID-Controlled Ball-and-Beam System

**Blurb:**  
Designed and built a ball-and-beam control system using an Arduino Uno, HC-SR04 ultrasonic sensor, and a NEMA-17 stepper motor driven by a TMC2209 on a CNC Shield. A discrete PID controller regulates ball position by adjusting beam angle; features include optional homing with an AS5600 magnetic encoder, manual setpoint via potentiometer, and serial data logging for tuning. Demonstrated skills in control systems, embedded programming, sensor integration, and rapid prototyping.

---

# README.md — Ball-and-Beam Control (Arduino + PID)

A reproducible ball-and-beam system with position feedback from an HC-SR04 ultrasonic sensor and actuation via a NEMA-17 stepper (TMC2209 on a CNC Shield). A discrete PID loop closes the loop on ball position. Optional homing/reference: AS5600 magnetic encoder or a limit switch.

## Features
- PID position control (manual and automatic setpoint modes)
- Stepper actuation via STEP/DIR (TMC2209; current set by driver)
- Ultrasonic position sensing with filtering and out-of-range handling
- Optional homing using AS5600 (I2C) or a mechanical limit switch
- Serial plotting/logging to aid tuning
- Configurable control period, geometry, and pin map

## System Overview
Sensor → Controller → Actuator  
1) Measure ball position along the beam with HC-SR04 (filtered).  
2) Compute error = setpoint − position; update PID at fixed period `Ts`.  
3) Map controller output to a target beam angle.  
4) Command the stepper to reach the target angle.  

## Bill of Materials
- Arduino Uno (5 V, USB)
- CNC Shield for Arduino + TMC2209 stepper driver (with heatsink)
- NEMA-17 stepper motor (≈ 1.2–2.0 A rated)
- HC-SR04 ultrasonic sensor
- Potentiometer (10 kΩ) for manual setpoint
- Optional: AS5600 magnetic encoder (I2C) or a normally-closed limit switch
- 12 V supply (≥ 2 A recommended)
- Beam assembly, bearings/hinge, steel ball (e.g., 16–20 mm), wiring, hardware

## Pinout (example, editable in `config.h`)
| Function          | Pin    |
|-------------------|--------|
| STEP              | D2     |
| DIR               | D3     |
| ENABLE (optional) | D8     |
| HC-SR04 TRIG      | D6     |
| HC-SR04 ECHO      | D7     |
| Potentiometer     | A0     |
| AS5600 SDA / SCL  | A4 / A5|
| Limit Switch      | D9     |
| Serial            | 115200 |

## Libraries
Install via Arduino Library Manager or from source:
- TMCStepper (TMC2209 config if using UART features)
- AccelStepper (optional; motion profiling)
- NewPing (or equivalent HC-SR04 helper)
- Wire (AS5600 I2C) and a simple AS5600 helper (optional)

## Geometry and Calibration
Mount the sensor to view the beam along its length. Calibrate a linear map from raw sensor readings to position `x` (mm).
1) Place the ball at two known positions; record sensor readings.  
2) Fit `x = a * reading + b`; store `a, b` in `config.h`.  
Clamp `x` to the physical beam limits.

## Build and Run
1) Clone the repo and open `firmware/ball_and_beam.ino` in Arduino IDE.  
2) Install the libraries above.  
3) Edit `config.h`: pins, calibration (`a, b`), PID gains (`Kp, Ki, Kd`), and control period `Ts`.  
4) Upload to the Arduino Uno.  
5) Open Serial Monitor at 115200 bps to view telemetry and adjust parameters.

## Control Loop (discrete PID)
`u` → target beam angle → step commands. Apply output limits and integral anti-windup.

## Tuning Procedure
1) Start with `Ki = 0`, `Kd = 0`. Increase `Kp` until fast response without sustained oscillation.  
2) Add `Kd` to reduce overshoot and ringing.  
3) Introduce small `Ki` to remove steady-state error; keep integral clamped.  
4) Validate across multiple setpoints; retune if sensor noise or missed steps appear.

## Sensor Filtering
Use a moving average or median filter. Reject invalid echoes and clamp to the beam limits. Ensure the sensor angle avoids specular reflections off the ball.

## Homing and Limits
- AS5600: establish a repeatable zero beam angle at startup.  
- Limit switch: slow approach to set mechanical zero.  
Always enforce software angle limits and add mechanical end-stops.

## Safety
Observe driver current limits and cooling. Use conservative accelerations to avoid missed steps. Add end-stops to protect the mechanism.

## Troubleshooting
- Oscillation: reduce `Kp` or increase `Kd`; verify control period timing.  
- Slow response: increase `Kp`, reduce friction, or increase allowable step rate.  
- Noisy position: improve filtering or calibration; check sensor mounting.  
- Missed steps: lower acceleration/velocity; confirm supply and driver current.

## Roadmap
- Feedforward term from simple dynamics model  
- Fusion of encoder angle with ultrasonic position  
- Serial bridge to ROS and real-time plotting  
- Automated gain sweeps for tuning

## License
MIT

