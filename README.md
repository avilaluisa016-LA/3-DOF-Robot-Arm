# 3 DOF Robotic Arm – Joystick Controlled
A microcontroller-based robotic arm with three degrees of freedom (DOF), controlled in real time using a joystick. Built using the Adafruit Feather M0 and the Adafruit 8 Channel Servo Controller, this project was developed as a hands-on learning exercise in robotic arm design, embedded systems, and the fundamentals of inverse kinematics.
---

## Overview
This project simulates the basic motion control of a robotic arm commonly found in industrial and aerospace applications. A joystick serves as the input device, translating analog signals into precise servo motor movements across three independent axes of rotation. Basic inverse kinematics principles were applied to calculate and set servo angles based on desired arm positions, providing a practical introduction to the mathematics behind robotic motion control.

The project was built with three goals in mind:
- Understand how to design and build a functional robotic arm
- Learn how to program and interface with the Adafruit Feather M0 microcontroller
- Gain hands-on experience with the Adafruit 8 Channel Servo Controller
---

## Features
- Real-time joystick control of three servo motors
- Three independent axes of movement (3 DOF)
- Servo angle calculation using basic inverse kinematics
- Smooth and responsive motion using PWM signals
- Compact and low-cost hardware setup
---

## Hardware Components
| Component | Description |
|---|---|
| Adafruit Feather M0 | Main microcontroller (SAMD21 based) |
| Adafruit 8 Channel Servo Controller | Multi-channel PWM servo driver |
| Joystick Module | Analog input device (X, Y axes + button) |
| Servo Motors (x3) | Actuators for each degree of freedom |
| Power Supply | External power for servo motors |
| Jumper Wires | Electrical connections |
| Breadboard | Prototyping base |
---

## How It Works
1. The joystick outputs analog voltage signals corresponding to its X and Y axis position
2. The Adafruit Feather M0 reads these signals via its analog input pins
3. Basic inverse kinematics calculations are applied to determine the required servo angles
4. The computed angles are sent to the Adafruit 8 Channel Servo Controller via I2C
5. Each servo motor receives a PWM signal and rotates to the corresponding position
6. The result is smooth, real-time control of the robotic arm's three joints
---

## Wiring Diagram

*Wiring diagram coming soon — see code comments for pin assignments*

---

## Code
The main sketch is located in the `/src` folder. Key libraries used:
- `Adafruit_PWMServoDriver.h` — Adafruit 8 channel servo controller library
- `Wire.h` — I2C communication library
---

## Concepts Learned
- Robotic arm design principles and joint configuration
- Inverse kinematics fundamentals for servo angle calculation
- Adafruit Feather M0 programming and hardware setup
- Multi-channel servo control using the Adafruit 8 channel servo driver
- I2C communication between microcontroller and servo driver
---

## Skills Demonstrated
- Embedded systems programming (C/C++)
- Basic inverse kinematics and motion control math
- PWM motor control
- Analog signal reading and mapping
- Hardware integration and wiring
- Mechanical and electronic system design
---

## Future Improvements
- Add a fourth DOF for a gripper/end effector
- Implement position feedback using encoders
- Expand inverse kinematics model for full 3D positional control
- Develop a preset position memory function
- Implement advanced motion control for smoother and more precise arm movement
- Replace the Adafruit Feather M0 ATWINC with the Adafruit Feather M0 LoRa for wireless communication capabilities
- Design and 3D print a custom arm structure in Fusion 360
---
