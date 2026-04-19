# NEXUS

Next-gen Expendable Unmanned System for exploratory missions.

NEXUS is a modular Mars rover cobot platform built for unknown and hazardous environments. It combines Arduino firmware, a real-time Python mission dashboard, multi-sensor telemetry, and safety-first drive control.

## Pitch

NEXUS is an expendable exploration cobot designed for rapid iteration. It delivers real-time telemetry, low-delay control, and layered safety systems so teams can explore, test, and evolve quickly in uncertain terrain.

## What This Project Does

- Streams live rover telemetry to a desktop dashboard
- Supports manual drive control from UI buttons and USB gamepad
- Uses multi-sensor inputs for environmental awareness
- Enforces motor safety with watchdog timeout and hazard overrides
- Supports camera and CV-assisted mission awareness on the dashboard

## Core Features

- Real-time telemetry packets from firmware to dashboard
- Live mission state model: BOOT, CAL, SCAN, HAZ, SAMP, DARK
- Motor command protocol with framed packets for robustness
- 900 ms motor watchdog auto-stop when commands stop arriving
- Distance hazard override to prevent unsafe forward/turn movement
- Motion hazard events from dashboard CV to firmware
- Onboard optional LCD status panel and optional barometer feed

## Repository Structure

- mars_rover.ino: Arduino firmware for sensors, drive logic, safety logic, telemetry output
- mars_dashboard.py: Desktop mission control dashboard, control UI, serial parser, camera and CV overlays

## Technology Stack

- Languages: Python, Arduino C/C++
- Desktop UI: Tkinter
- Serial communications: pyserial
- Camera and CV: OpenCV
- Gamepad input: pygame
- Embedded platform: Arduino-compatible board
- Motor drivers: L298N-compatible dual H-bridge setup
- Sensors: ultrasonic distance, thermistor temperature, LDR light, optional moisture, optional BMP280 barometer

## Hardware Overview

### Motor Driver Pin Mapping

Front motor driver:

- ENA: 3
- IN1: 2
- IN2: 4
- ENB: 5
- IN3: 7
- IN4: 8

Back motor driver:

- ENA: 6
- IN1: 10
- IN2: 11
- ENB: 9
- IN3: 12
- IN4: 13

### Sensor Pins

- LDR light: A0
- Thermistor: A2
- Ultrasonic TRIG: A1
- Ultrasonic ECHO: A3
- Moisture sensor: optional, currently disabled in firmware
- LCD and barometer share I2C when enabled

## Firmware Safety Architecture

NEXUS uses layered motor safety:

1. Framed command validation
2. Command watchdog
3. Distance hazard override
4. Effective command clamping in the control loop

### Drive Command Protocol

Dashboard to firmware packet format:

!<cmd><speed>\n
- cmd: F, B, L, R, S
- speed: 0-9 (0 to 90 percent), q (100 percent)

Examples:

- !F8
- !S0
- !Rq

Notes:

- Firmware ignores loose random bytes to reduce accidental movement from serial noise
- Firmware emits MARS_CMD acknowledgements for valid framed commands

### Watchdog

- CMD_WATCHDOG_MS is 900 ms
- If no valid motor command arrives in time, firmware forces STOP

### Hazard Override

- If measured distance is below hazard threshold, non-reverse commands are blocked
- Reverse is allowed so rover can back out of danger

## Telemetry Protocol

Firmware emits structured line packets:

- MARS_PKT for primary mission frame
- MARS_VAL for compact live values
- MARS_ALERT for state transitions and hazard events
- MARS_SUMMARY for periodic mission summary

These packets are parsed and visualized by mars_dashboard.py.

## Dashboard Capabilities

- Serial connect and live status indicators
- Mission state, sector, sensor cards, and trend plots
- Motor control panel with directional buttons and speed slider
- USB gamepad control with visual command highlighting
- Camera source selection and optional CV overlays
- Motion hazard score transmission to firmware

## Setup

## 1) Firmware (Arduino IDE)

- Open mars_rover.ino in Arduino IDE
- Select your board and serial port
- Install required libraries if needed:
  - rgb_lcd (if LCD enabled)
  - Seeed BMP280 or Adafruit BMP280 (if barometer enabled and not ornamental mode)
- Upload firmware to your Arduino-compatible board

## 2) Desktop Dashboard (macOS)

Create and activate virtual environment:

python3 -m venv .venv
source .venv/bin/activate

Install Python dependencies:

pip install pyserial opencv-python pygame pillow

Run dashboard:

python mars_dashboard.py

Optional full camera probe at startup:

MARS_ENABLE_CAMERA=1 python mars_dashboard.py

## Operation Flow

1. Power rover and connect Arduino by USB
2. Start dashboard
3. Select correct serial port and baud 115200
4. Click Connect
5. Use Motor Control panel or enable gamepad
6. Observe live telemetry and mission state transitions

## Configuration Notes

Firmware toggles near the top of mars_rover.ino:

- USE_LCD
- USE_BAROMETER
- BAROMETER_ORNAMENTAL
- USE_GROVE_BAROMETER
- USE_MOISTURE_SENSOR
- USE_ULTRASONIC_SINGLE_PIN
- FRONT_DRIVE_ONLY
- FRONT_LEFT_INVERTED
- FRONT_RIGHT_INVERTED
- SWAP_LEFT_RIGHT_COMMANDS

## Troubleshooting

Dashboard cannot connect:

- Confirm port and baud 115200
- Close any other serial monitor using the same port

Motors move opposite direction:

- Adjust FRONT_LEFT_INVERTED, FRONT_RIGHT_INVERTED, or SWAP_LEFT_RIGHT_COMMANDS

No camera image:

- Verify camera permissions on macOS
- Use MARS_ENABLE_CAMERA=1 for full source probing
- Try alternate source index in dashboard Camera dropdown

Unexpected stop behavior:

- Check command stream continuity from dashboard
- Remember watchdog will force stop if no valid command for 900 ms

## Why NEXUS

NEXUS is designed for exploratory missions where resilience, iteration speed, and safety matter more than preserving a single unit. It is expendable by design, modular by architecture, and open-source for rapid team innovation.

## License

Choose and add your preferred open-source license for distribution.
