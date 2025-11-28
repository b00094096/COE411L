# IoT-Based Multi-Sensor Environmental Monitoring System 🌱

## Project Overview
This project implements a real-time embedded IoT system for environmental monitoring and smart plant care using the STM32 NUCLEO-L476RG microcontroller. The system continuously measures temperature, humidity, soil moisture, ambient light, and human presence, then performs automated control actions such as watering, audio alerts, and color-coded visual feedback.

The system is developed using STM32CubeIDE and integrates multiple sensors and actuators into a single embedded platform using a modular, real-time design approach.

## Features
- Real-time sensor monitoring (0.5–1 Hz sampling)
- Automatic watering using servo motor
- RGB LED plant status indication
- Buzzer alerts for abnormal conditions
- I²C LCD for live system feedback
- FreeRTOS-based task scheduling
- Event flags, queues, and mutex-based synchronization
- Fault detection and safety supervision

## Hardware Components

### Microcontroller
- STM32 NUCLEO-L476RG (ARM Cortex-M4 @ 80 MHz)

### Sensors
- DHT22 — Temperature & Humidity (1-wire)
- Capacitive Soil Moisture Sensor — Moisture level (ADC)
- LDR (Photoresistor) — Ambient light intensity (ADC)
- HC-SR04 Ultrasonic Sensor — Presence detection

### Actuators
- SG90 Servo Motor — Water control
- RGB LED — Plant health indicator
- Passive Buzzer — Audio alerts
- 16×2 I²C LCD — User feedback display

## Pin Mapping Summary

| Component | Interface | Pin |
|------------|----------|-----|
| Soil Moisture | ADC | PA0 |
| LDR | ADC | PA1 |
| DHT22 | GPIO | PB10 |
| HC-SR04 Trig | GPIO | PA8 |
| HC-SR04 Echo | GPIO | PA9 |
| Servo Motor | PWM (TIM2) | PA5 |
| RGB LED | PWM (TIM3) | PB4, PB5, PB0 |
| Buzzer | PWM (TIM4) | PB6 |
| LCD (I2C) | SDA / SCL | PB9 / PB8 |

## Software Architecture

The project is built using STM32CubeIDE and includes FreeRTOS as the operating system framework. At the current stage, the system initializes FreeRTOS but the application logic is executed directly inside `main.c` rather than being distributed into separate FreeRTOS tasks.

All sensor reading, actuator control, and display logic are implemented in a structured firmware design using HAL drivers and timer peripherals. FreeRTOS is included to allow future extension of the system into a multi-tasking architecture (e.g., separate sensor, control, and display tasks).

This design choice keeps the current firmware simple and reliable while allowing scalability in later phases such as IoT integration and advanced automation.


## FreeRTOS Status

FreeRTOS is initialized in the project; however, no application-level tasks, queues, semaphores, or event groups are currently defined. All program logic executes sequentially within `main.c`.

The operating system framework is included for future expansion into a multi-tasking system but is not yet used for real-time task scheduling in this version.

## Project Structure

Core/Src/main.c → Main application code  
Core/Inc → Header files  
Drivers → STM32 HAL drivers  
Middlewares → FreeRTOS middleware  
Flowerpot.ioc → CubeMX pin configuration  
STM32L476RGTX_FLASH.ld → Linker script  
STM32L476RGTX_RAM.ld → RAM linker script  


## How to Build & Run

1. Open STM32CubeIDE
2. Import the project folder
3. Build the project
4. Flash to board using ST-Link
5. Observe LCD updates and system behavior

## Current Status
✔ All hardware integrated  
✔ Software architecture implemented  
✔ Real-time scheduling working  
✔ Alerts and automation functional  

## Future Work
- ESP32 Wi-Fi cloud integration
- Mobile app interface
- IoT dashboard
- Machine learning for pattern detection
- Additional gas and air-quality sensors

## Team Members
- Saif Altamimi – Hardware integration & wiring
- Ahmed Farahat – Firmware & FreeRTOS design
- Mohammed Alattar – Testing, calibration, documentation

## Course
COE411 – Embedded and Cyber Physical Systems  
American University of Sharjah

## Date
28 November 2025

