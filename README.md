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

The firmware is structured as independent FreeRTOS tasks:

### Tasks
- **Sensor Task** – Reads all sensors periodically
- **Control Task** – Compares readings with thresholds and controls outputs
- **Display Task** – Updates LCD screen
- **Safety Task** – Prevents overwatering and detects faults

### Inter-Task Communication
- Queues for sensor data and control commands
- Mutex protecting I²C bus
- Event flags for system state
- Semaphore for watering supervision

## Project Structure

Core/Src/main.c → Main application code  
Core/Inc → Header files  
Drivers → STM32 HAL drivers  
Middlewares → FreeRTOS middleware  
Flowerpot.ioc → CubeMX pin configuration  
STM32L476RGTX_FLASH.ld → Linker script  
STM32L476RGTX_RAM.ld → RAM linker script  


