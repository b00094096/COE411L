# Smart Flowerpot 🌱
IoT-Based Multi-Sensor Environmental Monitoring System

## Project Overview
This project is an embedded system for environmental monitoring and smart plant care using the STM32 NUCLEO-L476RG microcontroller. The system reads data from multiple sensors (soil moisture, temperature, humidity, light, and distance) and controls actuators (servo motor, RGB LED, buzzer, LCD) accordingly.

The project is developed using STM32CubeIDE and written in C using HAL drivers. It includes FreeRTOS for future expansion, although application tasks are not currently distributed into RTOS threads in this version.

---

## Features
- Soil moisture sensing with percentage calculation
- Temperature and humidity monitoring using DHT22
- Distance measurement using ultrasonic sensor (HC-SR04)
- Automatic servo-based watering control
- RGB LED plant status indication
- Buzzer warning alerts
- I²C LCD status display
- UART debug output via `printf`
- PWM-controlled actuators
- Microsecond timing using DWT cycle counter

---

## Hardware Used

### Microcontroller
- STM32 NUCLEO-L476RG (ARM Cortex-M4)

### Sensors
- DHT22 – Temperature & Humidity
- Capacitive Soil Moisture Sensor – ADC
- LDR – Light intensity
- HC-SR04 Ultrasonic Sensor – Distance / presence

### Actuators
- SG90 Servo Motor – Motion control
- RGB LED (Common Cathode)
- Passive Buzzer
- I²C LCD Display (16×2 with Backpack)

---

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

---

## Software Architecture

The project is structured as a HAL-based firmware application. All sensor acquisition, actuator control, and LCD handling are implemented inside `main.c`. PWM timers are used for servo motors, LEDs, and the buzzer, while ADC and GPIO are used for analog and digital inputs.

FreeRTOS is initialized but no application tasks are defined yet. The code currently runs sequentially and prepares the project for future expansion into a real-time multitasking architecture.

---

## FreeRTOS Status

FreeRTOS is initialized at kernel level, but no threads, queues, semaphores, or event flags are implemented in this version.

Future versions may separate:
- Sensor reading
- Control logic
- Display update

into independent RTOS tasks.

---

## Project Structure

Core/Src/main.c → Main firmware logic
Core/Inc → Header files
Drivers → STM32 HAL drivers
Middlewares → FreeRTOS source code
Flowerpot.ioc → CubeMX configuration
STM32L476RGTX_FLASH.ld → Linker script
STM32L476RGTX_RAM.ld → RAM linker script


---

## How to Build & Run

1. Open STM32CubeIDE  
2. Import the project folder  
3. Build project  
4. Flash to board using ST-Link  
5. Observe behavior on LCD and outputs

---

## Current Status
✔ Sensors working  
✔ Actuators operational  
✔ LCD communication active  
✔ UART output functional  
✔ PWM control implemented  
✔ FreeRTOS kernel enabled  
✔ System stable  

---

## Future Development
- RTOS task restructuring
- Cloud connectivity (ESP32)
- Mobile dashboard
- AI prediction
- Advanced safety logic
- Additional environmental sensors

---

## Team Members
- Saif Altamimi — Hardware Integration
- Ahmed Farahat — Firmware Development
- Mohammed Alattar — Testing & Documentation

---

## Course
COE411 – Embedded and Cyber Physical Systems  
American University of Sharjah

---

## Date
28 November 2025
