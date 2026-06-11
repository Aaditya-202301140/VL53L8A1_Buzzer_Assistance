# VL53L8A1 Buzzer Assistance System

## Overview
Blind assistance system using VL53L8A1 ToF sensor and STM32 NUCLEO-F401RE.

## Features
- Distance measurement using VL53L8A1
- Variable buzzer feedback
  - 2.5m – 1.5m : Slow beep
  - 1.5m – 0.5m : Fast beep
  - < 0.5m : Continuous tone
- PWM-based buzzer control using TIM2

## Hardware
- STM32 NUCLEO-F401RE
- VL53L8A1 ToF Sensor
- Passive Piezo Buzzer
- BD139 Transistor

## Development Environment
- STM32CubeIDE
- STM32CubeMX
- X-CUBE-TOF1
