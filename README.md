# Smart Proximity Alert System — Desktop GUI

A Python (Tkinter) desktop application that connects to an STM32 NUCLEO-F401RE 
running a VL53L8A1 Time-of-Flight (ToF) sensor over serial (UART), providing 
real-time distance monitoring, zone-based alerts, and live sensor configuration 
— all from a clean, theme-aware desktop interface.

## Features

- **Real-time distance display** — live readings from the VL53L8A1 sensor via serial
- **Zone-based alert system** — Danger / Warning / Caution / Safe zones with 
  color-coded visual feedback and buzzer alerts
- **Dark / Light mode** — full UI re-theme with no hardcoded colors
- **Demo mode** — simulate sensor readings without hardware connected, using 
  the same threshold/zone logic as live mode
- **Auto baud rate** (460800) — matched to STM32 firmware, no manual selection needed
- **Live logging panel** — timestamped TX/RX command log

## What You Can Change in the GUI

### 1. Alert Thresholds (Danger / Warning / Caution)
- Set the distance boundaries for each zone
- Values enforced in strict order: **Danger < Warning < Caution < Safe**
- Invalid combinations are flagged visually before you can apply them
- **Units:** switch between mm / cm / m — values auto-convert
- **Reset to defaults:** Danger 500mm, Warning 1500mm, Caution 2500mm

### 2. Buzzer Behavior
- **Enable/disable** buzzer alerts entirely
- **Alert mode** (Auto / manual — as exposed in the UI)
- **Test buzzer** button to trigger it on demand without a real detection

### 3. Sensor Configuration (sent live to the STM32)
- **Resolution:** e.g. 8×8 zones
- **Timing budget:** 5ms / 10ms / 20ms / 50ms / 100ms
- **Ranging frequency:** 1Hz / 5Hz / 10Hz / 15Hz / 30Hz
  - Available frequency options automatically adjust based on the selected 
    timing budget (the GUI won't let you pick a frequency the sensor's timing 
    budget can't support)

### 4. Sensor Control
- **Start / Stop** ranging on demand
- **Demo mode toggle** — run the full zone/alert logic on simulated data

### 5. Display Preferences
- **Unit selector** for live distance readout (mm / cm / m) — independent 
  from the threshold units
- **Dark / Light theme toggle**

## Hardware & Communication
- **Board:** STM32 NUCLEO-F401RE
- **Sensor:** VL53L8A1 (multi-zone ToF, max range 4000mm)
- **Connection:** Serial/UART, fixed baud rate 460800
- **Protocol:** Simple text commands (`THR:`, `BUZZ:`, `SENSOR:`, `CMD:START`, 
  `CMD:STOP`, `CMD:BUZZ_TEST`) sent to firmware; sensor readings parsed back

## Requirements
