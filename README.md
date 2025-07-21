# 🛸 Custom Drone Firmware

A lightweight, modular **ESP32-based flight controller** firmware designed for custom DIY drones. This project replaces traditional RC transmitters with a **PS5 Bluetooth controller**, integrates real-time telemetry, and supports advanced flight features like **PID stabilization**, **altitude hold**, and **failsafe mechanisms**.

---

## 🚀 Features

- 🎮 **PS5 Bluetooth Controller** – Direct joystick input without RC radio
- ⚙️ **Bidirectional DShot (DShot600)** – ESC control with RPM telemetry
- 📈 **PID Stabilization** – Real-time pitch, roll, yaw correction via IMU
- 🧭 **Altitude Hold** – Hover using BMP180 barometric sensor
- 🛑 **Failsafe System** – Auto-landing on signal loss or low battery
- 🖥️ **Custom Tuning GUI** – Web-based interface for PID and sensor tuning
- 🧩 **Modular Design** – Easy integration with GPS, vision systems, or autonomy modules

---

## 🧰 Tech Stack

- **Microcontroller**: ESP32, ESP8266
- **Sensors**: MPU6050 (IMU), BMP180 (Barometer)
- **ESC Protocol**: DShot600 with telemetry and PWM
- **Controller**: DualShock PS5 (Bluetooth)
- **UI**: Custom web GUI (via ESP32 WebServer)
- **Language**: C++ / Arduino Framework

---

## 📦 Getting Started

### 🔧 Hardware Required

- ESP32 (DevKit v1 recommended)  
- DYS Aria 4-in-1 45A ESC  
- Brushless motors (x4)  
- MPU6050 IMU  
- BMP180 barometer  
- PS5 Controller (Bluetooth)  
- LiPo battery (3S/4S)


## 📊 GUI Preview

- Web-based GUI (similar to Betaflight) for live PID tuning and sensor monitoring  
---

## 🛡️ Safety Notes

- Always test with props **off** first  
- Ensure failsafe is configured before flight  
- Calibrate sensors before every session

---

## 🙌 Acknowledgments

- Inspired by DIY drone community  
- Built for learning, experimentation, and open innovation

---

