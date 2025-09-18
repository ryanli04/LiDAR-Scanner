# 📡 LiDAR Scanner Project

## Overview
This project implements a custom-built **LiDAR scanner** using the **MSP432E401Y microcontroller** to map its surroundings. The system combines **hardware control, distance measurement, and real-time data visualization** to demonstrate LiDAR principles and provide hands-on experience with embedded systems.

Designed as a **student-friendly, low-cost project**, it is perfect for learning LiDAR, point cloud generation, and hardware-software integration.

---

## ✨ Features
- 360° scanning with a rotating LiDAR sensor  
- Distance measurement using **VL53L0X Time-of-Flight LiDAR sensor**  
- Real-time **2D point cloud visualization** in Python  
- MSP432E401Y-based motor and sensor control  
- Modular design for easy hardware or software upgrades  

---

## 🔧 Hardware
- **Microcontroller:** MSP432E401Y  
- **LiDAR Sensor:** VL53L0X (I2C)  
- **Motor:** DC motor with L298N driver (continuous rotation)  
- **Other Components:** Breadboard, jumper wires, 3D-printed mount for rotation  

---

## 💻 Software
- **Firmware:** Embedded C (MSP432E401Y)  
- **Visualization:** Python (Open3D+ PySerial)  
- **Communication:** UART over USB  

### Libraries / Tools
- **Embedded C:** MSP432 DriverLib  
- **Python:**  
  - `open3d`  
  - `numpy`  
  - `pyserial`  

