# IMU 2D Angle Estimation with Kalman Filter

This project implements a **2D angle estimation system** using an **MPU9250 IMU** and a **Kalman filter** for precise orientation tracking. The Kalman filter fuses accelerometer and gyroscope data to reduce noise and provide smooth angle estimation.

## 📌 Features
- **IMU Sensor:** MPU9250 for accelerometer and gyroscope data.
- **Kalman Filter:** Reduces noise and improves angle accuracy.
- **2D Angle Estimation:** Computes roll and pitch angles.
- **Real-time Processing:** Continuously updates angles with sensor data.
- **Optimized Performance:** Lightweight and efficient C++ implementation.

## 📂 Repository Structure
```
├── src/              # Source code files
│   └── main.cpp      # Main program logic
├── include/          # Header files
├── data/             # Example sensor datasets
└── README.md         # Project documentation
```

## 🚀 Getting Started
### 🔧 Requirements
- **Microcontroller:** ESP32, STM32, or Arduino (compatible with I2C)
- **Sensor:** MPU9250 IMU
- **Development Tools:** PlatformIO, Arduino IDE, or STM32CubeIDE

### 🛠️ Setup & Compilation
1. Clone the repository:
   ```bash
   git clone https://github.com/turkEman/IMU_2D_Angle_Kalman_Filter.git
   cd IMU_2D_Angle_Kalman_Filter
   ```
2. Install dependencies:
   ```bash
   sudo apt-get install gcc-arm-none-eabi  # For embedded systems (Linux)
   ```
3. Compile the code:
   ```bash
   make build  # (For STM32 or ESP32)
   ```

## 📊 Kalman Filter Overview
The Kalman filter fuses **gyroscope and accelerometer** data to estimate orientation while correcting for drift.

- **Gyroscope:** Provides angular velocity but drifts over time.
- **Accelerometer:** Measures tilt but is noisy.
- **Kalman Filter:** Combines both to give a stable and accurate angle estimate.

## 🙌 Credits
This Kalman filter implementation is adapted from **Carbon Aeronautics**.  
Original implementation: [CarbonAeronautics/Part-XV-1DKalmanFilter](https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter/)  

## 🏗️ Future Improvements
- Extend to **3D orientation estimation** (quaternions)
- Implement a **Mahony or Madgwick filter** for comparison
- Optimize for **low-power embedded systems**

---
