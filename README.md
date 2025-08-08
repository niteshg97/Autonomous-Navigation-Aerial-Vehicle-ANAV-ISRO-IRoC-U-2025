# ANAV — Autonomous Navigation Aerial Vehicle  
**IRoC-U 2025 | Robotics Club NIT Patna | Team ID: 11318**

---

## 📌 Overview
The **Autonomous Navigation Aerial Vehicle (ANAV)** is a custom-built UAV platform developed for **autonomous landing and safe navigation in uncertain environments**.  
Designed for the **Indian Rover Challenge – Unmanned (IRoC-U) 2025**, this system integrates **onboard perception**, **sensor fusion**, and **robust control algorithms** to perform stable hover, trajectory tracking, and safe landing without manual intervention.

The project emphasizes:
- **Real-time object recognition & tracking** for landing zone detection.
- **Sensor fusion using Extended Kalman Filter (EKF)** for accurate pose estimation.
- **Adaptive PID control** for smooth and stable flight.
- **Failsafe and emergency landing protocols** for operational safety.

---

## 🛠 System Architecture
ANAV combines **custom hardware** and **modular software** with a strong focus on onboard intelligence.

### **Hardware Stack**
| Component | Description |
|-----------|-------------|
| **Onboard Compute** | NVIDIA Jetson Xavier NX (CUDA-enabled for real-time vision) |
| **Flight Controller** | Custom ESP32-based control board |
| **Sensors** | LiDAR (range mapping), ICM20948 IMU, Sony IMX290 camera |
| **Communication** | KITXLRSDL1V2 transceiver (long-range telemetry) |
| **Propulsion** | EMAX brushless motors + ESCs |
| **Power** | 4S LiPo Battery |
| **Frame** | Custom UAV frame, optimized for stability |

### **Software Stack**
- **OS / Middleware**: Ubuntu 22.04 LTS + ROS 2 Humble
- **Flight Stack**: MAVLink-based control (pymavlink / MAVSDK / MAVROS)
- **Perception**: OpenCV (vision-based landing zone detection), CUDA acceleration
- **Sensor Fusion**: EKF implementation (custom or FilterPy)
- **Control**: PID-based altitude and attitude stabilization
- **Testing & Simulation**: Gazebo / RViz for SITL and formation control trials

---

## 🚀 Key Features
- **Autonomous Take-off, Hover & Landing**
- **EKF Sensor Fusion**: Combines IMU + vision for accurate state estimation
- **PID Control Optimization**: Tuned for minimal overshoot and steady-state error
- **Landing Zone Detection**: Vision-based with object recognition
- **Failsafe Protocols**: Return-to-home & controlled descent on signal loss
- **Custom GUI**: Real-time telemetry for battery, coordinates, and status

---

## 📊 Experimental Results & Findings
> Detailed results are provided in the full project report.  
> [📄 Download Full Report](https://your-link-to-report.com)

**Highlights:**<img width="576" height="455" alt="Change_detection_UAV" src="https://github.com/user-attachments/assets/132fbbbb-2a5c-4dbd-b189-380177341b17" />

- Stable hover achieved with <±5 cm altitude deviation.
- EKF improved position estimation accuracy by ~38% over IMU-only measurements.
- PID tuning reduced oscillations by 45% in altitude control tests.
- Emergency landing protocol executed within 3 seconds of signal loss.
- Real-time video and telemetry streaming maintained at <120 ms latency.

---

## 📂 Repository Structure
<img src="images/hover_stability.png" width="60%">
<img width="1280" height="832" alt="Screenshot 2025-07-28 at 11 04 53 PM" src="https://github.com/user-attachments/assets/a7a715ef-90b5-4816-ba69-90dadea2771b" />


