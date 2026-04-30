# GPS + IMU Sensor Fusion using Extended Kalman Filter (EKF)

## Overview

This project implements an **Extended Kalman Filter (EKF)** to fuse **GPS** and **IMU** data for accurate trajectory estimation.
The system addresses the fundamental limitation of inertial sensors—**drift due to double integration**—by continuously correcting motion estimates using GPS measurements.

---

## Objective

To estimate a stable and accurate 2D trajectory by combining:

* **IMU (accelerometer + orientation)** → short-term motion prediction
* **GPS** → long-term position correction

---

## Problem Statement

* IMU-based position estimation suffers from **rapid drift**
* GPS provides **absolute position**, but is **noisy and low-frequency**

👉 A fusion algorithm is required to combine the strengths of both sensors.

---

## Methodology

### 1. Sensor Processing

* Convert GPS (latitude, longitude) → local Cartesian coordinates
* Transform IMU acceleration from **body frame → world frame**
* Apply **gravity compensation**

### 2. EKF Design

**State Vector**

```
[x, y, vx, vy]
```

**Prediction Step**

* Uses IMU acceleration to propagate motion

**Update Step**

* Uses GPS position to correct drift

---

## Key Challenges Solved

* IMU drift due to double integration
* Gravity compensation in moving frame
* Coordinate transformation (body → world)
* Sensor time synchronization
* Outlier rejection in GPS measurements
* Stabilizing EKF with proper noise tuning

---

## Results

| Method     | Behavior                       |
| ---------- | ------------------------------ |
| IMU Only   | Large drift over time          |
| GPS Only   | Noisy trajectory               |
| EKF Fusion | Smooth and accurate trajectory |

The EKF successfully reduces drift while maintaining realistic motion.

---

## Dataset

The dataset used in this project is hosted on Zenodo:

👉 https://zenodo.org/records/6557994

Due to GitHub file size limitations, the dataset is not included in this repository.

---

## Repository Structure

```
code/
 └── final_ekf_gps_imu_json.m

results/
 └── trajectory.png

README.md
```

---

## How to Run

1. Download the dataset from Zenodo
2. Place `json_file` in the project root
3. Run the MATLAB script:

```
final_ekf_gps_imu_json.m
```

---

## Output

* GPS trajectory (noisy)
* IMU-only trajectory (drifting)
* EKF trajectory (smoothed and corrected)

See: `results/trajectory.png`

---

## Technologies Used

* MATLAB
* Sensor Fusion
* Extended Kalman Filter (EKF)
* Inertial Navigation Concepts

---

## Conclusion

This project demonstrates a practical implementation of **sensor fusion for navigation systems**, showing how EKF effectively combines noisy and drifting sensor data to produce a reliable trajectory estimate.

---

## Future Improvements

* Bias estimation for IMU sensors
* Full 3D motion modeling
* Real-time implementation
* Integration with wheel odometry or GNSS velocity

---
